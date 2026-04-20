"""
train_pinn.py — Offline training for the PINN contact force predictor.

WORKFLOW:
    1. Run the sim (run_sim.py) and collect CSV logs with contact events
    2. Run this script: python train_pinn.py --data logs/fencebot_YYYYMMDD.csv
    3. Trained weights saved to NeuralNetwork/pinn_contact.pth
    4. Use predict_contact.py for inference

WHAT THE DATA LOOKS LIKE:
    Each row in the CSV = one sim timestep.
    Inputs:  ee_x, ee_y, ee_z, j1, j2, j3, j4, j5, j6
    Outputs: fx, fy, fz, force_mag
"""

import argparse
import csv
from pathlib import Path

import torch
from torch.utils.data import Dataset, DataLoader, random_split

from pinn_contact import PINNContactPredictor, physics_informed_loss


class ContactDataset(Dataset):
    def __init__(self, csv_path: str):
        path = Path(csv_path)
        if not path.exists():
            raise FileNotFoundError(
                f"No CSV found at {csv_path}.\n"
                "Run the sim first: python isaac_env/run_sim.py\n"
                "Then collect contact events by moving the EE into the target cube."
            )

        inputs, outputs = [], []
        with open(path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    x = [
                        float(row["ee_x"]),  float(row["ee_y"]),  float(row["ee_z"]),
                        float(row["j1"]),    float(row["j2"]),    float(row["j3"]),
                        float(row["j4"]),    float(row["j5"]),    float(row["j6"]),
                    ]
                    y = [
                        float(row["force_fx"]), float(row["force_fy"]), float(row["force_fz"]),
                        float(row["force_mag"]),
                    ]
                    # Skip NaN rows (no VR pose received yet)
                    if any(v != v for v in x + y):
                        continue
                    inputs.append(x)
                    outputs.append(y)
                except (KeyError, ValueError):
                    continue

        if not inputs:
            raise ValueError(
                "CSV loaded but no valid rows found. "
                "Make sure the sim ran long enough to produce contact events."
            )

        self.X = torch.tensor(inputs,  dtype=torch.float32)
        self.Y = torch.tensor(outputs, dtype=torch.float32)

        # Normalize inputs — store mean/std for inference time
        self.x_mean = self.X.mean(dim=0)
        self.x_std  = self.X.std(dim=0) + 1e-8
        self.X = (self.X - self.x_mean) / self.x_std

        print(f"Loaded {len(self.X)} timesteps from {csv_path}")
        contact_rows = (self.Y[:, 3] > 0.5).sum().item()
        print(f"  Contact events (>0.5N): {int(contact_rows)} / {len(self.X)} "
              f"({100*contact_rows/len(self.X):.1f}%)")

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]


def train(args):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Training on: {device}")

    dataset = ContactDataset(args.data)
    n_train = int(0.8 * len(dataset))
    n_val   = len(dataset) - n_train
    train_set, val_set = random_split(dataset, [n_train, n_val])

    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True)
    val_loader   = DataLoader(val_set,   batch_size=args.batch_size, shuffle=False)

    model     = PINNContactPredictor(input_dim=9, hidden_dim=64).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=50, gamma=0.5)

    best_val_loss = float("inf")
    out_dir = Path("NeuralNetwork")
    out_dir.mkdir(exist_ok=True)

    print(f"\nTraining {args.epochs} epochs")
    print(f"{'Epoch':>6} | {'Train':>10} | {'Val':>10} | {'Data':>8} | {'Consist':>8} | {'FreeSpc':>8}")
    print("-" * 65)

    for epoch in range(1, args.epochs + 1):
        # Train
        model.train()
        train_total = 0.0
        for X_b, Y_b in train_loader:
            X_b, Y_b = X_b.to(device), Y_b.to(device)
            optimizer.zero_grad()
            fv_pred, fm_pred = model(X_b)
            loss, breakdown = physics_informed_loss(
                fv_pred, fm_pred, Y_b[:, :3], Y_b[:, 3], X_b, model)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            train_total += loss.item()
        scheduler.step()

        # Validate
        model.eval()
        val_total = 0.0
        with torch.no_grad():
            for X_b, Y_b in val_loader:
                X_b, Y_b = X_b.to(device), Y_b.to(device)
                fv_pred, fm_pred = model(X_b)
                loss, breakdown = physics_informed_loss(
                    fv_pred, fm_pred, Y_b[:, :3], Y_b[:, 3], X_b, model)
                val_total += loss.item()

        avg_train = train_total / len(train_loader)
        avg_val   = val_total   / len(val_loader)

        if epoch % 10 == 0 or epoch == 1:
            print(f"{epoch:>6} | {avg_train:>10.4f} | {avg_val:>10.4f} | "
                  f"{breakdown['data_loss']:>8.4f} | "
                  f"{breakdown['consistency_loss']:>8.4f} | "
                  f"{breakdown['freespace_loss']:>8.4f}")

        if avg_val < best_val_loss:
            best_val_loss = avg_val
            torch.save({
                "model_state_dict": model.state_dict(),
                "x_mean": dataset.x_mean,
                "x_std":  dataset.x_std,
                "input_dim": 9,
                "hidden_dim": 64,
            }, out_dir / "pinn_contact.pth")

    print(f"\nDone. Best val loss: {best_val_loss:.4f}")
    print(f"Weights saved to NeuralNetwork/pinn_contact.pth")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data",       type=str,   required=True)
    parser.add_argument("--epochs",     type=int,   default=100)
    parser.add_argument("--batch_size", type=int,   default=64)
    parser.add_argument("--lr",         type=float, default=1e-3)
    args = parser.parse_args()
    train(args)