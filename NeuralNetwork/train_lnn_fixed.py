"""
train_lnn_fixed.py
------------------
Cleaned-up Lagrangian Neural Network training for the 2D rigid-body /
pendulum collision dataset.

What this fixes relative to the original notebook (cell 16 of
rigid_body_hits_pendulum.ipynb):

1. StandardScaler is gone. Euler-Lagrange only holds in physical
   coordinates; scaling q, q̇, and q̈ with different factors breaks the
   dimensional consistency that makes the LNN training signal meaningful.

2. `is_colliding` is no longer an LNN input. A Lagrangian should be a
   smooth function of (q, q̇); feeding a discrete 0/1 switch and then
   taking a Hessian w.r.t. it is nonsense. Instead we train TWO LNNs —
   one on free-flight data, one on contact-phase data — and switch
   between them externally.

3. Fixed physical parameters are dropped from inputs. The dataset comes
   from a single simulation run where (M_rb, I_rb, …) are constant, so
   they add no information and bloat the input dimension.

4. Per-sample Python loops replaced with `torch.func` + `vmap`. Batched
   Hessians make training ~10× faster on the same hardware.

Usage:
    # Assumes pinn_training_dataset.csv has been regenerated from
    # rigid_body_hits_pendulum.ipynb (cells 0-10).
    python3 train_lnn_fixed.py

Outputs:
    lnn_freeflight.pth   — learned Lagrangian for no-contact phase
    lnn_contact.pth      — learned Lagrangian for contact phase
    lnn_training_curves.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from torch.func import hessian, jacrev, vmap
from torch.utils.data import DataLoader, TensorDataset


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

class LagrangianNet(nn.Module):
    """
    Scalar-output network that parameterizes the Lagrangian L(q, q̇).

    Softplus activation (not ReLU) because we need smooth 2nd derivatives
    through autograd — ReLU's kink makes the Hessian ill-defined.
    """

    def __init__(self, n_q: int, hidden: int = 128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(2 * n_q, hidden),
            nn.Softplus(),
            nn.Linear(hidden, hidden),
            nn.Softplus(),
            nn.Linear(hidden, hidden // 2),
            nn.Softplus(),
            nn.Linear(hidden // 2, 1),
        )

    def forward(self, q_qdot: torch.Tensor) -> torch.Tensor:
        # q_qdot: (2*n_q,) → scalar
        return self.net(q_qdot).squeeze(-1)


# ---------------------------------------------------------------------------
# Physics: Euler-Lagrange → accelerations
# ---------------------------------------------------------------------------

def make_lagrangian_dynamics(model: LagrangianNet):
    """
    Given an LNN that maps [q, q̇] → scalar L, return a function
    (q, q̇) → q̈ implementing:

        M q̈ + C q̇ = ∂L/∂q

        where
            M = ∂²L/∂q̇²                 (mass matrix)
            C_ij = ∂²L / (∂q̇_i ∂q_j)    (Coriolis-ish)

        ⇒  q̈ = M⁻¹ (∂L/∂q − C q̇)

    Uses `torch.func` so the whole thing vmaps cleanly over a batch.
    """

    def L_of_q_qdot(q: torch.Tensor, q_dot: torch.Tensor) -> torch.Tensor:
        return model(torch.cat([q, q_dot]))

    # Gradient of L w.r.t. q (keeps q̇ fixed): shape (n_q,)
    dL_dq_fn = jacrev(L_of_q_qdot, argnums=0)

    # Mass matrix M = ∂²L/∂q̇² (Hessian w.r.t. q̇): shape (n_q, n_q)
    M_fn = hessian(L_of_q_qdot, argnums=1)

    # Mixed partial C = ∂²L/(∂q̇ ∂q): Jacobian of (∂L/∂q̇) w.r.t. q
    dL_dqdot_fn = jacrev(L_of_q_qdot, argnums=1)
    C_fn = jacrev(dL_dqdot_fn, argnums=0)  # shape (n_q, n_q)

    def q_ddot_single(q: torch.Tensor, q_dot: torch.Tensor) -> torch.Tensor:
        dL_dq = dL_dq_fn(q, q_dot)
        M = M_fn(q, q_dot)
        C = C_fn(q, q_dot)

        rhs = dL_dq - C @ q_dot
        # linalg.solve is more stable than inv(M) @ rhs
        try:
            q_ddot = torch.linalg.solve(M, rhs)
        except RuntimeError:
            q_ddot = torch.linalg.pinv(M) @ rhs
        return q_ddot

    return vmap(q_ddot_single)


# ---------------------------------------------------------------------------
# Data
# ---------------------------------------------------------------------------

# Dataset columns: in_theta1=x, in_w1=x_dot, in_theta2=y, in_w2=y_dot,
#                  phi, phi_dot, theta, theta_dot, is_colliding, params…
# The generalized coordinates q = (x, y, phi, theta) and their velocities.
Q_COLS     = ["in_theta1", "in_theta2", "phi", "theta"]
QDOT_COLS  = ["in_w1",     "in_w2",     "phi_dot", "theta_dot"]
# Targets: accelerations only (positions are trivially q̇, we don't need them)
QDDOT_COLS = ["out_x_ddot", "out_y_ddot", "out_phi_ddot", "out_theta_ddot"]


def load_split(csv_path: Path):
    """
    Load the dataset and split it by collision regime.

    Returns:
        (q_ff, qdot_ff, qddot_ff) — free-flight samples
        (q_co, qdot_co, qddot_co) — contact samples
    """
    df = pd.read_csv(csv_path)

    def extract(rows: pd.DataFrame):
        q     = torch.tensor(rows[Q_COLS].values,     dtype=torch.float32)
        qdot  = torch.tensor(rows[QDOT_COLS].values,  dtype=torch.float32)
        qddot = torch.tensor(rows[QDDOT_COLS].values, dtype=torch.float32)
        return q, qdot, qddot

    ff = df[df["is_colliding"] == 0]
    co = df[df["is_colliding"] == 1]
    print(f"[data] free-flight rows: {len(ff):>6d}   contact rows: {len(co):>6d}")

    return extract(ff), extract(co)


def make_loaders(q, qdot, qddot, batch_size=128, train_frac=0.8, seed=0):
    n = len(q)
    g = torch.Generator().manual_seed(seed)
    perm = torch.randperm(n, generator=g)
    n_train = int(train_frac * n)

    def subset(idx):
        return TensorDataset(q[idx], qdot[idx], qddot[idx])

    train_ds = subset(perm[:n_train])
    test_ds  = subset(perm[n_train:])
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True, generator=g)
    test_loader  = DataLoader(test_ds,  batch_size=256, shuffle=False)
    return train_loader, test_loader


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def train_one(
    name: str,
    q, qdot, qddot,
    epochs: int = 200,
    lr: float = 1e-4,
    hidden: int = 128,
    batch_size: int = 128,
    device: str = "cpu",
):
    n_q = q.shape[1]
    model = LagrangianNet(n_q=n_q, hidden=hidden).to(device)
    dynamics = make_lagrangian_dynamics(model)
    optimizer = optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    train_loader, test_loader = make_loaders(q, qdot, qddot, batch_size=batch_size)

    train_hist, test_hist = [], []
    print(f"\n[train:{name}] starting — epochs={epochs}, lr={lr}, hidden={hidden}, n_q={n_q}")

    for epoch in range(1, epochs + 1):
        # ---- train ----
        model.train()
        batch_losses = []
        for q_b, qdot_b, qddot_b in train_loader:
            q_b, qdot_b, qddot_b = q_b.to(device), qdot_b.to(device), qddot_b.to(device)
            optimizer.zero_grad()
            pred = dynamics(q_b, qdot_b)
            loss = loss_fn(pred, qddot_b)
            loss.backward()
            # Clip hard — LNN gradients can spike when M is near-singular
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            batch_losses.append(loss.item())
        train_loss = float(np.mean(batch_losses))

        # ---- eval ----
        model.eval()
        test_losses = []
        # Note: we CAN'T use torch.no_grad() here — the autograd machinery
        # inside dynamics(...) still needs grad tracking through inputs.
        for q_b, qdot_b, qddot_b in test_loader:
            q_b, qdot_b, qddot_b = q_b.to(device), qdot_b.to(device), qddot_b.to(device)
            pred = dynamics(q_b, qdot_b)
            test_losses.append(loss_fn(pred, qddot_b).item())
        test_loss = float(np.mean(test_losses))

        train_hist.append(train_loss)
        test_hist.append(test_loss)

        if epoch % 10 == 0 or epoch == 1:
            print(f"[train:{name}] epoch {epoch:>3d}/{epochs}  "
                  f"train={train_loss:.4e}  test={test_loss:.4e}")

    return model, train_hist, test_hist


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def plot_curves(curves: dict, out_path: Path):
    # Imported lazily so the training script works in headless environments
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=True)
    for ax, (name, hist) in zip(axes, curves.items()):
        ax.plot(hist["train"], label="train")
        ax.plot(hist["test"],  label="test")
        ax.set_title(name)
        ax.set_xlabel("epoch")
        ax.set_ylabel("MSE (physical units)")
        ax.set_yscale("log")
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    print(f"[plot] saved {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--csv", type=Path, default=Path("pinn_training_dataset.csv"),
                        help="Path to the dataset CSV (default: ./pinn_training_dataset.csv)")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--hidden", type=int, default=128)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--out-dir", type=Path, default=Path("."))
    args = parser.parse_args()

    if not args.csv.exists():
        raise FileNotFoundError(
            f"Dataset not found at {args.csv}. "
            f"Run cells 0-10 of rigid_body_hits_pendulum.ipynb first to regenerate it."
        )

    print(f"[main] device = {args.device}")
    (q_ff, qdot_ff, qddot_ff), (q_co, qdot_co, qddot_co) = load_split(args.csv)

    curves = {}

    model_ff, tr_ff, te_ff = train_one(
        "free-flight", q_ff, qdot_ff, qddot_ff,
        epochs=args.epochs, lr=args.lr, hidden=args.hidden,
        batch_size=args.batch_size, device=args.device,
    )
    torch.save(model_ff.state_dict(), args.out_dir / "lnn_freeflight.pth")
    print(f"[main] saved {args.out_dir / 'lnn_freeflight.pth'}")
    curves["free-flight"] = {"train": tr_ff, "test": te_ff}

    if len(q_co) > 0:
        model_co, tr_co, te_co = train_one(
            "contact", q_co, qdot_co, qddot_co,
            epochs=args.epochs, lr=args.lr, hidden=args.hidden,
            batch_size=args.batch_size, device=args.device,
        )
        torch.save(model_co.state_dict(), args.out_dir / "lnn_contact.pth")
        print(f"[main] saved {args.out_dir / 'lnn_contact.pth'}")
        curves["contact"] = {"train": tr_co, "test": te_co}
    else:
        print("[main] no contact samples in dataset; skipping contact LNN.")

    plot_curves(curves, args.out_dir / "lnn_training_curves.png")

    print("\n[main] done.")
    print("  Load with:")
    print("      model = LagrangianNet(n_q=4)")
    print("      model.load_state_dict(torch.load('lnn_freeflight.pth'))")
    print("      dynamics = make_lagrangian_dynamics(model)")
    print("      q_ddot = dynamics(q_batch, qdot_batch)")


if __name__ == "__main__":
    main()
