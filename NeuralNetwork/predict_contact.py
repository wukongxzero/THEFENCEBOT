"""
predict_contact.py — Inference wrapper for the trained PINN contact predictor.

USAGE (standalone):
    python predict_contact.py --weights NeuralNetwork/pinn_contact.pth

USAGE (from run_sim.py):
    from predict_contact import ContactPredictor
    predictor = ContactPredictor("NeuralNetwork/pinn_contact.pth")
    force_vec, force_mag = predictor.predict(ee_pos, joint_pos)

HOW IT FITS INTO THE CONTROL LOOP:
    Current loop (reactive):
        joint state → IK → sim step → contact sensor reads force (after contact)

    With PINN (predictive):
        joint state → IK → PINN predicts force → sim step → contact sensor confirms
                                ↑
                        1-2 steps of lookahead — useful for modulating strike force
                        or detecting unexpected obstacles before they happen.
"""

import argparse
from pathlib import Path

import torch

from pinn_contact import PINNContactPredictor


class ContactPredictor:
    """
    Thin inference wrapper around the trained PINN.
    Handles loading weights, applying normalization, and running forward pass.
    """

    def __init__(self, weights_path: str, device: str = None):
        self.device = torch.device(
            device if device else ("cuda" if torch.cuda.is_available() else "cpu")
        )

        path = Path(weights_path)
        if not path.exists():
            raise FileNotFoundError(
                f"No weights found at {weights_path}.\n"
                "Train the model first: python train_pinn.py --data logs/fencebot_YYYYMMDD.csv"
            )

        checkpoint = torch.load(path, map_location=self.device)

        self.model = PINNContactPredictor(
            input_dim=checkpoint["input_dim"],
            hidden_dim=checkpoint["hidden_dim"],
        ).to(self.device)
        self.model.load_state_dict(checkpoint["model_state_dict"])
        self.model.eval()

        # Normalization parameters from training data
        self.x_mean = checkpoint["x_mean"].to(self.device)
        self.x_std  = checkpoint["x_std"].to(self.device)

        print(f"[ContactPredictor] Loaded weights from {weights_path}")

    def predict(
        self,
        ee_pos: torch.Tensor,
        joint_pos: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """
        Predict contact force from current robot state.

        Args:
            ee_pos:    (N, 3) end-effector position in world frame (x, y, z)
            joint_pos: (N, 6) joint positions [j1..j6] in radians

        Returns:
            force_vec: (N, 3) predicted force vector [fx, fy, fz] in Newtons
            force_mag: (N, 1) predicted force magnitude in Newtons
        """
        # Build input vector: [ee_x, ee_y, ee_z, j1..j6]
        x = torch.cat([ee_pos, joint_pos], dim=-1).to(self.device)

        # Apply same normalization used during training
        x = (x - self.x_mean) / self.x_std

        with torch.no_grad():
            force_vec, force_mag = self.model(x)

        return force_vec, force_mag

    def is_contact_predicted(self, ee_pos, joint_pos, threshold: float = 0.5) -> bool:
        """
        Convenience method: returns True if predicted force exceeds threshold.
        Useful for a simple contact flag in the control loop.
        """
        _, force_mag = self.predict(ee_pos, joint_pos)
        return bool((force_mag > threshold).any())


# ---------------------------------------------------------------------------
# Standalone test — runs inference on a dummy input to verify the model loads
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=str, default="NeuralNetwork/pinn_contact.pth")
    args = parser.parse_args()

    predictor = ContactPredictor(args.weights)

    # Dummy input — replace with real joint state from sim
    ee_pos    = torch.tensor([[0.5, 0.0, 0.6]])   # near target cube center
    joint_pos = torch.zeros(1, 6)                  # home position

    force_vec, force_mag = predictor.predict(ee_pos, joint_pos)

    print(f"\nDummy inference result:")
    print(f"  Predicted force vector: [{force_vec[0,0]:.3f}, {force_vec[0,1]:.3f}, {force_vec[0,2]:.3f}] N")
    print(f"  Predicted force mag:    {force_mag[0,0]:.3f} N")
    print(f"  Contact predicted:      {predictor.is_contact_predicted(ee_pos, joint_pos)}")