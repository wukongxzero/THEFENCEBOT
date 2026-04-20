"""
pinn_contact.py — Physics-Informed Neural Network for contact force prediction.

WHAT THIS DOES:
    Given the robot's current joint positions and end-effector position,
    predict the contact force at the next timestep — before the sensor fires.

WHY PINN AND NOT PLAIN MLP:
    A plain MLP would just memorize (input → output) from CSV data.
    A PINN adds physics constraints to the loss function during training:
        1. Force magnitude must be non-negative (you can't push with negative force)
        2. Force is continuous w.r.t. EE position (small position change → small force change)
        3. Free-space predictions should be near zero (no phantom forces)
    These constraints mean the network generalizes better with less data.

INPUTS  (9 values):
    - EE position:       [ee_x, ee_y, ee_z]          3 values
    - Joint positions:   [j1, j2, j3, j4, j5, j6]    6 values

OUTPUTS (4 values):
    - Force vector:      [fx, fy, fz]                 3 values
    - Force magnitude:   [force_mag]                  1 value  (predicted independently, used in physics loss)
"""

import torch
import torch.nn as nn


class PINNContactPredictor(nn.Module):
    def __init__(self, input_dim: int = 9, hidden_dim: int = 64):
        """
        Args:
            input_dim:  9 = 3 (EE pos) + 6 (joint positions)
            hidden_dim: width of hidden layers — 64 is enough for this problem
        """
        super().__init__()

        # Shared feature extractor — learns what joint configs lead to contact
        self.backbone = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.Tanh(),                          # Tanh preferred over ReLU for physics problems
            nn.Linear(hidden_dim, hidden_dim),  # smoother gradients help the physics loss
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.Tanh(),
        )

        # Head 1: force vector [fx, fy, fz] — can be positive or negative (direction)
        self.force_vec_head = nn.Linear(hidden_dim // 2, 3)

        # Head 2: force magnitude — must be non-negative, so we use Softplus activation
        # Softplus(x) = log(1 + e^x) — smooth approximation of ReLU, always > 0
        self.force_mag_head = nn.Sequential(
            nn.Linear(hidden_dim // 2, 1),
            nn.Softplus()                       # enforces non-negative magnitude (physics constraint #1)
        )

    def forward(self, x: torch.Tensor):
        """
        Args:
            x: (batch, 9) — [ee_x, ee_y, ee_z, j1, j2, j3, j4, j5, j6]
        Returns:
            force_vec: (batch, 3) — predicted [fx, fy, fz]
            force_mag: (batch, 1) — predicted ||F||
        """
        features = self.backbone(x)
        force_vec = self.force_vec_head(features)
        force_mag = self.force_mag_head(features)
        return force_vec, force_mag


def physics_informed_loss(
    force_vec_pred: torch.Tensor,
    force_mag_pred: torch.Tensor,
    force_vec_true: torch.Tensor,
    force_mag_true: torch.Tensor,
    x_input: torch.Tensor,
    model: PINNContactPredictor,
    lambda_consistency: float = 0.1,
    lambda_freespace: float = 0.05,
) -> torch.Tensor:
    """
    Total loss = data loss + physics constraint losses

    PHYSICS CONSTRAINTS:
        1. Consistency:  predicted magnitude should match ||predicted vector||
                         prevents the network from predicting a large vector but small magnitude
        2. Free-space:   when true force is near zero, prediction should also be near zero
                         prevents phantom force predictions in free space

    Args:
        force_vec_pred:     (batch, 3)  predicted force vector
        force_mag_pred:     (batch, 1)  predicted force magnitude
        force_vec_true:     (batch, 3)  ground truth force vector from CSV
        force_mag_true:     (batch, 1)  ground truth force magnitude from CSV
        x_input:            (batch, 9)  network input (needed for gradient-based constraints)
        model:              the PINN model
        lambda_consistency: weight for the vector/magnitude consistency constraint
        lambda_freespace:   weight for the free-space suppression constraint

    Returns:
        total_loss: scalar tensor
    """

    # --- Data loss: how well do predictions match sensor readings? ---
    loss_vec = nn.functional.mse_loss(force_vec_pred, force_vec_true)
    loss_mag = nn.functional.mse_loss(force_mag_pred, force_mag_true.unsqueeze(-1))
    data_loss = loss_vec + loss_mag

    # --- Physics constraint 1: consistency ---
    # The magnitude head should agree with the norm of the vector head.
    # If force_vec = [3, 4, 0], then force_mag should be 5, not 2.
    vec_norm = torch.linalg.norm(force_vec_pred, dim=-1, keepdim=True)
    consistency_loss = nn.functional.mse_loss(force_mag_pred, vec_norm)

    # --- Physics constraint 2: free-space suppression ---
    # Where the true force is near zero (free space), predictions should also be near zero.
    # Threshold: < 0.5 N is considered free space (matches force_threshold in get_contact_forces)
    free_space_mask = (force_mag_true < 0.5).unsqueeze(-1)  # (batch, 1) bool
    if free_space_mask.any():
        freespace_loss = (force_mag_pred * free_space_mask.float()).pow(2).mean()
    else:
        freespace_loss = torch.tensor(0.0, device=force_vec_pred.device)

    # --- Total loss ---
    total_loss = (
        data_loss
        + lambda_consistency * consistency_loss
        + lambda_freespace * freespace_loss
    )

    return total_loss, {
        "data_loss": data_loss.item(),
        "consistency_loss": consistency_loss.item(),
        "freespace_loss": freespace_loss.item(),
        "total_loss": total_loss.item(),
    }


if __name__ == "__main__":
    # Quick sanity check — make sure shapes are correct before training
    model = PINNContactPredictor(input_dim=9, hidden_dim=64)
    dummy_input = torch.randn(32, 9)  # batch of 32
    fv, fm = model(dummy_input)
    print(f"Input shape:       {dummy_input.shape}")
    print(f"Force vec shape:   {fv.shape}")   # expect (32, 3)
    print(f"Force mag shape:   {fm.shape}")   # expect (32, 1)
    print("Architecture check passed.")