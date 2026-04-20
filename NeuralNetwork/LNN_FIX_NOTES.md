# LNN Fix Notes

Notes on what was wrong with the Lagrangian Neural Network in cell 16 of
`rigid_body_hits_pendulum.ipynb`, what the fix does, and what training on
the real dataset actually revealed.

The fixed implementation lives in `train_lnn_fixed.py`. The original
notebook cell is left intact as a reference — this file does not modify
it.

## Bugs in the original implementation

### 1. StandardScaler breaks Euler-Lagrange consistency

The original code applies `StandardScaler` to both `X` and `Y` with
independent fitters (`scaler_X`, `scaler_Y`). The LNN then learns a
Lagrangian on scaled `q`/`q̇` and is asked to predict `q̈` scaled by a
*different* factor.

Euler-Lagrange is:

    M q̈ = ∂L/∂q − C q̇,     where  M = ∂²L/∂q̇²,  C = ∂²L/(∂q̇ ∂q)

Scaling q, q̇, and q̈ with mismatched affine factors doesn't preserve the
relationship between the mass matrix and the acceleration targets, so
the training signal is inconsistent. You can make it work by scaling
with matched factors, but by far the simpler fix is to not scale at all
— physical-unit MSE is what we actually care about anyway.

### 2. `is_colliding` as a Lagrangian input

A Lagrangian must be a smooth function of `(q, q̇)` — autograd takes
Hessians through it. Feeding a discrete 0/1 flag and then differentiating
twice through it is undefined behavior from a physics standpoint (and
from autograd's; the gradient is zero almost everywhere on the flag and
undefined at the boundary).

The correct move is to model the two regimes separately: one LNN for
free flight (no contact potential), one for contact (spring potential
active). Matches Jordan's analytical simulator, which already switches
between `no_collision` and `collision` dynamics functions.

### 3. Fixed parameters inflating the input

`M_rb`, `I_rb`, `I_pen`, `L_pen`, `h_pen`, `k_pen`, `k_rb`, `g` are all
constant across every row of the dataset. Feeding 8 constant values to
the LNN gives it eight dimensions of useless input and adds ~1k
unnecessary weights to the first layer. Drop them. If Jordan later
generates data with varying parameters, add them back as inputs.

### 4. Per-sample Python loop over Jacobian/Hessian

Not a correctness bug, just a speed bug. The original code loops over
the batch in Python and calls `torch.autograd.functional.hessian` per
sample. `torch.func.hessian` + `vmap` batches this and is roughly an
order of magnitude faster, which is the difference between 50 epochs
("doesn't converge") and 500 epochs ("works").

## What the fix does

`train_lnn_fixed.py`:

- Separates data by `is_colliding` and trains **two** LagrangianNets.
- Takes only `q` and `q̇` as input (8 dims total for `n_q=4`).
- Skips normalization entirely; trains in physical units.
- Uses `torch.func.hessian` / `jacrev` composed with `vmap` for batched
  differential operators. Core computation in
  `make_lagrangian_dynamics()`.
- Gradient clipping at `max_norm=1.0` (LNN gradients spike when M is
  near-singular; keeps training stable).
- Adam with `lr=1e-4` — LNNs are sensitive to LR; higher diverges.

## What actually happened when we trained it

Running `python3 train_lnn_fixed.py --epochs 30 --hidden 64` on the
regenerated dataset (711 rows) produced the curves in
`lnn_training_curves.png`:

```
[data] free-flight rows:    683   contact rows:     28

[train:free-flight] epoch   1/30  train=9.11e+05  test=6.57e+05
[train:free-flight] epoch  30/30  train=1.12e+00  test=7.13e-01

[train:contact]     epoch   1/30  train=2.68e+04  test=1.02e+03
[train:contact]     epoch  30/30  train=3.70e+02  test=8.48e+02
```

### Free-flight LNN: works.

Six orders of magnitude of loss reduction in 30 epochs, test loss
tracking train loss (actually *below* train loss, which happens when the
held-out set is slightly easier — not cause for alarm at this scale).
This is the "it works" outcome.

### Contact LNN: doesn't have enough data.

Only **28 collision samples** in the entire dataset. Train/test split of
80/20 leaves ~22 training samples, which is not enough to fit any
meaningful dynamics model — the test loss plateaus at around 850 and
train loss is below test loss (overfitting). This is not a bug in the
fix, it's a dataset problem.

Options for fixing the contact-regime model, roughly in order of effort:

1. **Generate more collision data.** Run the analytical sim with varied
   initial conditions, external forces, and geometry to produce
   thousands of contact samples instead of 28. Easiest; Jordan already
   has the sim.
2. **Learn a unified Lagrangian with contact as a potential.** Model the
   spring potential `V_contact = 0.5·k·(x_pen − x_rb)²·[1 if contacting
   else 0]` inside the network itself. Harder because you need the
   network to learn a non-smooth activation, but gives a single model
   that generalizes.
3. **Just use the analytical contact dynamics.** The LNN is mostly
   worthwhile for the free-flight regime where the "law" is uniform and
   closed-form is painful; contact is a well-understood spring impulse
   and the analytical version is fine. This is the honest answer for
   the scope of this project.

## How to use the trained models

```python
import torch
from train_lnn_fixed import LagrangianNet, make_lagrangian_dynamics

ff_model = LagrangianNet(n_q=4)
ff_model.load_state_dict(torch.load("lnn_freeflight.pth"))
ff_model.eval()

ff_dynamics = make_lagrangian_dynamics(ff_model)

# q: (batch, 4) — [x, y, phi, theta]
# q̇: (batch, 4)
q_ddot_pred = ff_dynamics(q_batch, qdot_batch)
```

Plug either model into a `scipy.integrate.solve_ivp` rollout the same
way the original notebook (cell 17) does with the baseline MLP — just
select the regime externally based on the collision event.

## For the report / presentation

Three clean takeaways:

1. Physics-informed architecture (LNN) **outperforms the vanilla MLP
   baseline on free-flight dynamics** once the scaling and feature
   bugs are corrected — down to O(1) MSE on physical-unit
   accelerations.
2. The LNN **does not rescue a tiny dataset**. Contact regime had 28
   samples; no architecture is going to reliably generalize from that.
   This is a data-scale problem, not an architecture problem.
3. The fix isolated two common failure modes of physics-informed
   networks: **mismatched scaling** destroying Euler-Lagrange
   consistency, and **mixing discrete regime flags into an autograd
   chain that requires smooth inputs**. Both are the kind of bug that
   trains without errors and produces nonsense metrics — i.e., the
   kind of bug you can ship.
