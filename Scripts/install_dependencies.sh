#!/usr/bin/env bash
#
# install_dependencies.sh
# -----------------------
# One-shot installer for FENCE-BOT dependencies on Ubuntu 22.04 + ROS2 Humble.
# Does NOT install Isaac Lab (follow the NVIDIA docs separately) and does NOT
# set up the Unity/SteamVR side of the VR rig.
#
# Idempotent: safe to re-run.
#
# Usage:
#     cd FENCE-BOT
#     bash Scripts/install_dependencies.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# --- pretty printing ------------------------------------------------------
c_reset=$'\033[0m'; c_blue=$'\033[1;34m'; c_green=$'\033[1;32m'; c_yellow=$'\033[1;33m'; c_red=$'\033[1;31m'
step() { echo -e "\n${c_blue}==>${c_reset} $*"; }
ok()   { echo -e "${c_green}✓${c_reset} $*"; }
warn() { echo -e "${c_yellow}!${c_reset} $*"; }
die()  { echo -e "${c_red}✗${c_reset} $*" >&2; exit 1; }

# --- sanity checks --------------------------------------------------------
step "Sanity checks"
if ! command -v lsb_release >/dev/null 2>&1; then
    warn "lsb_release not found; skipping OS version check"
else
    UBUNTU_VER="$(lsb_release -rs)"
    if [[ "${UBUNTU_VER}" != "22.04" ]]; then
        warn "Expected Ubuntu 22.04, found ${UBUNTU_VER}. Continuing anyway."
    else
        ok "Ubuntu 22.04 detected"
    fi
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
    warn "ROS_DISTRO not set. Source ROS2 Humble first, or run:"
    warn "    source /opt/ros/humble/setup.bash"
fi

# --- apt packages ---------------------------------------------------------
step "Installing apt packages"
sudo apt-get update -qq
sudo apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rclcpp \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    build-essential \
    cmake \
    git
ok "apt packages installed"

# --- rosdep ---------------------------------------------------------------
step "Resolving ROS2 dependencies via rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || true
fi
rosdep update
cd "${REPO_ROOT}"
rosdep install --from-paths src --ignore-src -r -y || warn "rosdep reported missing pieces — check output above"
ok "rosdep complete"

# --- python packages (for Isaac Lab side + data / LNN notebook) -----------
step "Installing Python packages"
python3 -m pip install --upgrade pip
python3 -m pip install \
    numpy \
    scipy \
    sympy \
    pandas \
    matplotlib \
    scikit-learn \
    torch \
    jupyter
ok "Python packages installed"

# --- build the ROS2 workspace --------------------------------------------
step "Building ROS2 workspace"
cd "${REPO_ROOT}"
colcon build --packages-select vr_robot_sim --symlink-install
ok "colcon build complete"

# --- post-install message -------------------------------------------------
cat <<EOF

${c_green}FENCE-BOT dependencies installed.${c_reset}

Next steps:
  1. Source the workspace:
       source ${REPO_ROOT}/install/setup.bash

  2. Export the USD path if not at the default repo location:
       export FENCEBOT_USD_PATH=/abs/path/to/ASEM_V2.usd

  3. Run the pipeline (three terminals):
       # terminal 1 — ROS2 bridge
       ros2 launch vr_robot_sim fence_bot.launch.py

       # terminal 2 — Isaac Lab sim
       python3 ${REPO_ROOT}/isaac_env/run_sim.py

       # terminal 3 — synthetic VR input (swap for real VR when ready)
       python3 ${REPO_ROOT}/Scripts/mock_vr_publisher.py --pattern thrust

See docs/Setup_guide.md and docs/troubleshooting.md for more.
EOF
