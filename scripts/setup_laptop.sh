#!/usr/bin/env bash
# ============================================================================
# QArm ROS 2 - one-shot environment setup for a fresh Ubuntu 24.04 machine
#
# Target: Ubuntu 24.04 (noble) + ROS 2 Kilted. This is what Quanser's own
# QArm ROS 2 packages (qarm_nodes, the hardware driver) require, so it is the
# only combination on which our stack and the real arm can run together.
# ============================================================================
#
#   bash scripts/setup_laptop.sh
#
# Safe to re-run: every step either skips or is idempotent. It will ask for
# your sudo password once, near the start.
#
# WHAT IT DOES NOT DO
#   - it does not clone the repo (you already have it if you are reading this)
#   - it does not touch ~/.bashrc without telling you
#   - it does not install the Quanser hardware driver, which is not public
# ============================================================================

set -o pipefail     # NOTE: deliberately no `set -u`. ROS 2's setup.bash reads
                    # unset variables, so nounset aborts the sourcing.

RED=$'\033[0;31m'; GRN=$'\033[0;32m'; YEL=$'\033[1;33m'; NC=$'\033[0m'
step() { echo; echo "${GRN}==> $*${NC}"; }
warn() { echo "${YEL}WARNING: $*${NC}"; }
die()  { echo "${RED}ERROR: $*${NC}" >&2; exit 1; }

ROS_DISTRO_TARGET=kilted
UBUNTU_TARGET=noble

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR" || die "cannot enter $REPO_DIR"
echo "Workspace: $REPO_DIR"

# ----------------------------------------------------------------------------
step "0. Sanity checks"
# ----------------------------------------------------------------------------
. /etc/os-release
[ "${UBUNTU_CODENAME:-}" = "$UBUNTU_TARGET" ] \
  || die "this project targets Ubuntu 24.04 ($UBUNTU_TARGET); found '${PRETTY_NAME}'.
       ROS 2 Kilted is built for noble, and Quanser's QArm ROS 2 packages
       require Ubuntu 24.04 + Kilted. Install Ubuntu 24.04 and re-run."
echo "  Ubuntu 24.04 ($UBUNTU_TARGET) confirmed"

[ -d src/qarm_description ] && [ -d src/qarm_kinematics ] \
  || die "run this from the repo root; src/qarm_description was not found"
echo "  repo layout looks right"

# ----------------------------------------------------------------------------
step "1. ROS 2 apt repository"
# ----------------------------------------------------------------------------
if [ -f /etc/apt/sources.list.d/ros2.list ] \
   || ls /etc/apt/sources.list.d/ros2-apt-source.* >/dev/null 2>&1; then
  echo "  already configured, skipping"
else
  echo "  adding packages.ros.org"
  sudo apt update
  sudo apt install -y software-properties-common curl gnupg lsb-release
  sudo add-apt-repository -y universe
  sudo curl -sSL \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    || die "could not fetch the ROS signing key. If this URL has moved,
       follow the current instructions at
       https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html
       and then re-run this script."
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

# ----------------------------------------------------------------------------
step "2. Install packages from requirements-apt.txt"
# ----------------------------------------------------------------------------
# This is a big download (~2 GB for ros-kilted-desktop). Expect 5-20 minutes
# on a first run depending on the connection.
[ -f requirements-apt.txt ] || die "requirements-apt.txt is missing"
PKGS=$(grep -vE '^\s*#|^\s*$' requirements-apt.txt | tr '\n' ' ')
echo "  packages: $PKGS"
sudo apt update
# shellcheck disable=SC2086
sudo apt install -y $PKGS || die "apt install failed - see the output above"

# ----------------------------------------------------------------------------
step "3. rosdep"
# ----------------------------------------------------------------------------
# rosdep derives dependencies from the package.xml files, independently of
# the hand-maintained list above. If it installs anything extra here, then
# requirements-apt.txt has drifted and should be updated.
if [ ! -d /etc/ros/rosdep/sources.list.d ]; then
  sudo rosdep init || warn "rosdep init failed; it may already be initialised"
fi
rosdep update || warn "rosdep update failed (network?); continuing"
# shellcheck disable=SC1091
source "/opt/ros/$ROS_DISTRO_TARGET/setup.bash" \
  || die "/opt/ros/$ROS_DISTRO_TARGET/setup.bash not found; the apt install did not complete"
rosdep install --from-paths src --ignore-src -y --rosdistro "$ROS_DISTRO_TARGET" \
  || warn "rosdep reported unresolved dependencies; check the output"

# ----------------------------------------------------------------------------
step "4. Build"
# ----------------------------------------------------------------------------
colcon build || die "colcon build failed"
# shellcheck disable=SC1091
source install/setup.bash

# ----------------------------------------------------------------------------
step "5. Verify - this is the part that matters"
# ----------------------------------------------------------------------------
# A successful build is NOT evidence the environment is right. These checks
# compare real numbers against the Quanser manual.

echo
echo "  --- colcon test ---"
colcon test || warn "colcon test returned nonzero"
colcon test-result --verbose | tail -20
TEST_RC=${PIPESTATUS[0]}

echo
echo "  --- URDF parses ---"
xacro src/qarm_description/urdf/qarm.urdf.xacro > /tmp/qarm_check.urdf \
  || die "xacro failed to generate the URDF"
check_urdf /tmp/qarm_check.urdf | head -12

echo
echo "  --- ros2_control block generates ---"
xacro src/qarm_description/urdf/qarm.urdf.xacro ros2_control:=true \
      > /tmp/qarm_control.urdf || die "ros2_control URDF failed to generate"
echo "    plugin: $(grep -o '<plugin>[^<]*</plugin>' /tmp/qarm_control.urdf)"

echo
echo "  --- forward kinematics at the home pose ---"
ros2 run qarm_kinematics qarm_fk 0 0 0 0 || warn "qarm_fk failed"

echo
echo "  --- IK validation table ---"
ros2 run qarm_kinematics qarm_validate_ik || warn "qarm_validate_ik failed"

echo
echo "  --- FK against the live TF tree ---"
ros2 run qarm_kinematics qarm_validate_fk || warn "qarm_validate_fk failed"

# ----------------------------------------------------------------------------
step "DONE"
# ----------------------------------------------------------------------------
cat <<'EOF'

Every new shell needs these two lines:

    source /opt/ros/kilted/setup.bash      # put this in ~/.bashrc
    source install/setup.bash              # must be run from the repo root

To make the first one automatic:

    echo 'source /opt/ros/kilted/setup.bash' >> ~/.bashrc

Expected results from step 5, for comparison:
    colcon test ........ 45 tests, 0 errors, 0 failures, 0 skipped
    check_urdf ......... base_link -> link_1 -> link_2 -> link_3 -> link_4
                         -> end_effector -> {finger_left, finger_right,
                         gripper_tip}
    home pose FK ....... (qarm_fk 0 0 0 0: upper arm up, forearm level)
                         end_effector xyz 0.450000 0 0.490000
                         gripper_tip  xyz 0.496447 0 0.490000
    IK validation ...... all 15 checks PASS, including
                         "exactly at max reach  ... reach 0.753553 m" and
                         "vertical max reach    ... z = 0.893553 m"
                         (end_effector max reach; add the 0.046447 m
                         fingertip offset to get the manual's 0.800 / 0.940)
    FK vs TF ........... all 16 rows PASS

If the numbers differ, STOP. The geometry is validated against the Quanser
manual to 1e-16 m, so a mismatch means the environment is wrong, not the arm.
EOF

exit "${TEST_RC:-0}"
