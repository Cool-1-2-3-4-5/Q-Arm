# QArm ROS 2 - setting up a second machine

Getting from a bare Ubuntu 22.04 box to the state the project is currently in.

## Short version

```bash
git clone https://github.com/Cool-1-2-3-4-5/Q-Arm.git ~/workspace/qarm
cd ~/workspace/qarm
bash scripts/setup_laptop.sh
```

That script configures the ROS 2 apt repository, installs everything in
[`requirements-apt.txt`](requirements-apt.txt), runs `rosdep`, builds, and then
**verifies the result against real numbers from the Quanser manual**. It is safe
to re-run. Budget 15–30 minutes, most of it downloading
`ros-humble-desktop` (~2 GB).

## What you need

| | |
|---|---|
| OS | Ubuntu 22.04 (jammy), native or WSL2 |
| ROS | Humble Hawksbill |
| Disk | ~4 GB |
| Everything else | listed in `requirements-apt.txt` |

ROS 2 Humble is tied to jammy. On a different Ubuntu release the script stops
rather than installing a mismatched distro.

## Three things the clone will NOT give you

**1. Uncommitted work.** A clone only contains what was pushed. Before you
leave the other machine:

```bash
git status --short      # must be empty
git log origin/main..HEAD   # must be empty
```

If either prints anything, that work exists on one computer only.

**2. `daily_workflow.txt` and `setup_summary.txt`.** Both are in `.gitignore`,
so they are not in the repository. Copy them across by hand if you want them —
`daily_workflow.txt` in particular holds the start-up and shut-down procedure.
The day summaries (`day2_summary.txt`, `day3_4_summary.txt`) *are* tracked and
will clone normally.

**3. `build/`, `install/` and `log/`.** Build artifacts are correctly ignored.
This is why `setup_laptop.sh` runs `colcon build` — the clone is source only,
and `ros2 run qarm_kinematics ...` will not work until you have built and
sourced.

## Every new terminal

```bash
source /opt/ros/humble/setup.bash   # put this in ~/.bashrc
source install/setup.bash           # from the repo root, every time
```

The second one cannot go in `~/.bashrc` because it is workspace-relative.
Forgetting it produces `Package 'qarm_kinematics' not found`.

## Confirming the environment is actually right

A clean build proves nothing. `setup_laptop.sh` finishes by printing real
measurements; compare them against these:

| Check | Expected |
|---|---|
| `colcon test` | 45 tests, 0 errors, 0 failures, 0 skipped |
| `check_urdf` | `base_link → link_1 → link_2 → link_3 → link_4 → end_effector` |
| Reach, `end_effector` | 0.753553 m horizontal, 0.893553 m vertical |
| Reach, `gripper_tip` | 0.800000 m horizontal, 0.940000 m vertical |
| IK validation | all 15 checks PASS |

The last two reproduce the manual's quoted figures exactly. If your numbers
differ, stop and fix the environment — the geometry is validated to 1e-16 m,
so a mismatch is never the robot's fault.

## Headless machines

Assume no reliable X11 under WSL2. Everything is verifiable without a display:

```bash
ros2 run qarm_kinematics qarm_validate_fk    # FK vs the live TF tree
ros2 run qarm_kinematics qarm_validate_ik    # IK validation table
ros2 run qarm_kinematics qarm_fk 0 0 0 0     # pose from joint angles in degrees
```

RViz (`ros2 launch qarm_description display.launch.py`) is for looking, not for
verifying.

## Not covered by this script

- **The Quanser hardware driver.** Not public, not on apt. Install it from
  Quanser's own distribution on whichever machine is wired to the arm.
- **USB or Ethernet access to the arm from WSL2.** WSL2 needs `usbipd-win` on
  the Windows side for USB devices, and sits behind NAT for Ethernet. If the
  arm connects over USB, strongly consider native Ubuntu rather than WSL2 on
  the machine that drives it.
