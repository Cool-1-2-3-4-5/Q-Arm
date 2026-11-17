# QArm ROS 2 - setting up a second machine

Getting from a bare Ubuntu 24.04 box to the state the project is currently in.

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
`ros-kilted-desktop` (~2 GB).

## What you need

| | |
|---|---|
| OS | Ubuntu 24.04 (noble). WSL2 is fine for development; the machine wired to the arm must be native (USB) |
| ROS | Kilted Kaiju |
| Disk | ~4 GB |
| Everything else | listed in `requirements-apt.txt` |

Quanser's "ROS 2 For QArm" guide requires Ubuntu 24.04 + ROS 2 Kilted for its
`qarm_nodes` hardware driver, and the Quanser SDK for Linux supports 24.04 as a
minimum. The project was moved from Humble/22.04 to match. On a different
Ubuntu release the script stops rather than installing a mismatched distro.

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
source /opt/ros/kilted/setup.bash   # put this in ~/.bashrc
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
| Home pose, `qarm_fk 0 0 0 0` | `end_effector` 0.450 / 0 / 0.490 m, `gripper_tip` 0.496447 / 0 / 0.490 m |
| IK validation | all 15 checks PASS, including max reach 0.753553 m and vertical max reach 0.893553 m |
| FK vs TF | all 16 rows PASS |

The home pose is upper arm up, forearm level. It is not the maximum reach.
Max reach is checked by the IK validation; adding the 0.046447 m fingertip
offset to its `end_effector` figures gives 0.800 / 0.940 m, the manual's
quoted reach. If your numbers
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

- **The Quanser software.** Not on apt. Install the
  [Quanser SDK for Linux](https://github.com/quanser/quanser_sdk_linux) and
  Quanser Academic Resources from Quanser's GitHub. The QArm ROS 2 driver
  (`qarm_nodes`, node `qarm_hardware`) then lives in
  `~/Documents/Quanser/5_research/qarm/ROS Examples/ros2` and is built
  separately with `colcon build --symlink-install`.
- **USB or Ethernet access to the arm from WSL2.** WSL2 needs `usbipd-win` on
  the Windows side for USB devices, and sits behind NAT for Ethernet. If the
  arm connects over USB, strongly consider native Ubuntu rather than WSL2 on
  the machine that drives it.
