# QArm Kinematics Reference

Source for all raw values: Quanser QArm user manual V2.1, Table 2 (link
lengths), Figure 3 (the drawing that defines what each length spans and the
home pose), Section D (workspace) and Table 7 (joint ranges and speeds).

Implemented in `src/qarm_kinematics/` (Python + numpy). The geometry is declared
twice on purpose - once in `qarm_kinematics/robot_params.py` and once in
`qarm_description/urdf/qarm.urdf.xacro` - and `test/test_fk_vs_tf.py` is the
guard that keeps the two in agreement.

---

## 1. Raw physical dimensions (Table 2)

| Symbol | Value    | Meaning                                                    |
|--------|----------|------------------------------------------------------------|
| L1     | 0.14 m   | Worksurface (mounting plane) to the shoulder axis, joint_2 |
| L2     | 0.35 m   | Upper arm, component ALONG the upper arm's own axis        |
| L3     | 0.05 m   | Upper arm, component PERPENDICULAR to it: the elbow kink   |
| L4     | 0.25 m   | Forearm: elbow axis (joint_3) to wrist axis (joint_4)      |
| L5     | 0.15 m   | Wrist axis to `end_effector`, where the fingers start      |
| beta   | 8.13 deg | Tilt of the upper arm off vertical (see below)             |

L1 is already measured from the worksurface, so `base_link` sits on the
worksurface and needs no extra height term.

## 2. Effective (lambda) lengths, and why L3 and beta are the same thing

A DH chain is not built from the raw L values. It is built from the
straight-line distance between consecutive joint AXES. Figure 3 labels those
lambda_1..lambda_3.

```
          elbow
            o
           /|
          / |   L2 = 0.35   (vertical dashed line in Figure 3)
lambda_2 /  |
        / b |   b = beta
       o====+
   shoulder  L3 = 0.05      (horizontal offset at the top of L2)
```

Figure 3 labels **three** of them, lambda_1..lambda_3:

| Symbol   | Definition              | Spans                       | Value      |
|----------|-------------------------|-----------------------------|------------|
| lambda_1 | L1                      | base axis -> shoulder axis  | 0.140000 m |
| lambda_2 | sqrt(L2^2 + L3^2)       | shoulder axis -> elbow axis | 0.353553 m |
| lambda_3 | L4 + L5                 | elbow axis -> `end_effector`| 0.400000 m |

**lambda_3 spans L4 AND L5, straight across joint 4.** The manual's lambda_3
bracket starts at the elbow axis and ends at the frame-4 origin, with the
wrist-roll joint sitting at the L4/L5 boundary partway along it.

Lumping a translation past a joint like that is legal in DH only because
joint 4 rotates about the very axis it slides along: the wrist roll runs
straight down the forearm, so L4 and L5 remain parallel whatever q4 does, and
Rz/Tz about a shared axis commute. That same collinearity is the reason q4
cannot change end-effector position - one fact, two consequences.

An earlier revision of this document split this into lambda_3 = L4 plus an
invented lambda_4 = L5. That notation is not in the manual. The numbers were
always identical (0.25 + 0.15 = 0.40 either way), so correcting it changed no
math - only the symbols, which now match Quanser's.

**L3 and beta are TWO DESCRIPTIONS OF ONE MECHANICAL FEATURE.** L2 and L3 are
the two Cartesian components of the single shoulder-to-elbow vector; lambda_2 is
that vector's length and beta is its angle:

    lambda_2 = sqrt(0.35^2 + 0.05^2) = 0.353553 m
    beta     = atan(0.05 / 0.35)     = 8.1301 deg

Table 2's quoted 8.13 deg is exactly this, to the two decimals it is printed
with. So the kink must be applied **exactly once**. Applying an L3 translation
*and* a beta rotation double-counts it (that was the original bug: it shortened
the effective upper arm from 353.6 mm to 346.5 mm). Omitting the kink entirely
would leave 350.0 mm. Both failure modes are caught by
`test_kink_applied_exactly_once`.

**Kink direction** (confirmed from Figure 3): the elbow is offset toward the
side the arm reaches out over, and a positive shoulder angle tips the arm
further that way. The model matches. Re-check on the hardware.

In the Xacro and in `robot_params.py`, L1..L5 are the only typed-in numbers;
beta and every lambda are computed. A derived value therefore cannot drift away
from the physical one.

## 3. Joint configuration (RPPR) and the zero convention

**Every joint reads 0 at the manual's home pose** (Figure 3, and the "Home
Configuration" of Figure 4b): upper arm up, forearm sticking out horizontally.
That is the pose the real arm reports as zero and the pose Table 7's ranges are
measured from, so angles pass between hardware, URDF, FK and IK unchanged.

| # | Name       | Type | Axis in parent frame | Range (deg) | Positive direction            |
|---|------------|------|----------------------|-------------|-------------------------------|
| 1 | base roll  | R    | +Z of `base_link`    | -170 .. 170 | to confirm on hardware        |
| 2 | shoulder   | P    | +Y of `link_1`       | -85 .. 85   | tips the arm toward the kink  |
| 3 | elbow      | P    | +Y of `link_2`       | -95 .. 75   | forearm down toward the table |
| 4 | wrist roll | R    | +Z of `link_3`       | -160 .. 160 | to confirm on hardware        |

Ranges and the max speed (pi/2 rad/s) are from Table 7. The gripper is
commanded 0..100 % at up to 100 %/s.

**Why the elbow zero moved (Day 3).** The model used to define elbow = 0 as
"arm dead straight". In that convention the manual's home pose sits at
elbow = 81.87 deg, outside the -95 .. +75 range - the model could not reach the
arm's own home pose, which is how the mismatch was found. Base, shoulder and
wrist zeros already matched Quanser's; only the elbow changed. The arm is now
dead straight at

    ELBOW_STRAIGHT = beta - 90 deg = -81.8699 deg

**Still to confirm on hardware:** place the arm in the Figure 4b home
configuration and check all four joints read ~0; check the base and wrist
positive directions.

The first three joints position the end effector. The fourth only orients it:
joint_4 rolls about an axis that runs straight down the forearm, and
`end_effector` sits on that axis, so **q4 cannot change end-effector position**.
Asserted in `test_q4_does_not_move_the_end_effector` (FK) and
`test_q4_does_not_change_position_in_tf` (TF tree).

## 4. URDF chain

The Xacro keeps all elbow geometry in a single place, `joint_3`'s `<origin>`,
which makes a second application of the kink structurally impossible:

| transform                     | content                       |
|-------------------------------|-------------------------------|
| `base_link -> link_1`         | Rz(q1)                        |
| `link_1 -> link_2`            | Tz(L1) Ry(q2)                 |
| `link_2 -> link_3`            | T(L3, 0, L2) Ry(pi/2) Ry(q3)  |
| `link_3 -> link_4`            | Tz(L4) Rz(q4)                 |
| `link_4 -> end_effector`      | Tz(L5)                        |
| `end_effector -> gripper_tip` | Tz(L_gripper)                 |

The Ry(pi/2) in `joint_3` is the elbow's zero: at q3 = 0 the forearm points
along `link_2`'s +x, square to L2. It still contains beta exactly once:
pi/2 = beta + (90 deg - beta), the structural kink plus the 81.87 deg home-pose
offset.

**Gripper (visual only, no effect on the arm kinematics).** The L5 cylinder is
the gripper body, with a palm crossbar at its end. Two straight fingers run
from `end_effector` to `gripper_tip` (L_gripper = 46.4 mm) and slide apart
along the end effector's y axis:

| joint                | type      | range         | note                          |
|----------------------|-----------|---------------|-------------------------------|
| `finger_left_joint`  | prismatic | 0 .. 0.020 m  | 0 = closed, 0.020 = fully open |
| `finger_right_joint` | prismatic | 0 .. 0.020 m  | mimics `finger_left_joint`     |

So the gap between the fingers runs 0 .. 40 mm, and a full stroke takes 1 s
(Table 7: 100 %/s). The real QArm gripper is tendon-driven, two fingers of two
links each, one motor (manual section E.ii); this model only reproduces open
and closed.

## 5. Standard DH table (derived from the chain above, not guessed)

Standard DH convention: the transform from frame i-1 to frame i is

    T_i = Rot(z, theta_i) * Trans(z, d_i) * Trans(x, a_i) * Rot(x, alpha_i)

Derivation. Because (L3, 0, L2) = lambda_2 * (sin beta, 0, cos beta), the kink
offset is itself a rotated translation:

    T(L3, 0, L2) = Ry(beta) Tz(lambda_2) Ry(-beta)

Substituting that into the URDF chain, and letting Rz/Tz about a common axis
commute, gives

    T = Rz(q1) Tz(lambda_1) Ry(q2+beta) Tz(lambda_2)
        Ry(q3 + pi/2 - beta) Tz(lambda_3) Rz(q4)

which is four joints acting on three effective lengths - the DH form:

| i | theta_i            | d_i                   | a_i                 | alpha_i | physical meaning / URDF joint                        |
|---|--------------------|-----------------------|---------------------|---------|------------------------------------------------------|
| 1 | q1                 | lambda_1 = 0.140000   | 0                   | -pi/2   | base roll, `joint_1`; climb L1 to the shoulder axis  |
| 2 | q2 + beta - pi/2   | 0                     | lambda_2 = 0.353553 | 0       | shoulder pitch, `joint_2`; the kink's length is HERE |
| 3 | q3 + pi - beta     | 0                     | 0                   | +pi/2   | elbow pitch, `joint_3`; zero = home pose             |
| 4 | q4                 | lambda_3 = 0.400000   | 0                   | 0       | wrist roll, `joint_4`; tool runs along the roll axis |

**Row 3, read it as theta_3 = (q3 - ELBOW_STRAIGHT) + pi/2.** The bracket is the
elbow's bend measured from "arm dead straight", i.e. from the lambda_2 line that
row 2 just laid down. The + pi/2 is the same frame bookkeeping as before.
Expanding with ELBOW_STRAIGHT = beta - pi/2 gives q3 + pi - beta.

**The -beta in row 3 is not the old double-count bug.** That bug applied the
kink's LENGTH twice and shortened the upper arm. Here no length is touched: a_2
is still lambda_2, and row 3 has d = a = 0. The -beta only says where the
elbow's zero points: row 2 aims along lambda_2 (tilted +beta), but the elbow
measures from L2 (not tilted), so row 3 rotates back by beta.

The constant -pi/2 in theta_2 and the +pi/2 in theta_3 are frame bookkeeping,
not physical angles: they convert "pitch about y" into DH's "rotate about z,
twist about x". alpha_1 = -pi/2 (not +pi/2) so the new z lands on +y of the
base, matching the direction `joint_2`'s axis actually points, which avoids a
sign flip on q2.

PROOF, not assertion: `test_dh_matches_urdf_chain` multiplies out the URDF
origins directly and compares against this table over 200 random in-limit
poses; agreement is within 1e-12. `test_fk_vs_tf.py` then compares against the
TF tree that `robot_state_publisher` computes from the URDF itself; agreement is
within 1e-16 m, well under the 1e-6 tolerance.

## 6. Zero pose = the manual's home pose

At q = [0, 0, 0, 0], `base_link -> end_effector` is:

    translation    = [0.450000, 0.000000, 0.490000]  m
    rotation (RPY) = [0, 90 deg, 0]   (tool pointing straight out along +x)
    gripper_tip    = [0.496447, 0.000000, 0.490000]  m

This is Figure 3 read directly: the drawing brackets L3, L4 and L5 side by side
along the top, so x = 0.05 + 0.25 + 0.15 = 0.45 m, and stacks L1 and L2 up the
side, so z = 0.14 + 0.35 = 0.49 m. Asserted in
`test_zero_pose_is_the_manual_home_pose`.

Pitch is exactly 90 deg, which is RPY gimbal lock; roll is pinned to 0 by
convention. (`rpy_from_transform` was fixed on Day 3 so it reports this as
[0, 90, 0] instead of the equivalent but confusing [180, 90, 180].)

**Arm dead straight**, q = [0, 0, -81.87 deg, 0] - the zero pose before Day 3,
numbers unchanged:

    translation    = [0.106569, 0.000000, 0.885980]  m
    rotation (RPY) = [0, 8.1301 deg, 0]
    gripper_tip    = [0.113137, 0.000000, 0.931960]  m

**The old value [0.106, 0.000, 0.879] is SUPERSEDED.** It came from the version
of the URDF that applied the kink twice, which lost 7.1 mm of effective upper
arm.

## 7. Reach reconciliation

The manual gives two sets of reach numbers. Reach poses have the elbow straight
(q3 = -81.87 deg): horizontal with q2 = 90 deg - beta, vertical with
q2 = -beta.

| direction  | to `end_effector` (computed)                  | Section D | to `gripper_tip` | Section B |
|------------|-----------------------------------------------|-----------|------------------|-----------|
| horizontal | lambda_2 + lambda_3 = 0.753553 m              | 0.7536 m  | 0.800000 m       | 0.800 m   |
| vertical   | lambda_1 + lambda_2 + lambda_3 = 0.893553 m   | 0.8936 m  | 0.940000 m       | 0.940 m   |

**Section D (Workspace) matches the chain to `end_effector` exactly.** That is
independent confirmation of L1..L5, beta and the DH table from Quanser's own
numbers.

**Section B is larger by the same 0.046447 m in both directions.** A wrong link
length would project differently into the horizontal and the vertical, so an
identical offset can only be a constant length appended at the tip.
Decision (Day 3): Section B is measured to the **fingertips**, 46.4 mm past
`end_effector`. The manual does not say this outright. **Measure it on the
hardware**: the distance from the palm face at the end of L5 to the fingertips
should be ~46 mm.

So the Xacro declares `L_gripper = 0.046447` and an ADDITIONAL fixed frame
`gripper_tip`. `end_effector` is unchanged and remains the FK/IK target.
**L1..L5 are correct as given and must not be adjusted to force any reach
number to match.**

## 8. Using it

```bash
# Pose from four joint angles in DEGREES (0 0 0 0 = home pose)
ros2 run qarm_kinematics qarm_fk 0 0 0 0

# Headless FK-vs-TF comparison table
ros2 run qarm_kinematics qarm_validate_fk

# Same comparison as an automatic regression test
colcon test --packages-select qarm_kinematics && colcon test-result --verbose

# RViz + sliders + gripper toggle
ros2 launch qarm_description display.launch.py

# Open / close the gripper (works with or without the GUI)
ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: true}"    # open
ros2 topic pub --once /gripper/command std_msgs/msg/Bool "{data: false}"   # close
```

The `qarm_gripper` node (in `qarm_description/scripts/`) publishes the finger
positions on `/gripper/joint_states`; `joint_state_publisher_gui` merges them
into `/joint_states` through its `source_list` parameter, so the toggle, not the
slider, drives the fingers.

## 9. Inverse kinematics (next phase - NOT implemented)

Because q4 is decoupled from position, IK splits cleanly:

1. Given a target position p = (x, y, z) in `base_link`, solve q1, q2, q3.
   q1 orients the whole arm's plane, after which it reduces to a two-link
   planar reach problem in lambda_2 = 0.353553 and lambda_3 = 0.400000, with
   the beta offset folded into q2 exactly as in row 2 of the DH table. The
   two-link solution gives the elbow bend measured from straight; convert to
   the joint variable with q3 = bend + ELBOW_STRAIGHT.
2. Given a target orientation, solve q4.

A closed-form solution is expected. Elbow-up / elbow-down will give two solution
families; choose by joint limits or by shortest motion from the current pose.
Figure 5a of the manual also shows the inner workspace boundary, needed for the
"target too close" case.
