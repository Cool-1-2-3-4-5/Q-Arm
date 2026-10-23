# QArm Kinematics Reference

Source for all values: Quanser QArm User Manual, Table 2 and Figure 3
(Standard Denavit-Hartenberg assignment).

## Physical parameters

| Symbol | Value       | Meaning                                         |
|--------|-------------|-------------------------------------------------|
| L1     | 0.14 m      | Base (ground) to shoulder (joint 2) axis        |
| L2     | 0.35 m      | Shoulder axis to elbow axis along upper arm     |
| L3     | 0.05 m      | Perpendicular elbow offset (mechanical kink)    |
| L4     | 0.25 m      | Elbow axis to wrist axis along forearm          |
| L5     | 0.15 m      | Wrist axis to end-effector reference point     |
| beta   | 8.13 deg    | Fixed structural angle at elbow                 |

Datasheet net reach in home pose: 0.800 m horizontal, 0.940 m vertical.

## Joint configuration (RPPR)

| # | Name         | Type | Axis in parent frame | Range (rad)       |
|---|--------------|------|----------------------|-------------------|
| 1 | base roll    | R    | +Z of base_link      | [-pi, +pi]        |
| 2 | shoulder     | P    | +Y of link_1         | [-pi/2, +pi/2]    |
| 3 | elbow        | P    | +Y of link_elbow     | [-pi/2, +pi/2]    |
| 4 | wrist roll   | R    | +Z of link_3         | [-pi, +pi]        |

Joint 4 is collinear with the forearm, so q4 does not affect
end-effector position (only orientation).

## Standard DH table (to be verified against datasheet Figure 3)

Standard DH convention: the transform from frame i-1 to frame i is

    T_i = Rot(z, theta_i) * Trans(z, d_i) * Trans(x, a_i) * Rot(x, alpha_i)

| i | theta_i    | d_i  | a_i | alpha_i  | notes                         |
|---|------------|------|-----|----------|-------------------------------|
| 1 | q1         | L1   | 0   | +pi/2    | base roll, lift Z up to joint 2 |
| 2 | q2         | 0    | L2  | 0        | shoulder pitch along upper arm  |
| 3 | q3 + beta  | L3   | 0   | +pi/2    | elbow + fixed beta offset       |
| 4 | q4         | L4+L5| 0   | 0        | wrist roll + tool length        |

TODO: verify this table against Figure 3 before relying on it. The
`+pi/2` entries and the (q3 + beta) coupling are the usual pattern for a
roll-pitch-pitch-roll arm with a 90-degree axis change at the shoulder
and a structural angle at the elbow, but the signs may need to flip
once we check against the figure's drawn axes.

## Forward kinematics (symbolic)

End-effector pose in base_link frame:

    T_be = T_1(q1) * T_2(q2) * T_3(q3) * T_4(q4)

Verification method: for each q = [q1, q2, q3, q4] vector,

    1. Set joint sliders to q in RViz
    2. ros2 run tf2_ros tf2_echo base_link end_effector
    3. Compare the printed 4x4 matrix to T_be(q) computed by hand

Known sanity point: q = [0, 0, 0, 0] gives

    translation = [0.106, 0.000, 0.879]  (meters)
    rotation (RPY) = [0, 8.13 deg, 0]

(The 8.13 deg pitch is beta propagating through the chain.)

## Inverse kinematics (to design)

Because q4 is decoupled from position, IK splits:

    1. Given a target end-effector position p = (x, y, z) in base_link,
       solve for q1, q2, q3 such that FK positions the tip at p.
       This is a 3-variable problem over a 4-link planar-ish chain
       (base rotates the whole chain by q1, so after projecting into
       the arm's own plane it is a 2-link + offset problem).

    2. Given a target end-effector orientation, solve for q4.

Analytical solution expected (closed-form). Elbow-up / elbow-down
branches will give two solution families; pick based on joint limits
or shortest motion from current pose.
