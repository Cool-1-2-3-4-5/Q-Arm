# QArm Kinematics Reference

Source for all raw values: Quanser QArm user manual, Table 2 (link lengths) and
Figure 3 (the drawing that defines what each length spans).

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
| L5     | 0.15 m   | Wrist axis to `end_effector`, the gripper mounting face    |
| beta   | 8.13 deg | Tilt of the upper arm off vertical (see below)             |

L1 is already measured from the worksurface, so `base_link` sits on the
worksurface and needs no extra height term.

## 2. Effective (lambda) lengths, and why L3 and beta are the same thing

A DH chain is not built from the raw L values. It is built from the
straight-line distance between consecutive joint AXES. Figure 3 labels those
lambda_1..lambda_4.

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

| Symbol   | Definition              | Value      |
|----------|-------------------------|------------|
| lambda_1 | L1                      | 0.140000 m |
| lambda_2 | sqrt(L2^2 + L3^2)       | 0.353553 m |
| lambda_3 | L4                      | 0.250000 m |
| lambda_4 | L5                      | 0.150000 m |

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

lambda_3 and lambda_4 are collinear - both run along the forearm/tool axis with
no offset between them - so the DH chain only ever needs their sum, 0.40 m.

In the Xacro and in `robot_params.py`, L1..L5 are the only typed-in numbers;
beta and every lambda are computed. A derived value therefore cannot drift away
from the physical one.

## 3. Joint configuration (RPPR)

| # | Name       | Type | Axis in parent frame | Range (deg) |
|---|------------|------|----------------------|-------------|
| 1 | base roll  | R    | +Z of `base_link`    | -170 .. 170 |
| 2 | shoulder   | P    | +Y of `link_1`       | -85 .. 85   |
| 3 | elbow      | P    | +Y of `link_2`       | -95 .. 75   |
| 4 | wrist roll | R    | +Z of `link_3`       | -160 .. 160 |

Ranges come from the Quanser product-page specifications table, stated in
Quanser's joint convention. The symmetric ranges transfer no matter where zero
sits; the asymmetric elbow range assumes Quanser's elbow zero equals ours and
should be confirmed on hardware before being trusted for IK.

The first three joints position the end effector. The fourth only orients it:
joint_4 rolls about an axis that runs straight down the forearm, and
`end_effector` sits on that axis, so **q4 cannot change end-effector position**.
Asserted in `test_q4_does_not_move_the_end_effector` (FK) and
`test_q4_does_not_change_position_in_tf` (TF tree).

## 4. URDF chain

The Xacro keeps all elbow geometry in a single place, `joint_3`'s `<origin>`,
which makes a second application of the kink structurally impossible:

| transform              | content                            |
|------------------------|------------------------------------|
| `base_link -> link_1`  | Rz(q1)                             |
| `link_1 -> link_2`     | Tz(L1) Ry(q2)                      |
| `link_2 -> link_3`     | T(L3, 0, L2) Ry(beta) Ry(q3)       |
| `link_3 -> link_4`     | Tz(L4) Rz(q4)                      |
| `link_4 -> end_effector` | Tz(L5)                           |
| `end_effector -> gripper_tip` | Tz(L_gripper)               |

## 5. Standard DH table (derived from the chain above, not guessed)

Standard DH convention: the transform from frame i-1 to frame i is

    T_i = Rot(z, theta_i) * Trans(z, d_i) * Trans(x, a_i) * Rot(x, alpha_i)

Derivation. Because (L3, 0, L2) = lambda_2 * (sin beta, 0, cos beta), the kink
offset is itself a rotated translation:

    T(L3, 0, L2) = Ry(beta) Tz(lambda_2) Ry(-beta)

Substituting that into the URDF chain, the Ry(-beta) cancels the structural
Ry(beta) that immediately follows it, and Rz/Tz about a common axis commute, so

    T = Rz(q1) Tz(lambda_1) Ry(q2+beta) Tz(lambda_2) Ry(q3)
        Tz(lambda_3+lambda_4) Rz(q4)

which is four joints acting on three effective lengths - the DH form:

| i | theta_i            | d_i                   | a_i               | alpha_i | physical meaning / URDF joint                         |
|---|--------------------|-----------------------|-------------------|---------|-------------------------------------------------------|
| 1 | q1                 | lambda_1 = 0.140000   | 0                 | -pi/2   | base roll, `joint_1`; climb L1 to the shoulder axis   |
| 2 | q2 + beta - pi/2   | 0                     | lambda_2 = 0.353553 | 0     | shoulder pitch, `joint_2`; the kink lives HERE, once  |
| 3 | q3 + pi/2          | 0                     | 0                 | +pi/2   | elbow pitch, `joint_3`; no beta - already spent       |
| 4 | q4                 | lambda_3+lambda_4 = 0.400000 | 0          | 0       | wrist roll, `joint_4`; tool runs along the roll axis  |

The constant -pi/2 and +pi/2 in theta_2 and theta_3 are frame bookkeeping, not
physical angles: they convert "pitch about y" into DH's "rotate about z, twist
about x". alpha_1 = -pi/2 (not +pi/2) so the new z lands on +y of the base,
matching the direction `joint_2`'s axis actually points, which avoids a sign
flip on q2.

PROOF, not assertion: `test_dh_matches_urdf_chain` multiplies out the URDF
origins directly and compares against this table over 200 random in-limit
poses; agreement is within 1e-12. `test_fk_vs_tf.py` then compares against the
TF tree that `robot_state_publisher` computes from the URDF itself; agreement is
within 1e-16 m, well under the 1e-6 tolerance.

## 6. Zero-pose sanity point

At q = [0, 0, 0, 0] the arm is dead straight and tilted by the structural kink,
so `base_link -> end_effector` is:

    translation   = [0.106569, 0.000000, 0.885980]  m
    rotation (RPY) = [0, 8.1301 deg, 0]
    gripper_tip    = [0.113137, 0.000000, 0.931960]  m

**The old value [0.106, 0.000, 0.879] is SUPERSEDED.** It came from the version
of the URDF that applied the kink twice, which lost 7.1 mm of effective upper
arm; the z error (0.886 vs 0.879) is that loss projected onto the vertical.
The 8.13 deg pitch is correct in both: at the zero pose the only rotation left
in the whole chain is beta.

## 7. Reach reconciliation (not a modelling error)

The manual quotes a net reach of 0.800 m horizontal and 0.940 m vertical above
the worksurface. The corrected chain gives, to `end_effector`:

| direction  | pose                  | computed                                      | manual  | gap        |
|------------|-----------------------|-----------------------------------------------|---------|------------|
| horizontal | q2 = 90 deg - beta    | lambda_2+lambda_3+lambda_4 = 0.753553 m       | 0.800 m | 0.046447 m |
| vertical   | q2 = -beta            | lambda_1+lambda_2+lambda_3+lambda_4 = 0.893553 m | 0.940 m | 0.046447 m |

The gap is **identical** in both directions. That is the whole argument: a wrong
link length would project differently into the horizontal and the vertical, so
an identical shortfall can only be a constant length appended at the tip.

Conclusion: `end_effector` is the gripper MOUNTING reference, and there is
~46.4 mm of gripper/fingertip beyond it. The manual measures reach to the
fingertip. **L1..L5 are correct as given and must not be adjusted to force the
reach numbers to match.**

So the Xacro declares `L_gripper = 0.046447` and an ADDITIONAL fixed frame
`gripper_tip`, L_gripper past `end_effector` along the tool axis:

    horizontal to gripper_tip = 0.753553 + 0.046447 = 0.800000 m
    vertical   to gripper_tip = 0.893553 + 0.046447 = 0.940000 m

Both reproduce the manual exactly. `end_effector` is unchanged and remains the
FK/IK target, so existing work and the saved RViz config stay valid;
`gripper_tip` is an addition, not a replacement.

## 8. Using it

```bash
# Pose from four joint angles in DEGREES
ros2 run qarm_kinematics qarm_fk 0 0 0 0

# Headless FK-vs-TF comparison table
ros2 run qarm_kinematics qarm_validate_fk

# Same comparison as an automatic regression test
colcon test --packages-select qarm_kinematics && colcon test-result --verbose
```

## 9. Inverse kinematics (next phase - NOT implemented)

Because q4 is decoupled from position, IK splits cleanly:

1. Given a target position p = (x, y, z) in `base_link`, solve q1, q2, q3.
   q1 orients the whole arm's plane, after which it reduces to a two-link
   planar reach problem in lambda_2 and (lambda_3 + lambda_4), with the beta
   offset folded into q2 exactly as in row 2 of the DH table.
2. Given a target orientation, solve q4.

A closed-form solution is expected. Elbow-up / elbow-down will give two solution
families; choose by joint limits or by shortest motion from the current pose.
