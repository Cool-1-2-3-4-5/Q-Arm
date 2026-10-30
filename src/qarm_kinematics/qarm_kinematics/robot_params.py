"""Single source of truth for QArm geometry.

Every number the kinematics code uses lives here and nowhere else. If you need
to change a dimension, change it HERE and in the Xacro, then re-run the FK-vs-TF
test (``test/test_fk_vs_tf.py``) which will fail loudly if the two drift apart.

Source of the raw numbers: Quanser QArm user manual, Table 2 (link lengths) and
Figure 3 (the drawing that defines what each length actually spans).

WHY THERE ARE TWO COPIES OF THE GEOMETRY
----------------------------------------
``qarm_description/urdf/qarm.urdf.xacro`` declares the same physical L1..L5
independently, because a URDF has to stand on its own for RViz, TF and
(later) ros2_control. We deliberately do NOT parse the URDF here - that would
hide bugs by construction, since a wrong URDF would produce an equally wrong
FK and the two would always agree.

Instead the two descriptions are kept independent and the FK-vs-TF test in
Piece 4 is the GUARD: it compares this module's FK against the TF tree that
robot_state_publisher computes from the Xacro. If anyone edits one copy and
forgets the other, that test fails.
"""

import math


# ---------------------------------------------------------------------------
# RAW PHYSICAL DIMENSIONS - Quanser QArm manual, Table 2. Metres.
#
# These five numbers are the ONLY independent inputs. Everything below is
# computed from them, so a derived value can never silently disagree with the
# physical one.
# ---------------------------------------------------------------------------
L1 = 0.14   # worksurface (mounting plane) up to the shoulder axis (joint_2)
L2 = 0.35   # upper arm, component measured ALONG the upper arm's own axis
L3 = 0.05   # upper arm, component measured PERPENDICULAR to it: the elbow kink
L4 = 0.25   # forearm: elbow axis (joint_3) to wrist axis (joint_4)
L5 = 0.15   # wrist axis to end_effector, the gripper MOUNTING reference


# ---------------------------------------------------------------------------
# THE ELBOW KINK - why beta is computed and never typed in.
#
# Figure 3 draws the upper arm as a right triangle:
#
#               elbow
#                 o
#                /|
#               / |   L2 = 0.35   (vertical dashed line in Figure 3)
#     lambda_2 /  |
#             / b |   b = beta
#            o----+
#        shoulder  L3 = 0.05      (horizontal offset at the top of L2)
#
# L2 and L3 are the two CARTESIAN COMPONENTS of one single vector, the
# shoulder -> elbow vector. lambda_2 is that vector's LENGTH; beta is its
# ANGLE off vertical. They describe THE SAME mechanical feature, so applying
# both an L3 translation and a beta rotation double-counts the kink. (That was
# the original bug in the URDF.) Pythagoras and atan2 are two views of one
# triangle, not two separate pieces of hardware.
#
# AGREEMENT CHECK: Table 2 prints beta = 8.13 deg.
#   atan2(0.05, 0.35) = 0.141897 rad = 8.1301 deg.
# Identical to the two decimals the manual gives, which confirms our reading
# of the figure.
# ---------------------------------------------------------------------------
BETA = math.atan2(L3, L2)          # 0.141897 rad == 8.1301 deg


# ---------------------------------------------------------------------------
# EFFECTIVE (lambda) LINK LENGTHS - what the DH chain is actually built from.
#
# A DH link length is the straight-line distance between two consecutive joint
# AXES. Figure 3 labels these lambda_1..lambda_3 - THREE of them, not four.
# For two of the three the raw L value already is that distance; only the
# elbow link differs, because of the kink above.
#
#   lambda_1 = L1                     base axis  -> shoulder axis
#   lambda_2 = sqrt(L2^2 + L3^2)      shoulder axis -> elbow axis  (hypotenuse)
#   lambda_3 = L4 + L5                elbow axis -> end_effector
#
# NOTE ON lambda_3 (verified against the manual's Figure 3 bracket): the
# manual's lambda_3 bracket starts at the elbow axis and runs all the way to
# the frame-4 origin, so it spans L4 AND L5 - straight ACROSS joint 4, whose
# wrist-roll axis sits at the L4/L5 boundary.
#
# Lumping a translation past a joint like that is legal in DH only because
# joint 4 rotates about the very axis it slides along: the wrist roll is
# collinear with the forearm, so L4 and L5 stay parallel no matter what q4
# does, and Rz/Tz about a shared axis commute. That same collinearity is why
# q4 cannot change the end-effector POSITION (see the q4-invariance tests).
#
# An earlier version of this file split this into lambda_3 = L4 and an
# invented lambda_4 = L5. That notation does not exist in the manual. The
# numbers were always identical (0.25 + 0.15 = 0.40 either way), so this
# rename changed no math - only the symbol names, which now match Quanser's.
# ---------------------------------------------------------------------------
LAMBDA_1 = L1
LAMBDA_2 = math.sqrt(L2 * L2 + L3 * L3)   # 0.353553 m
LAMBDA_3 = L4 + L5                        # 0.400000 m, elbow axis -> end_effector

# Kept as a readable alias for DH row 4's d term; identical to LAMBDA_3.
TOOL_LENGTH = LAMBDA_3


# ---------------------------------------------------------------------------
# GRIPPER EXTENSION beyond end_effector.
#
# The manual gives TWO sets of reach numbers:
#   Section D (Workspace)      0.7536 m horizontal, 0.8936 m vertical
#   Section B (Configuration)  0.800  m horizontal, 0.940  m vertical
# Our chain reaches, to end_effector:
#   horizontal  lambda_2 + lambda_3            = 0.753553 m
#   vertical    lambda_1 + lambda_2 + lambda_3 = 0.893553 m
# which is Section D exactly - independent confirmation of the geometry.
#
# Section B is larger by the SAME 0.046447 m in both directions. A link-length
# error would show up differently in the two directions; an identical offset
# can only be a constant length tacked onto the tip. Decision: Section B is
# measured to the FINGERTIPS, 46.4 mm past end_effector (where the fingers
# start). The manual does not say this outright - measure it on the hardware.
# L1..L5 are correct as given and must not be "fixed".
# ---------------------------------------------------------------------------
L_GRIPPER = 0.046447


# ---------------------------------------------------------------------------
# JOINT ZERO CONVENTION - all four joints read 0 at the manual's HOME POSE.
#
# The home pose is drawn in Figure 3 and Figure 4b of the manual: upper arm
# up, forearm sticking out HORIZONTALLY. It is the pose the real arm reports
# as zero and the pose Table 7's joint ranges are measured from, so we use the
# same zero. Angles then pass between hardware, URDF, FK and IK unchanged.
#
# Base, shoulder and wrist zeros were already the same as Quanser's. Only the
# elbow moved: at elbow = 0 the forearm is perpendicular to L2 (the upper
# arm's own axis), NOT in line with the effective upper arm lambda_2. (Before
# this change, elbow = 0 meant "arm dead straight", and the manual's home pose
# sat at elbow = 81.87 deg - outside the old -95..+75 limit, which is how the
# mismatch was found.)
#
# The arm is dead straight - forearm continuing along lambda_2 - when the
# elbow is raised from home by 90 deg - beta:
#
#     ELBOW_STRAIGHT = beta - 90 deg = -81.8699 deg
#
# Positive elbow swings the forearm DOWN toward the table (student's
# statement of the Quanser convention; confirm on hardware).
# ---------------------------------------------------------------------------
ELBOW_STRAIGHT = BETA - math.pi / 2.0      # -1.428899 rad == -81.8699 deg


# ---------------------------------------------------------------------------
# JOINT LIMITS (radians), matching the <limit> tags in the Xacro.
# Source: Quanser QArm user manual, Table 7 ("Joint range"), which states them
# relative to the home pose above. Because our zero IS that home pose, the
# values apply directly with no conversion.
#
# Still to confirm on hardware: with the arm placed in the Figure 4b home
# configuration, the joint readings should all be ~0.
# ---------------------------------------------------------------------------
JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3', 'joint_4')

JOINT_LIMITS = (
    (math.radians(-170.0), math.radians(170.0)),   # joint_1  base roll
    (math.radians(-85.0), math.radians(85.0)),     # joint_2  shoulder pitch
    (math.radians(-95.0), math.radians(75.0)),     # joint_3  elbow pitch
    (math.radians(-160.0), math.radians(160.0)),   # joint_4  wrist roll
)

NUM_JOINTS = len(JOINT_NAMES)

# TF frames of interest.
BASE_FRAME = 'base_link'
EE_FRAME = 'end_effector'        # the FK/IK target
TIP_FRAME = 'gripper_tip'        # fingertip, EE_FRAME + L_GRIPPER along the tool
