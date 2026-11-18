"""Field sheet Part 4 on Windows, through Quanser's pal library. No ROS.

Needs, on the lab PC:
  - Quanser SDK for Windows (or QUARC), which installs the QArm USB driver
  - the Quanser Python API:
        python -m pip install --upgrade --find-links "%QSDK_DIR%python" ^
            "%QSDK_DIR%python\\quanser_api-<version>-py2.py3-none-any.whl"
  - numpy
  - Quanser Academic Resources in Documents\\Quanser (for pal)

    python part4.py

It reads where the arm is and HOLDS that pose before moving anything, so the
arm does not stand up to home on start-up. Every command is in degrees and
may move no joint more than 15 deg from the previous command. Starting the
script commands the gripper to 0.1, which opens it. Hold the arm before
quitting: it may go limp when the connection closes.
"""

import os
import sys
import time

import numpy as np

sys.path.append(os.path.expanduser(r'~\Documents\Quanser\0_libraries\python'))
from pal.products.qarm import QArm  # noqa: E402

MAX_STEP = np.radians(15.01)


def settle(arm, seconds=1.5):
    """Keep reading for a moment so the arm can move, then return degrees."""
    for _ in range(int(seconds / 0.01)):
        arm.read_std()
        time.sleep(0.01)
    return np.degrees(arm.measJointPosition[:4]).round(1)


def main():
    """Hold the current pose, then run small commands typed by the user."""
    with QArm(hardware=1) as arm:
        if not arm.status:
            sys.exit('Could not open the QArm. Is it plugged in and powered on?')
        arm.read_std()
        q = arm.measJointPosition[:4].copy()
        g = np.array([0.1])
        arm.read_write_std(phiCMD=q, gprCMD=g)          # hold where it is
        print('Holding at', settle(arm), 'deg (base, shoulder, elbow, wrist)')

        while True:
            s = input('base,shoulder,elbow,wrist in deg [,gripper 0.1-0.9]'
                      '  (blank = quit): ')
            if not s.strip():
                break
            try:
                v = [float(x) for x in s.split(',')]
                new = np.radians(v[:4])
                if len(new) != 4:
                    raise ValueError
            except ValueError:
                print('Need 4 numbers, e.g. 10,0,0,0')
                continue
            if np.max(np.abs(new - q)) > MAX_STEP:
                print('Refused: more than 15 deg from the last command.')
                continue
            q = new
            if len(v) > 4:
                g = np.array([v[4]])
            arm.read_write_std(phiCMD=q, gprCMD=g)
            print('Measured:', settle(arm), 'deg')

        input('HOLD THE ARM, then press Enter to disconnect...')


if __name__ == '__main__':
    main()
