#!/usr/bin/env python

import matplotlib.pyplot as mp
from trajectory_tracking import Trajectory_control

if __name__ == "__main__":
    tc=Trajectory_control()
    mp.plot(tc.x_d, tc.y_d)
    mp.show
    pass