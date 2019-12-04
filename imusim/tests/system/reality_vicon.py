from __future__ import division
from imusim.io import vicon_csv as vcsv
from imusim.capture.marker import SplinedMarkerCapture
from imusim.trajectories.multi_marker import MultiMarkerTrajectory
from imusim.trajectories.offset import OffsetTrajectory
from imusim.capture.sensor import SensorDataCapture
from imusim.trajectories.rigid_body import SampledBodyModel, SampledJoint
from imusim.trajectories.rigid_body import SplinedBodyModel
from imusim.platforms.imus import IdealIMU
from imusim.behaviours.imu import BasicIMUBehaviour
from imusim.environment.base import Environment
from imusim.maths.vector_fields import NaturalNeighbourInterpolatedField
from imusim.utilities.time_series import TimeSeries
from imusim.simulation.base import Simulation
from imusim.testing.vectors import assert_vectors_correlated
import numpy as np
from os import path
import matplotlib.pyplot as plt
import pandas as pd

def vicon_test():
    # Define dir (empty char instead of __file__, works in jupyter nb).
    dir = path.dirname('')
    # Define some constants.
    refTime = 1.0
    # posStdDev = 0.0005
    # rotStdDev = 0.004
    # Load the files.
    vicon_file = "S01_E2_L_1.csv"
    # smartwatch_file = vicon_file + ".gz"
    vicon_object = SplinedMarkerCapture(vcsv.loadViconCSVFile(vicon_file))
    jointNames = vcsv.get_joints(vicon_file)
    if "L" in vicon_file:
        imu_marker = "L_RS"
        print("IMU marker set to left hand.")
    elif "R" in vicon_file:
        imu_marker = "R_RS"
        print("IMU marker set to right hand.")
    else:
        print("IMU marker can not be set.")
        return
    # jointMarkerSets = lambda c: [
    #     list(map(c.marker, jointNames))
    # ]
    imuMarkerSets = lambda c: [
        [c.marker(imu_marker)]
    ]
    print(vicon_object)
    # jointTrajectories = [MultiMarkerTrajectory(j, r+i, refTime=refTime)
    #     for j, r, i in zip(*(jointMarkerSets(vicon_object)))
    # ]
    imuVecTrajectories = [MultiMarkerTrajectory(p, refTime=refTime)
        for p in imuMarkerSets(vicon_object)]
    print("We good.")

vicon_test()