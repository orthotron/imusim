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
    refTime = 5.0
    # posStdDev = 0.0005
    # rotStdDev = 0.004
    # Load the files.
    ref_file = "S01_cal.csv"
    vicon_file = "S01_E2_L_1.csv"
    cal = vcsv.loadViconCSVFile(ref_file)
    csv = vcsv.loadViconCSVFile(vicon_file)
    # smartwatch_file = vicon_file + ".gz"
    ref_object = SplinedMarkerCapture(cal)
    vicon_object = SplinedMarkerCapture(csv)
    if "L" in vicon_file:
        imu_hand = "L_"
        print("IMU markers set to left hand.")
    elif "R" in vicon_file:
        imu_hand = "R_"
        print("IMU marker set to right hand.")
    else:
        print("IMU marker can not be set.")
        return
    imu_marker = imu_hand + "RS"
    # imu_marker = vicon_object.marker(imu_hand + "RS")
    refMarkerNames = [imu_hand + "ME", imu_hand + "MCP2"]
    # refMarkerNames = [vicon_object.marker(ref) \
    #     for ref in [imu_hand + "ME", imu_hand + "MCP2"]]
    jointMarkerNames = [imu_hand + "ME", imu_hand + "RS", imu_hand + "MCP2"]
    # jointMarkerNames = [vicon_object.marker(joint) \
    #     for joint in [imu_hand + "ME", imu_hand + "RS", imu_hand + "MCP2"]]
    jointMarkerSets = lambda c: [
        list(map(c.marker, jointMarkerNames)),
        list(map(c.marker, refMarkerNames)),
        c.marker(imu_marker)
    ]
    jointRefTrajectories = [MultiMarkerTrajectory(j, r + i, refTime=refTime) \
        for j, r, i in zip(*(jointMarkerSets(ref_object)))]
    imuTrajectory = \
        MultiMarkerTrajectory(imu_marker, refMarkerNames, refTime=refTime)
    # print(imuTrajectory)
    print(jointRefTrajectories)
    print("We good.")
    joints = []
    for i in range(len(jointMarkerNames)):
        name = jointMarkerNames[i]
    return imuTrajectory

vicon_test()