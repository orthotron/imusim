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
    csv = vcsv.loadViconCSVFile(vicon_file)
    # smartwatch_file = vicon_file + ".gz"
    vicon_object = SplinedMarkerCapture(csv)
    jointNames = vcsv.get_joints(vicon_file)
    if "L" in vicon_file:
        imu_hand = "L_"
        print("IMU markers set to left hand.")
        jointMarkerNames = ["L_ME", "L_RS", "L_MCP2"]
        print("Reference markers set.")
    elif "R" in vicon_file:
        imu_hand = "R_"
        print("IMU marker set to right hand.")
        jointMarkerNames = ["R_ME", "R_RS", "R_MCP2"]
        print("Reference markers set.")
    else:
        print("IMU marker can not be set.")
        return
    imu_marker = vicon_object.marker(imu_hand + "RS")
    refMarkerNames = [vicon_object.marker(ref) \
        for ref in [imu_hand + "ME", imu_hand + "MCP2"]]
    jointMarkerNames = [vicon_object.marker(joint) \
        for joint in [imu_hand + "ME", imu_hand + "RS", imu_hand + "MCP2"]]
    imuTrajectory = \
        MultiMarkerTrajectory(imu_marker, refMarkerNames, refTime=refTime)
    print(imuTrajectory)
    print("We good.")
    joints = []
    for i in range(len(jointMarkerNames)):
        name = jointMarkerNames[i]
    return imuTrajectory