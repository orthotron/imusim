from __future__ import division
from imusim.io.qualisys_tsv import loadQualisysTSVFile
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
from imusim.io import vicon_csv as vcsv
from imusim.tests.system import reality_test as r

def cal_x():
    plt.plot(imu.accelerometer.calibratedMeasurements.timestamps, imu.accelerometer.calibratedMeasurements.values[0], 'r-', label='calibrated x')
def cal_y():
    plt.plot(imu.accelerometer.calibratedMeasurements.timestamps, imu.accelerometer.calibratedMeasurements.values[1], '-g', label='calibrated y')
def cal_z():
    plt.plot(imu.accelerometer.calibratedMeasurements.timestamps, imu.accelerometer.calibratedMeasurements.values[2], 'b-', label='calibrated z')
def cal():
    cal_x()
    cal_y()
    cal_z()
def basics():
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.legend()
    plt.show()

r.testAgainstReality()
print("Graphs done")

posStdDev = 0.0005
refTime = 1.0
csv = vcsv.loadViconCSVFile("S01_E1_L_2.csv")
spline_3D = SplinedMarkerCapture(csv, positionStdDev=posStdDev)
markers = jointNames = vcsv.process_names(vcsv.get_joints(vcsv.read_data("S01_E1_L_2.csv")))
# marker = input("Choose a marker \n")
marker = "L_US"
print("Creating object.")
imu = Orient3IMU()
env = Environment()
print("Configuring calibrator.")
samplingPeriod = csv.framePeriod
samples = csv.frameCount
rotationalVelocity = 20
calibrator = ScaleAndOffsetCalibrator(env, samples, samplingPeriod, rotationalVelocity)
calibration = calibrator.calibrate(imu)
print("Preparing simulation.")
trajectory = MultiMarkerTrajectory(csv.marker(chosen_marker), [csv.marker(chosen_marker)])
splinedTrajectory = SplinedTrajectory(trajectory)
simDuration = csv.frameCount/csv.frameRate
sim = Simulation(environment=env)
imu.simulation = sim
imu.trajectory = splinedTrajectory
sim.time = 0
BasicIMUBehaviour(imu, samplingPeriod, calibration, initialTime=sim.time)
sim.run(simDuration)

