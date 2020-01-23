from imusim.all import *
import matplotlib.pyplot as plt
import csv
import sys
import os
import pandas as pd
# from imusim.io 
from imusim.io import vicon_csv as vcsv
from imusim.tests.system import smartwatch as sw
# from imusim.tests.system import reality_vicon as rv
# from imusim.tests.system import reality_test as r
# from r import s
# from vicon_csv import loadViconCSVFile

# test = input("Testing csv or reality? \n")
# filename = input("Please choose a csv file to load.")
# csv = loadViconCSVFile(filename)
# filename = "S01_E0.csv"
print("Hola")
marker_filename = "S01_E2_L_1.csv"
imu_filename = "S01_E2_L_1.csv.gz"
csv = vcsv.loadViconCSVFile(marker_filename)
jointNames = vcsv.process_names(vcsv.get_joints(vcsv.read_data(marker_filename)))
# print(dir(vcsv))
# print(dir(vcsv))
# print("Here's a joint \n")
# print(vcsv.get_joint(0))
'''
Accelerometer - Acceleration (m/s^2)
Gyroscope - Rotational Acceleration (rads/s^2)
Magnetometer - mT
'''
def raw_accel_x():
    plt.plot(imu.accelerometer.rawMeasurements.timestamps, imu.accelerometer.rawMeasurements.values[0], 'c-', label='raw x')
def cal_accel_x():
    plt.plot(imu.accelerometer.calibratedMeasurements.timestamps, imu.accelerometer.calibratedMeasurements.values[0], 'r-', label='calibrated x')
def raw_accel_y():
    plt.plot(imu.accelerometer.rawMeasurements.timestamps, imu.accelerometer.rawMeasurements.values[1], '-m', label='raw y')
def cal_accel_y():
    plt.plot(imu.accelerometer.calibratedMeasurements.timestamps, imu.accelerometer.calibratedMeasurements.values[1], '-g', label='calibrated y')
def raw_accel_z():
    plt.plot(imu.accelerometer.rawMeasurements.timestamps, imu.accelerometer.rawMeasurements.values[2], 'y-', label='raw z')
def cal_accel_z():
    plt.plot(imu.accelerometer.calibratedMeasurements.timestamps, imu.accelerometer.calibratedMeasurements.values[2], 'b-', label='calibrated z')
def raw_accel():
    raw_accel_x()
    raw_accel_y()
    raw_accel_z()
def cal_accel():
    cal_accel_x()
    cal_accel_y()
    cal_accel_z()
def raw_gyro_x():
    plt.plot(imu.gyroscope.rawMeasurements.timestamps, imu.gyroscope.rawMeasurements.values[0], 'c-', label='raw x')
def cal_gyro_x():
    plt.plot(imu.gyroscope.calibratedMeasurements.timestamps, imu.gyroscope.rawMeasurements.values[0], 'r-', label='calibrated x')
def raw_gyro_y():
    plt.plot(imu.gyroscope.rawMeasurements.timestamps, imu.gyroscope.rawMeasurements.values[1], 'm-', label='raw y')
def cal_gyro_y():
    plt.plot(imu.gyroscope.calibratedMeasurements.timestamps, imu.gyroscope.rawMeasurements.values[1], 'g-', label='calibrated y')
def raw_gyro_z():
    plt.plot(imu.gyroscope.rawMeasurements.timestamps, imu.gyroscope.rawMeasurements.values[2], 'y-', label='raw z')
def cal_gyro_z():
    plt.plot(imu.gyroscope.calibratedMeasurements.timestamps, imu.gyroscope.rawMeasurements.values[2], 'b-', label='calibrated z')
def raw_gyro():
    raw_gyro_x()
    raw_gyro_y()
    raw_gyro_z()
def cal_gyro():
    cal_gyro_x()
    cal_gyro_y()
    cal_gyro_z()
def raw_mag_x():
    plt.plot(imu.magnetometer.rawMeasurements.timestamps, imu.magnetometer.rawMeasurements.values[0], 'c-', label='raw x')
def cal_mag_x():
    plt.plot(imu.magnetometer.calibratedMeasurements.timestamps, imu.magnetometer.calibratedMeasurements.values[0], 'r-', label='calibrated x')
def raw_mag_y():
    plt.plot(imu.magnetometer.rawMeasurements.timestamps, imu.magnetometer.rawMeasurements.values[1], 'm-', label='raw y')
def cal_mag_y():
    plt.plot(imu.magnetometer.calibratedMeasurements.timestamps, imu.magnetometer.calibratedMeasurements.values[1], 'g-', label='calibrated y')
def raw_mag_z():
    plt.plot(imu.magnetometer.rawMeasurements.timestamps, imu.magnetometer.rawMeasurements.values[2], 'y-', label='raw z')
def cal_mag_z():
    plt.plot(imu.magnetometer.calibratedMeasurements.timestamps, imu.magnetometer.calibratedMeasurements.values[2], 'b-', label='calibrated z')
def raw_mag():
    raw_mag_x()
    raw_mag_y()
    raw_mag_z()
def cal_mag():
    cal_mag_x()
    cal_mag_y()
    cal_mag_z()
def accel():
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.legend()
def gyro():
    plt.xlabel("Time (s)")
    plt.ylabel("Rotational Acceleration (rads/s^2)")
    plt.legend()
def mag():
    plt.xlabel("Time (s)")
    plt.ylabel("Magnetic Field Strength (mT)")
    plt.legend()
def fig(n):
    plt.figure(n, figsize=(100,10))
def save(name):
    plt.savefig(name, bbox_inches='tight')
if "L" in marker_filename:
    imu_marker = "L_RS"
elif "R" in marker_filename:
    imu_marker = "R_RS"
else:
    print("Marker not found.")
    exit
ref_marker = "SS"
def run_sim():
    print("Creating object.")
    imu = Orient3IMU()
    env = Environment()
    print("Configuring calibrator.")
    samplingPeriod = csv.framePeriod
    samples = csv.frameCount
    rotationalVelocity = 20
    calibrator = ScaleAndOffsetCalibrator(env, samples, samplingPeriod, rotationalVelocity)
    calibration = calibrator.calibrate(imu)
    # rotationalVelocity = 0
    print("Preparing simulation.")
    # Multi Marker Trajectory takes a Sampled Position Trajectory and a Sampled Rotation Trajectory
    # trajectory = rv.vicon_test()
    trajectory = \
        MultiMarkerTrajectory(csv.marker(imu_marker), \
            [csv.marker("L_ME"), csv.marker("L_MCP2")], refTime=500.0)
    # trajectory = MultiMarkerTrajectory(csv.marker(imu_marker), [csv.marker(imu_marker)])
    '''
    Splined Position Trajectory takes Sampled Position Trajectory 
    Sampled Position Trajectory takes Position Trajectory
    Position Trajectory takes Abstract Trajectory
    Abstract Trajectory takes ABC
    ABC is an Abstract Base Class

    Splined Position Trajectory relies on rotation because of Offset
    Removing rotation from Offset essentially breaks everything as a bunch of other files rely on the rotation from Offset
    Is there a solution that ISN'T removing rotation from all of the files and forcing them to be solely position compatible?
    (We may want to implement 6D data in the future, so rotations and the resulting gyroscope data would be useful.)
    '''
    # marker = chosen_marker
    # base_traj = PositionTrajectory(marker)
    # sampled_traj = SampledPositionTrajectory(base_traj)
    # sampled_traj = csv.marker(chosen_marker)
    # print("Sampled Position")
    # print(sampled_traj)
    # print("Sampled Rotation?")
    # print([sampled_traj])
    # splined_traj = SplinedPositionTrajectory(sampled_traj)
    # trajectory = splined_traj
    # trajectory = SplinedPositionTrajectory(SampledPositionTrajectory(PositionTrajectory(csv.marker(chosen_marker))))
    splinedTrajectory = SplinedTrajectory(trajectory)
    simDuration = csv.frameCount/csv.frameRate
    sim = Simulation(environment=env)
    imu.simulation = sim
    imu.trajectory = splinedTrajectory
    # imu.trajectory = trajectory
    sim.time = splinedTrajectory.startTime - 5
    print(sim.time)
    print(simDuration)
    # sim.time = 0
    BasicIMUBehaviour(imu, samplingPeriod, calibration, initialTime=sim.time)
    # BasicIMUBehaviour(imu, samplingPeriod)
    sim.run(splinedTrajectory.startTime + simDuration)
    return imu
# sim.run(simDuration)
# times = imu.accelerometer.calibratedMeasurements.timestamps
# xdata = imu.accelerometer.calibratedMeasurements.values[0]
# ydata = imu.accelerometer.calibratedMeasurements.values[1]
# zdata = imu.accelerometer.calibratedMeasurements.values[2]
# print("Calibration scale:")
# print(calibration[imu.accelerometer].scale)
# print("Calibratrion offset:")
# print(calibration[imu.accelerometer].offset)
# print("Creating graphs from accelerometer data.")

# raw_times = imu.accelerometer.rawMeasurements.timestamps
# raw_x = imu.accelerometer.rawMeasurements.values[0]
# raw_y = imu.accelerometer.rawMeasurements.values[1]
# raw_z = imu.accelerometer.rawMeasurements.values[2]
# cal_times = imu.accelerometer.calibratedMeasurements.timestamps
# cal_x = imu.accelerometer.calibratedMeasurements.values[0]
# cal_y = imu.accelerometer.calibratedMeasurements.values[1]
# cal_z = imu.accelerometer.calibratedMeasurements.values[2]

# print(imu.accelerometer.calibratedMeasurements.earliestTime)
# print(imu.accelerometer.calibratedMeasurements.earliestValue)

# fig(1)
# cal_5()
# basics()
# save(chosen_marker + ":cal_5.png")
# print("Created cal_5.")
# print(imu.accelerometer.rawMeasurements._timestampsArray)
# print(imu.accelerometer.calibratedMeasurements._timestampsArray)
# print(imu.accelerometer.rawMeasurements.timestamps)
# print(imu.accelerometer.calibratedMeasurements.timestamps)
# print(imu.accelerometer.rawMeasurements._timestamps)
# print(imu.accelerometer.calibratedMeasurements._timestamps)

# fig(1)
# raw_accel()
# basics()
# save(ref_marker + ":raw_accel.png")
# # # plt.savefig('raw.png', bbox_inches='tight')
# print("Created", ref_marker, "raw accel.")

# fig(2)
# raw_gyro()
# basics()
# save(ref_marker + ":raw_gyro.png")
# print("Created", ref_marker, "raw gyro.")

# fig(3)
# raw_mag()
# basics()
# save(ref_marker + ":raw_mag.png")
# print("Created", ref_marker, "raw mag.")
# in_dir = sys.argv[1]
# df1 = pd.read_csv(in_dir + imu_filename, compression='gzip')
SENSOR_COLUMN = 1
DATA_COLUMNS = np.array([0, 2, 3, 4])

df1 = pd.read_csv(imu_filename, compression='gzip')
X = df1.iloc[:, SENSOR_COLUMN] == 1
s = df1[X]
acc = s.iloc[:, DATA_COLUMNS].values
Y = df1.iloc[:, SENSOR_COLUMN] == 4
y = df1[Y]
gyr = y.iloc[:, DATA_COLUMNS].values
Z = df1.iloc[:, SENSOR_COLUMN] == 2
z = df1[Z]
mag = z.iloc[:, DATA_COLUMNS].values

imu = run_sim()
print(imu.accelerometer.calibratedMeasurements.timestamps[0:600])
# for time in imu.accelerometer.calibratedMeasurements.timestamps:
#     time = time-5000
# for time in imu.gyroscope.calibratedMeasurements.timestamps:
#     time = time-5000
# for time in imu.magnetometer.calibratedMeasurements.timestamps:
#     time = time-5000
sensors1 = {'acc': acc, 'gyr': gyr, 'mag': mag}
sensors2 = {'acc': imu.accelerometer.calibratedMeasurements,\
     'gyr': imu.gyroscope.calibratedMeasurements,\
          'mag': imu.magnetometer.calibratedMeasurements}
# sensors2 = {'acc': imu.accelerometer.rawMeasurements,\
#      'gyr': imu.gyroscope.rawMeasurements,\
#           'mag': imu.magnetometer.rawMeasurements}

sw.plot9x(sensors1, sensors2)
plt.tight_layout()
# plt.savefig(in_dir + "figures/plt_" + imu_filename[0:8] + ".pdf")
# plt.savefig("plt_" + imu_filename[0:8] + ".pdf")
# plt.savefig("plt_" + imu_filename + ".pdf")
plt.savefig("yeehaw.pdf")
plt.close()

# fig(1)
# cal_accel()
# accel()
# save(ref_marker + ":cal_accel.png")
# print("Created", ref_marker, "cal accel.")

# fig(2)
# cal_gyro()
# gyro()
# save(ref_marker + ":cal_gyro.png")
# print("Created", ref_marker, "cal gyro.")

# fig(3)
# cal_mag()
# mag()
# save(ref_marker + ":cal_mag.png")
# print("Created", ref_marker, "cal mag.")

# fig(3)
# raw_accel()
# cal_accel()
# basics()
# save(ref_marker + ":compare_accel.png")
# print("Created", ref_marker, "compare accel.")
# plt.savefig('cal.png', bbox_inches='tight')

# fig(3)
# raw_x()
# cal_x()
# basics()
# save(chosen_marker + ":compare_x.png")
# print("Created compare_x.")

# fig(4)
# raw_y()
# cal_y()
# basics()
# save(chosen_marker + ":compare_y.png")
# print("Created compare_y.")

# fig(5)
# raw_z()
# cal_z()
# basics()
# save(chosen_marker + ":compare_z.png")
# print("Created compare_z.")

# fig(6)
# raw_x()
# raw_y()
# raw_z()
# cal_x()
# cal_y()
# cal_z()
# basics()
# save(chosen_marker + ":raw_vs_cal.png")
# print("Created raw_vs_cal.")

# plt.plot(times, ydata, 'go', label='calibrated y')
# plt.plot(times, zdata, 'ob-', label='calibrated z')
# plt.plot(imu.accelerometer.rawMeasurements.timestamps, imu.accelerometer.rawMeasurements.values)
# i = 500
# plt.plot(int(imu.accelerometer.calibratedMeasurements.timestamps[i:1100] * 100 + 0.1), imu.accelerometer.calibratedMeasurements.values[0][i:1100], label='calibrated x')
# plt.plot(int(imu.accelerometer.calibratedMeasurements.timestamps[i:1100] * 100 + 0.1), imu.accelerometer.calibratedMeasurements.values[1][i:1100], label='calibrated y')
# plt.plot(int(imu.accelerometer.calibratedMeasurements.timestamps[i:1100] * 100 + 0.1), imu.accelerometer.calibratedMeasurements.values[2][i:1100], label='calibrated z')

# while (i < samples):
#     if (i%5 == 0):
# # for i in range(len(imu.accelerometer.calibratedMeasurements.timestamps)):
#         plt.plot(int(imu.accelerometer.calibratedMeasurements.timestamps[i] * 100 + 0.1), imu.accelerometer.calibratedMeasurements.values[0][i], 'r-')
#         plt.plot(int(imu.accelerometer.calibratedMeasurements.timestamps[i] * 100 + 0.1), imu.accelerometer.calibratedMeasurements.values[1][i], 'og')
#         plt.plot(int(imu.accelerometer.calibratedMeasurements.timestamps[i] * 100 + 0.1), imu.accelerometer.calibratedMeasurements.values[2][i], 'ob')
#         # if (i < 1010):
#             # print(int  (imu.accelerometer.calibratedMeasurements.timestamps[i] * 100 + 0.1), " ")
#         # print(imu.accelerometer.calibratedMeasurements.values[i], "\n")
#             # print(imu.accelerometer.calibratedMeasurements.values[0][i], " ")
#         # print(imu.accelerometer.calibratedMeasurements.values[1][i], " ")
#         # print(imu.accelerometer.calibratedMeasurements.values[2][i], "\n")
#     i += 1

# plt.plot(imu.accelerometer.calibratedMeasurements.timestamps, imu.accelerometer.calibratedMeasurements.values)
# for i in range(len(imu.accelerometer.rawMeasurements.values)):
#     # if (i < 30):
#             # print(imu.accelerometer.rawMeasurements.values[i])
#             # print("\n")
#             # print(i)
#     if (i%3 == 0):
#         plt.plot(imu.accelerometer.rawMeasurements.timestamps, imu.accelerometer.rawMeasurements.values[i], label='sim x')
#         print(16, flush = True)
#         # plt.plot(imu.accelerometer.rawMeasurements.timestamps, imuAccel[:,0], label='true x') 
#     elif (i%3 == 1):
#         plt.plot(imu.accelerometer.rawMeasurements.timestamps, imu.accelerometer.rawMeasurements.values[i], label='sim y')
#         # plt.plot(imu.accelerometer.rawMeasurements.timestamps, imuAccel[:,1], label='true y')
#     elif (i%3 == 2):
#         plt.plot(imu.accelerometer.rawMeasurements.timestamps, imu.accelerometer.rawMeasurements.values[i], label='sim z')
#         # plt.plot(imu.accelerometer.rawMeasurements.timestamps, imuAccel[:,2], label='true z')
# plt.xlabel("Time (s)")
# print(17, flush = True)
# plt.ylabel("Acceleration (m/s^2)")
# print(18, flush = True)
# plt.legend()
# plt.xticks(np.arange(1000, 2000, 50))
# print(19, flush = True)

# if (chosen_marker == "L_US"):
#     plt.savefig('accel_l_us.png', bbox_inches='tight')
# elif (chosen_marker == "L_RS"):
#     plt.savefig('accel_l_rs.png', bbox_inches='tight')

# elif (chosen_marker == "SS"):
#     plt.savefig('accel_ss.png', bbox_inches='tight')
# print(20, flush = True)
# plt.figure(2)
# for i in range(len(imu.accelerometer.rawMeasurements.values)):
#     if (i%3 == 0):
#         plt.plot(imu.gyroscope.rawMeasurements.timestamps, imu.gyroscope.rawMeasurements.values[0], label='sim x')
#         # plt.plot(imu.gyroscope.rawMeasurements.timestamps, imuGyro[:,0], label='true x')
#     elif (i%3 == 1):
#         plt.plot(imu.gyroscope.rawMeasurements.timestamps, imu.gyroscope.rawMeasurements.values[1], label='sim y')
#         # plt.plot(imu.gyroscope.rawMeasurements.timestamps, imuGyro[:,1], label='true y')
#     elif (i%3 == 2):
#         plt.plot(imu.gyroscope.rawMeasurements.timestamps, imu.gyroscope.rawMeasurements.values[2], label='sim z')
#         # plt.plot(imu.gyroscope.rawMeasurements.timestamps, imuGyro[:,2], label='true z')
# plt.xlabel("Time (s)")
# plt.ylabel("Rotational Velocity (rad/s)")
# plt.legend()
# if (chosen_marker == "L_US"):
#     plt.savefig('gyro_l_us.png', bbox_inches='tight')
# elif  (chosen_marker == "L_RS"):
#     plt.savefig('gyro_l_rs.png', bbox_inches='tight')
# # elif (chosen_marker == "SS"):
# #     plt.savefig('gyro_ss.png', bbox_inches='tight')
# print(1, dir(r_test))
print("Reality test initiated.")
# q = r.testAgainstReality()
print("Reality test complete.")
# print(q)
# print("Do the thing!")
# r_test.thingy()
# print("The thing has been done!")
# print("And now for the real test...")
# r_test.the_test()
# print("Testing complete.")