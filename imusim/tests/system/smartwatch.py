import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

TIME_COLUMN = 0
SENSOR_COLUMN = 1
DATA_COLUMNS = np.array([0, 2, 3, 4])
ACCEL_SENSOR = 1
GYRO_SENSOR = 4
MAGNET_SENSOR = 2
INERTIAL_SENSORS = np.array([1,2,4])

def plot9x(sensors1, sensors2, labels = None): # todo: show labelings
    print("Hi!")
    f, axes = plt.subplots(nrows=len(sensors1), ncols=1)
    f.set_size_inches(15,15)
    for i, s in enumerate(sensors1):
        plot3x(sensors1[s], sensors2[s], s, ax=axes[i])

def plot3x(sensor1, sensor2, ylabel, labels = None, ax = None):
    if ax is None:
        plt.figure()
        ax = plt.gca()
    ax.plot(sensor1[:,0], sensor1[:,1], 'r-')
    ax.plot(sensor1[:,0], sensor1[:,2], 'g-')
    ax.plot(sensor1[:,0], sensor1[:,3], 'b-')
    print(sensor2)
    # ax.plot(sensor2[:, 0], sensor2[:, 1], 'r-', linestyle=':')
    # ax.plot(sensor2[:, 0], sensor2[:, 2], 'g-', linestyle=':')
    # ax.plot(sensor2[:, 0], sensor2[:, 3], 'b-', linestyle=':')
    if sensor2 == "gyr":
        print("yeehaw")
        gyro = [[val*100 for val in sensor2.values] for i in sensor2]
        print(gyro)
    # ax.plot(sensor2.timestamps, sensor2.values[0], 'c-')
    # ax.plot(sensor2.timestamps, sensor2.values[1], 'm-')
    # ax.plot(sensor2.timestamps, sensor2.values[2], 'y-')
    ax.plot(sensor2.timestamps, gyro[0], 'c-')
    ax.plot(sensor2.timestamps, gyro[1], 'm-')
    ax.plot(sensor2.timestamps, gyro[2], 'y-')
    ax.set_xlabel('time [s]')
    ax.set_ylabel(ylabel)
    # ax.legend(['x-Good','y-Good','z-Good','x-Bad','y-Bad','z-Bad'])
    ax.legend(['imu-x', 'imu-y', 'imu-z', 'sim-x', 'sim-y', 'sim-z'])



def main(argv=sys.argv[1:]):
    in_dir = argv[0]
    files = []
    for filenames in os.walk(in_dir):
        files.extend(filenames)

    #files.remove('.DS_Store')
    files.sort()

    for f in range(0, len(files), 2):
        file1 = files[f]
        file2 = files[f+1]
        print (file1,file2)
        df1 = pd.read_csv(in_dir + file1, compression='gzip')
        X = df1.iloc[:, SENSOR_COLUMN] == 1
        s = df1[X]
        acc = s.iloc[:, DATA_COLUMNS].values
        Y = df1.iloc[:, SENSOR_COLUMN] == 4
        y = df1[Y]
        gyr = y.iloc[:, DATA_COLUMNS].values
        Z = df1.iloc[:, SENSOR_COLUMN] == 2
        z = df1[Z]
        mag = z.iloc[:, DATA_COLUMNS].values
        sensors1 = {'acc': acc, 'gyr': gyr, 'mag': mag}
        df2 = pd.read_csv(in_dir + file2, compression='gzip')
        X = df2.iloc[:, SENSOR_COLUMN] == 1
        s = df2[X]
        acc = s.iloc[:, DATA_COLUMNS].values
        Y = df2.iloc[:, SENSOR_COLUMN] == 4
        y = df2[Y]
        gyr = y.iloc[:, DATA_COLUMNS].values
        Z = df2.iloc[:, SENSOR_COLUMN] == 2
        z = df2[Z]
        mag = z.iloc[:, DATA_COLUMNS].values
        sensors2 = {'acc': acc, 'gyr': gyr, 'mag': mag}
        plot9x(sensors1, sensors2)
        plt.tight_layout()
        plt.savefig(in_dir + "figures/plt_" + file1[0:8] + ".pdf")
        plt.close()
        #plt.show()


if __name__ == '__main__':
    main()