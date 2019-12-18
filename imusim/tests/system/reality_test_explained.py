"""
Test simulated outputs against real captured sensor data.
"""
# Copyright (C) 2009-2011 University of Edinburgh
#
# This file is part of IMUSim.
#
# IMUSim is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IMUSim is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with IMUSim.  If not, see <http://www.gnu.org/licenses/>.

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

def testAgainstReality():
    # Define dir (empty char instead of __file__, works in jupyter nb).
    dir = path.dirname('')
    # Define refbase and filebase for standing and swinging motions.
    refbase = "stand"
    filebase = "swing"
    # Define magbases for magsweep files.
    magbases = [f for f in ['magsweep1', 'magsweep2']]
    # Create maglookup dictionary.
    maglookup = {
        'Upper Leg IMU' : '66',
        'Orient 8' : '8',
        'Orient 43': '43'}
    # Define constants for the file.
    magSamples = 2000
    refTime = 1.0
    posStdDev = 0.0005
    rotStdDev = 0.004
    # Create SplinedMarkerCapture objects by loading .tsv files.
    ref3D = SplinedMarkerCapture(
        loadQualisysTSVFile(refbase + "_3D.tsv"), positionStdDev=posStdDev)
    ref6D = SplinedMarkerCapture(
        loadQualisysTSVFile(refbase + "_6D.tsv"), rotationStdDev=rotStdDev)
    capture3D = SplinedMarkerCapture(
        loadQualisysTSVFile(filebase + "_3D.tsv"), positionStdDev=posStdDev)
    # Load SensorDataCapture(.sdc) file
    captureSD = SensorDataCapture.load(filebase + ".sdc")
    # Define hip, thigh, knee, shin, ankle to be used later.
    hip, thigh, knee, shin, ankle = \
            ['Hip', 'Thigh', 'Knee Hinge', 'Shin', 'Ankle']
    '''
    Note the differences between all of the lists below, as they will be used
     to create marker and trajectory objects.
    '''
    jointNames = ['Upper Leg', 'Lower Leg', 'Foot']
    jointAbbrevs = ['femur', 'tibia', 'foot']
    orientIDs = ['66', '43', '8'] # as seen in the maglookup dictionary
    jointMarkerNames = [hip, knee, ankle]
    refMarkerNames = [[thigh, knee], [shin, ankle], []]
    imuMarkerNames = \
            [[j + ' IMU - ' + str(i) for i in range(1,4)] for j in jointNames]
    # Define lambda functions that will be used to create markers from lists.
    jointMarkerSets = lambda c: [
        list(map(c.marker, jointMarkerNames)),
        [list(map(c.marker, r)) for r in refMarkerNames],
        [list(map(c.marker, i)) for i in imuMarkerNames]]
    imuMarkerSets = lambda c: [
        [c.marker(i[0]) for i in imuMarkerNames],
        [list(map(c.marker,i[1:])) for i in imuMarkerNames]]
    '''
    Use lambda function to build markers from file and create list of trajectories
     from markers. j is a SplinedMarker3DOF object, r is a list of 2 of these objects (it will be 
     an empty list in the last iteration of the loop), i is a list of 3, and jointRefTrajectories
      is a list of 3 MultiMarkerTrajectory objects.
    '''
    jointRefTrajectories = [MultiMarkerTrajectory(j, r + i, refTime=refTime)
        for j, r, i in zip(*(jointMarkerSets(ref3D)))]
    '''
    Similar to jointRefTrajectories above, but markers are built from capture3D file and m,
     a MultiMarkerTrajectory object, is used later in reference to jointRefTrajectories.
    '''
    jointTrajectories = [
        MultiMarkerTrajectory(j, r + i, refVectors=m.refVectors) \
            for j, r, i, m in \
                zip(*(jointMarkerSets(capture3D) + [jointRefTrajectories]))]
    # Similar to above, p is a SplinedMarker3DOF object, r is a list of 2.
    imuRefTrajectories = [MultiMarkerTrajectory(p, r, refTime=refTime)
        for p, r in zip(*(imuMarkerSets(ref3D)))]
    '''
    Similar to above, p is a SplinedMarker3DOF object, r is a list of 2, m is a 
    MultiMarkerTrajectory referring to imuRefTrajectories.
    '''
    imuVecTrajectories = [MultiMarkerTrajectory(p, r, refVectors=m.refVectors)
        for p, r, m in zip(*(imuMarkerSets(capture3D) + [imuRefTrajectories]))]
    # Create a list of 3 SplinedMarker6DOF objects.
    imuRefMarkers = [ref6D.marker(j + ' IMU') for j in jointNames]
    # Create a list of 3 x, y, z offset arrays. i and j are both MultiMarkerTrajectory objects.
    imuOffsets = [i.position(refTime) - j.position(refTime)
        for i, j in zip(imuRefTrajectories, jointRefTrajectories)]
    # Create a list of 3 Quaternions. t is a MultiMarkerTrajectory, m is a SplinedMarker6DOF.
    imuRotations = [t.rotation(refTime).conjugate * m.rotation(refTime)
        for t, m in zip(imuRefTrajectories, imuRefMarkers)]
    '''
    Create a list of 3 OffsetTrajectory objects using the 3 lists above. v is a MultiMarkerTrajectory, 
    o is an array of x, y, z offsets, and r is a Quaternion.
    '''
    imuTrajectories = [OffsetTrajectory(v, o, r)
        for v, o, r in zip(imuVecTrajectories, imuOffsets, imuRotations)]
    # Create a list of 3 CaptureDevice objects.
    imuData = [captureSD.device(i) for i in orientIDs]
    # Create empty list to be used in for loop.
    joints = []
    '''
    Iterate over jointNames, get name and trajectory from previously defined lists, use 
    the first element to create a model and assign key frames based on the TimeSeries object 
    traj.posMarker.positionKeyFrames. The model is then assigned to joint, rotationKeyFrames 
    are added similarly to position, and the joint becomes the first element in a list of joints.
    All subsequent elemtns will have a parent joint (the last item in the list of joints). It will 
    have assigned trajectories for itself and its parent, as well as positions from these trajectories.
    Finally a SampledJoint object is created with a parent joint, joint name, and an offset. rotationKeyFrames
    are assigned and the joint is added to the list.
    '''
    for i in range(len(jointNames)):
        name = jointNames[i]
        traj = jointTrajectories[i]
        if i == 0:
            model = SampledBodyModel(name)
            model.positionKeyFrames = traj.posMarker.positionKeyFrames
            joint = model
        else:
            parent = joints[-1]
            refTraj = jointRefTrajectories[i]
            parentRefTraj = jointRefTrajectories[i - 1]
            pos = refTraj.position(refTime)
            parentPos = parentRefTraj.position(refTime)
            joint = SampledJoint(joints[-1],name, offset=(pos - parentPos))
        joint.rotationKeyFrames = traj.rotationKeyFrames
        joints.append(joint)
    # Create a splined version of the model
    model = SplinedBodyModel(model)
    # Assign the model's joints to joints (stored as a pre-order tree traversal).
    joints = model.joints
    '''
    Create a list of OffsetTrajectory objectsd. j is a SplinedBodyModel (1st element only) or SplinedJoint,
    o is an offset array, and r is a Quaternion.
    '''
    imuJointTrajectories = [OffsetTrajectory(j, o, r)
        for j, o, r in zip(joints, imuOffsets, imuRotations)]
    # More empty list declarations.
    positionSets = []
    valueSets = []
    for magbase in magbases:
        # Create SensorDataCapture object from .sdc.
        orient = SensorDataCapture.load(magbase + '.sdc')
        # Create MarkerCapture object from .tsv.
        optical = loadQualisysTSVFile(magbase + '_6D.tsv')
        for marker in optical.markers:
            # Create CapturedDevice object from marker.
            device = orient.device(maglookup[marker.id])
            # Create list of magnetometer data separated into 3 lists.
            magData = device.sensorData('magnetometer').values
            # Add position and rotation data to appropriate list.
            # Both positionSets and valueSets are lists of lists.
            positionSets.append(marker.positionKeyFrames.values)
            valueSets.append(
                    marker.rotationKeyFrames.values.rotateVector(magData))
    '''
    Call the hstack function on positionSets and valueSets. This SHOULD return the 
    transpose of each, has no effect when the lists are printed. Unclear.
    '''
    positions = np.hstack(positionSets)
    values = np.hstack(valueSets)
    # Create a boolean array showing if none of the values in the above lists are invalid for each index.
    valid = ~np.any(np.isnan(positions),axis=0) & ~np.any(np.isnan(values),axis=0)
    # Create a list of lists showing the deviation of each value in values from the median value.
    dev = values - np.median(values[:,valid],axis=1).reshape((3,1))
    # int step is equal to the number of valid columns in values divided by magSamples.
    step = np.shape(values[:,valid])[1] // magSamples
    # Create 2 new lists of lists from positions and values using the valid values and a step of 30.
    posSamples = positions[:,valid][:,::step]
    valSamples = values[:,valid][:,::step]
    # Create an instance of Environement and configure its magneticField using posSamples and valSamples.
    environment = Environment()
    environment.magneticField = \
            NaturalNeighbourInterpolatedField(posSamples, valSamples)
    '''
    Create an instance of Simulation with the environment defined above and set its 
    start time to match that of the model.
    '''
    sim = Simulation(environment=environment)
    sim.time = model.startTime
    # Empty list declaration.
    distortIMUs = []
    # Set dt equal to the period of capture3D.
    dt = 1/capture3D.sampled.frameRate
    '''
    Iterate through imuJointTrajectories, create an IdealIMU object for each trajectory, 
    set BasicIMUBehaviour for each IMU using dt, and add the BasicIMUBehaviour object to 
    the distortIMUs list.
    '''
    for traj in imuJointTrajectories:
        platform = IdealIMU(sim, traj)
        distortIMUs.append(BasicIMUBehaviour(platform, dt))
    # Run the simulation with the model's endTime.
    sim.run(model.endTime)
    '''
    Iterate through a list of 3 IMUs. For each of the 3 sensors of each IMU, 
    set a TimeSeries (sim) and a list of 3 lists (true).
    '''
    sensors = ['accelerometer', 'magnetometer', 'gyroscope']
    ylabels = ["Acceleration(m/s^2)", "Rotational Acceleration(rad/s^2)", "Magnetic Field Strength(mT)"]
    for imu in range(3):
        for sensorName in sensors:
            # Create an empty list of 3 lists to store the calibrated sim values.
            sim_cal = [[], [], []]
            sim = getattr(distortIMUs[imu].imu,sensorName).rawMeasurements
            true = imuData[imu].sensorData(sensorName)(sim.timestamps + model.startTime)
            for i in range(3):
                for value in sim.values[i-1]:
                    if sensorName == "accelerometer":
                        sim_cal[i-1].append(value/10)
                    elif sensorName == "magnetometer":
                        sim_cal[i-1].append(value/2)
                    else:
                        sim_cal[i-1].append(value)
            plt.figure()
            plt.plot(sim.timestamps, sim_cal[0], 'c-', label = 'sim x')
            plt.plot(sim.timestamps, sim_cal[1], 'm-', label = 'sim y')
            plt.plot(sim.timestamps, sim_cal[2], 'y-', label = 'sim z')
            plt.plot(sim.timestamps, true[0], 'r-', label = 'true x')
            plt.plot(sim.timestamps, true[1], 'g-', label = 'true y')
            plt.plot(sim.timestamps, true[2], 'b-', label = 'true z')
            plt.title = sensorName
            plt.xlabel("Time (s)")
            plt.ylabel(ylabels[sensors.index(sensorName)])
            plt.legend()
            plt.savefig(sensorName+"_"+str(imu+1)+".pdf", bbox_inches='tight')
            # yield assert_vectors_correlated, sim.values, true, 0.8
    return assert_vectors_correlated(sim.values, true, 0.8)

testAgainstReality()