#!/usr/bin/env python
# coding: utf-8

# In[1]:


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
from imusim.io.vicon_csv import loadViconCSVFile
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
import pandas as pd
from os import path
import matplotlib.pyplot as plt


# In[2]:


def testAgainstReality():
    print("Begin testing.")
    # dir = path.dirname(__file__)
    dir = path.dirname('')
    # filebase = path.join(dir, "swing")
    # refbase = path.join(dir, "stand")
    marker_filename = "S01_E2_L_1.csv"
    imu_filename = "S01_E2_L_1.csv.gz"
    # magbases = [path.join(dir, f) for f in ['magsweep1', 'magsweep2']]
    # magbases = [f for f in ['magsweep1', 'magsweep2']]
    # maglookup = {
    #     'Upper Leg IMU' : '66',
    #     'Orient 8' : '8',
    #     'Orient 43': '43'}
    # magSamples = 2000
    # refTime = 1.0
    # posStdDev = 0.0005
    # rotStdDev = 0.004


    # In[3]:


    # ref3D = SplinedMarkerCapture(
    #     loadQualisysTSVFile(refbase + "_3D.tsv"), positionStdDev=posStdDev)
    # ref6D = SplinedMarkerCapture(
    #     loadQualisysTSVFile(refbase + "_6D.tsv"), rotationStdDev=rotStdDev)
    # capture3D = SplinedMarkerCapture(
    #     loadQualisysTSVFile(filebase + "_3D.tsv"), positionStdDev=posStdDev)
    marker_3D = SplinedMarkerCapture(loadViconCSVFile(marker_filename), positionStdDev=posStdDev)
    print("Made it 1")
    # captureSD = SensorDataCapture.load(filebase + ".sdc")
    imu_3D = pd.read_csv(imu_filename, compression='gzip')
    hip, thigh, knee, shin, ankle =         ['Hip', 'Thigh', 'Knee Hinge', 'Shin', 'Ankle']
    jointNames = ['Upper Leg', 'Lower Leg', 'Foot']
    jointAbbrevs = ['femur', 'tibia', 'foot']
    orientIDs = ['66', '43', '8']
    jointMarkerNames = [hip, knee, ankle]
    refMarkerNames = [[thigh, knee], [shin, ankle], []]
    imuMarkerNames =         [[j + ' IMU - ' + str(i) for i in range(1,4)] for j in jointNames]
    print("imu marker names")
    print(imuMarkerNames)
    print("Made it 2")


    # In[4]:


    jointMarkerSets = lambda c: [
        list(map(c.marker, jointMarkerNames)),
        [list(map(c.marker, r)) for r in refMarkerNames],
        [list(map(c.marker, i)) for i in imuMarkerNames]]
    print("joint marker sets")
    print(jointMarkerSets)
    imuMarkerSets = lambda c: [
        [c.marker(i[0]) for i in imuMarkerNames],
        [list(map(c.marker,i[1:])) for i in imuMarkerNames]]
    print("imu marker sets")
    print(imuMarkerSets)
    jointRefTrajectories = [MultiMarkerTrajectory(j, r + i, refTime=refTime)
        for j, r, i in zip(*(jointMarkerSets(ref3D)))]
    print("joint ref trajectories")
    print(jointRefTrajectories)
    jointTrajectories = [
        MultiMarkerTrajectory(j, r + i, refVectors=m.refVectors) \
            for j, r, i, m in \
                zip(*(jointMarkerSets(capture3D) + [jointRefTrajectories]))]
    print("joint trajectories")
    print(jointTrajectories)
    print("Made it 3")


    # In[5]:


    imuRefTrajectories = [MultiMarkerTrajectory(p, r, refTime=refTime)
        for p, r in zip(*(imuMarkerSets(ref3D)))]
    print("imu ref trajectories")
    print(imuRefTrajectories)
    imuVecTrajectories = [MultiMarkerTrajectory(p, r, refVectors=m.refVectors)
        for p, r, m in zip(*(imuMarkerSets(capture3D) + [imuRefTrajectories]))]
    print("imu vec trajectories")
    print(imuVecTrajectories)
    imuRefMarkers = [ref6D.marker(j + ' IMU') for j in jointNames]
    print("imu ref markers")
    print(imuRefMarkers)
    imuOffsets = [i.position(refTime) - j.position(refTime)
        for i, j in zip(imuRefTrajectories, jointRefTrajectories)]
    print("imu offsets")
    print(imuOffsets)
    imuRotations = [t.rotation(refTime).conjugate * m.rotation(refTime)
        for t, m in zip(imuRefTrajectories, imuRefMarkers)]
    print("imu rotations")
    print(imuRotations)
    imuTrajectories = [OffsetTrajectory(v, o, r)
        for v, o, r in zip(imuVecTrajectories, imuOffsets, imuRotations)]
    print("imu trajectories")
    print(imuTrajectories)
    print("Made it 4")


    # In[6]:


    imuData = [captureSD.device(i) for i in orientIDs]
    print("imu data")
    print(imuData)
    joints = []
    for i in range(len(jointNames)):
        name = jointNames[i]
        print("name")
        print(name)
        traj = jointTrajectories[i]
        print("traj")
        print(traj)
        if i == 0:
            model = SampledBodyModel(name)
            print("model")
            print(model)
            model.positionKeyFrames = traj.posMarker.positionKeyFrames
            print("model.positionKeyFrames")
            print(model.positionKeyFrames)
            joint = model
            print("joint")
            print(joint)
        else:
            parent = joints[-1]
            print("parent")
            print(parent)
            refTraj = jointRefTrajectories[i]
            print("refTraj")
            print(refTraj)
            parentRefTraj = jointRefTrajectories[i - 1]
            print("parentRefTraj")
            print(parentRefTraj)
            pos = refTraj.position(refTime)
            print("pos")
            print(pos)
            parentPos = parentRefTraj.position(refTime)
            print("parentPos")
            print(parentPos)
            joint = SampledJoint(joints[-1],name, offset=(pos - parentPos))
            print("joint")
            print(joint)
        joint.rotationKeyFrames = traj.rotationKeyFrames
        print("joint.rotationKeyFrames")
        print(joint.rotationKeyFrames)
        joints.append(joint)
    print("Made it 5")


    # In[7]:


    model = SplinedBodyModel(model)
    print("model")
    print(model)
    joints = model.joints
    print("joints")
    print(joints)
    imuJointTrajectories = [OffsetTrajectory(j, o, r)
        for j, o, r in zip(joints, imuOffsets, imuRotations)]
    print("imuJointTrajectories")
    print(imuJointTrajectories)
    positionSets = []
    valueSets = []
    print("Made it 6")
    for magbase in magbases:
        orient = SensorDataCapture.load(magbase + '.sdc')
        print("orient")
        print(orient)
        optical = loadQualisysTSVFile(magbase + '_6D.tsv')
        print("optical")
        print(optical)
        for marker in optical.markers:
            device = orient.device(maglookup[marker.id])
            print("device")
            print(device)
            magData = device.sensorData('magnetometer').values
            print("magData")
            print(magData)
            positionSets.append(marker.positionKeyFrames.values)
            valueSets.append(
                    marker.rotationKeyFrames.values.rotateVector(magData))
    print("positionSets")
    print(positionSets)
    print("valueSets")
    print(valueSets)
    print("Made it 7")


    # In[8]:


    positions = np.hstack(positionSets)
    print("positions")
    print(positions)
    values = np.hstack(valueSets)
    print("values")
    print(values)
    valid = ~np.any(np.isnan(positions),axis=0) & ~np.any(np.isnan(values),axis=0)
    print("valid")
    print(valid)
    dev = values - np.median(values[:,valid],axis=1).reshape((3,1))
    print("dev")
    print(dev)
    step = np.shape(values[:,valid])[1] // magSamples
    print("step")
    print(step)
    posSamples = positions[:,valid][:,::step]
    print("posSamples")
    print(posSamples)
    valSamples = values[:,valid][:,::step]
    print("valSamples")
    print(valSamples)
    environment = Environment()
    print("environment")
    print(environment)
    environment.magneticField =         NaturalNeighbourInterpolatedField(posSamples, valSamples)
    print("environment.magneticField")
    print(environment.magneticField)
    print("Made it 8")


    # In[9]:


    sim = Simulation(environment=environment)
    print("sim")
    print(sim)
    sim.time = model.startTime
    print("sim.time")
    print(sim.time)
    distortIMUs = []
    dt = 1/capture3D.sampled.frameRate
    print("dt")
    print(dt)
    for traj in imuJointTrajectories:
        platform = IdealIMU(sim, traj)
        print("platform")
        print(platform)
        distortIMUs.append(BasicIMUBehaviour(platform, dt))
    print("distortIMUs")
    print(distortIMUs)
    print("Made it 9")


    # In[10]:


    sim.run(model.endTime)
    for imu in range(3):
        for sensorName in ['accelerometer', 'magnetometer', 'gyroscope']:
            sim = getattr(distortIMUs[imu].imu,sensorName).rawMeasurements
            print("sim")
            print(sim.values)
            print("Made it 10")
            true = imuData[imu].sensorData(sensorName)(sim.timestamps + model.startTime)
            print("true")
            print(true)
    # return passed
    print("Passed?")
    print(assert_vectors_correlated(sim.values, true, 0.8))
            # if (sensorName == 'accelerometer'):
            #     plt.figure(i)
            #     plt.plot(true)
            #     plt.title = i
            #     plt.xlabel("Time (s)")
            #     plt.ylabel("Acceleration (m/s^2)")
            #     plt.legend()
            #     plt.show()
            #     i += 1
            # yield assert_vectors_correlated, sim.values, true, 0.8
            # return assert_vectors_correlated(sim.values, True, 0.8)

testAgainstReality()
    # In[ ]:




