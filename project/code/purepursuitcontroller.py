__author__ = 'manuelli'
import numpy as np
import scipy.integrate as integrate
import ddapp.objectmodel as om

from ddapp.debugVis import DebugData
import ddapp.visualization as vis

from controller import ControllerObj
import utils


class PurePursuitController(object):

    def __init__(self, car, sensor):
        self.car = car
        self.sensor = sensor
        self.l_fw = 0.0
        self.initializeGains()
        self.initializeIntegrators()

    def initializeGains(self):
        self.gains = {}
        self.gains['v'] = {'Kp': 1.0, 'Kd': 1.0, 'Ki': 0.0}
        self.gains['psi'] = {'Kp': 10.0}

    def initializeIntegrators(self):
        self.integrators = {}
        self.integrators['v'] = 0.0

    def computeControlInput(self, carState, dt, L_fw, eta, v_des=10.0, verbose=False):
        u = np.zeros(2)
        u[0] = self.computeThrottleControlInput(carState, dt, v_des = v_des)
        u[1] = self.computeSteeringControlInput(carState, L_fw, eta, verbose=verbose)
        return u

    def computeSteeringControlInput(self, carState, L_fw, eta, verbose=False):
        psi = carState[self.car.idx['psi']]
        psiDes = np.arctan(self.car.L*np.sin(eta)/(L_fw/2.0 + self.l_fw*np.cos(eta)))
        psiDot = self.gains['psi']['Kp']*(psiDes - psi)

        if verbose:
            print "psiDes ", psiDes
            print "psiDot ", psiDot
        return psiDot

    def computeThrottleControlInput(self, carState, dt, v_des=5.0):
        v_des = v_des
        v = carState[self.car.idx['v']]
        u_throttle = -self.gains['v']['Kp']*(v - v_des) - self.gains['v']['Ki']*self.integrators['v']
        self.integrators['v'] += (v-v_des)*dt
        return u_throttle



class SimplePlanner(object):

    def __init__(self, sensor, L_fw=0.8):
        self.sensor = sensor
        self.L_fw = L_fw
        self.C = 20
        self.collisionThreshold = 1.3
        self.initializeOptions()
        self.initializeAngleGrid()

    def initializeOptions(self):
        self.options = {}
        self.options['angleSweep'] = np.deg2rad(40)
        self.options['angleResolution'] = np.deg2rad(10)

    def initializeAngleGrid(self):
        sweepGrid = np.arange(-np.pi/2.0, np.pi/2.0, self.options['angleResolution'])
        self.angleData = {}
        for theta in sweepGrid:
            thetaMin = theta - self.options['angleSweep']/2.0
            thetaMax = theta + self.options['angleSweep']/2.0
            idx = np.where(np.logical_and(self.sensor.angleGrid >= thetaMin, self.sensor.angleGrid <= thetaMax))[0]
            self.angleData[theta] = idx

    def computePursuitPoint(self, raycastData, plot=False):

        valBest = None
        thetaBest = None
        inverseTruncate = utils.inverseTruncate(raycastData, self.C, self.sensor.rayLength, self.collisionThreshold)
        for theta, idx in self.angleData.iteritems():
            val = np.mean(inverseTruncate[idx])

            if (val < valBest) or valBest is None:
                valBest = val
                thetaBest = theta


        if plot:
            d = DebugData()
            L_fw_vis = 10
            ptCarFrame = L_fw_vis*np.array([np.cos(thetaBest), np.sin(thetaBest),0])
            carFrame = om.findObjectByName('robot frame').transform
            ptWorldFrame = carFrame.TransformPoint(ptCarFrame)
            d.addSphere(ptWorldFrame, radius=1.0, color=[0,0,1])

            vis.updatePolyData(d.getPolyData(), 'pursuit point', colorByName='RGB255')
        return self.L_fw, thetaBest

    def testFromCurrent(self):
        raycastData = self.sensor.raycastAllFromCurrentFrameLocation()
        L_fw, theta = self.computePursuitPoint(raycastData, plot=True)
        print "theta = ", theta
        return L_fw, theta


class FeatureDetector(object):

    def __init__(self, sensor):
        self.sensor = sensor
        self.initializeOptions()

    def initializeOptions(self):
        self.options = {}
        self.options['corner'] = {}
        self.options['corner']['distanceThreshold'] = 8.0
        self.options['corner']['slidingWindowAngle'] = np.deg2rad(10)

    def detectCorner(self, raycastDistance, plot=False, startFromRight=True, verbose=False):
        windowSize = int(np.floor(self.options['corner']['slidingWindowAngle']/self.sensor.angleSpacing))
        cornerDict = {}
        for idx in xrange(windowSize, self.sensor.numRays):
            windowIdx = range(idx - windowSize, idx)
            raycastWindow = raycastDistance[windowIdx]

            windowMin = np.min(raycastWindow)
            windowMax = np.max(raycastWindow)
            threshold = self.options['corner']['distanceThreshold']
            if (windowMax - windowMin > threshold):

                longRangeIdx = np.where(raycastWindow > windowMax - threshold/3.0)[0]
                shortRangeIdx = np.where(raycastWindow < windowMin + threshold/3.0)[0]

                if np.mean(shortRangeIdx) < np.mean(longRangeIdx):
                    cornerIdx = windowIdx[np.max(shortRangeIdx)]
                else:
                    cornerIdx = windowIdx[np.min(shortRangeIdx)]

                if verbose:
                    print "detected a corner"
                    print "angle", self.sensor.angleGrid[cornerIdx]
                    print "distance", raycastDistance[cornerIdx]
                    print "windowIdx", windowIdx
                    print "raycastWindow", raycastWindow
                    print ""

                cornerDict[cornerIdx] = {'angle': self.sensor.angleGrid[cornerIdx], 'distance': raycastDistance[cornerIdx]}
                break


        if plot and len(cornerDict) > 0:
            d = DebugData()
            carFrame = om.findObjectByName('robot frame').transform
            for cornerIdx, cornerData in cornerDict.iteritems():
                theta = cornerData['angle']
                ptCarFrame = cornerData['distance']*np.array([np.cos(theta), np.sin(theta),0])
                ptWorldFrame = carFrame.TransformPoint(ptCarFrame)
                d.addSphere(ptWorldFrame, radius=1.0, color=[1,0,1])

            vis.updatePolyData(d.getPolyData(), 'corner detections', colorByName='RGB255')

        return cornerDict

