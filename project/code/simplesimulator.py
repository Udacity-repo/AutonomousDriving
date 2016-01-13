__author__ = 'manuelli'
import numpy as np


from dubinscar import DubinsCar
from sensor import SensorObj
from world import World
from controller import DubinsController
from controller import DubinsPIDController
from walldetector import WallDetector
from purepursuitcontroller import SimplePlanner
from purepursuitcontroller import PurePursuitController
from purepursuitcontroller import FeatureDetector



import ddapp.vtkAll as vtk
import ddapp.visualization as vis
import ddapp.objectmodel as om
from ddapp.debugVis import DebugData
from ddapp.consoleapp import ConsoleApp
from ddapp.timercallback import TimerCallback
from ddapp import applogic
from ddapp import screengrabberpanel
from ddapp import transformUtils


from PythonQt import QtCore, QtGui





# want it to launch a hallway world
# also want to be able to start/stop the simulation by clicking the
# play/pause button
# don't worry about recording playback or anything, at least for now


class SimpleSimulator(object):

    def __init__(self):
        self.setDefaultOptions()
        self.initialize()

    def setDefaultOptions(self):
        defaultOptions = dict()
        defaultOptions['Sensor'] = dict()
        defaultOptions['Sensor']['rayLength'] = 20
        defaultOptions['Sensor']['numRays'] = 100
        defaultOptions['Sensor']['FOV'] = 270

        defaultOptions['Sim'] = dict()
        defaultOptions['Sim']['dt'] = 0.05
        defaultOptions['Sim']['usePursuitController'] = True

        defaultOptions['featureDetector'] = {}
        defaultOptions['featureDetector']['detectCorners'] = True
        self.options = defaultOptions

        self.app = ConsoleApp()
        self.view = self.app.createView(useGrid=False)

    def initialize(self):
        self.car = DubinsCar()

        self.sensor = SensorObj(rayLength=self.options['Sensor']['rayLength'],
                                numRays=self.options['Sensor']['numRays'],
                                FOV=self.options['Sensor']['FOV'])


        self.controller = DubinsPIDController(self.sensor, self.car)
        self.world = World.buildDonutWorld()
        self.locator = World.buildCellLocator(self.world.visObj.polyData)
        self.sensor.setLocator(self.locator)
        self.robot, self.carFrame = World.buildRobot()
        self.car.setFrame(self.carFrame)
        self.locator = World.buildCellLocator(self.world.visObj.polyData)
        self.wallDetector = WallDetector(self.sensor)
        self.planner = SimplePlanner(self.sensor)
        self.pursuitController = PurePursuitController(self.car, self.sensor)
        self.featureDetector = FeatureDetector(self.sensor)

    def updateDrawIntersection(self, frame):

        origin = np.array(frame.transform.GetPosition())
        #print "origin is now at", origin
        d = DebugData()

        colorMaxRange = [0,1,0]

        for i in xrange(self.sensor.numRays):
            ray = self.sensor.rays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            #print "rayTransformed is", rayTransformed
            intersection = self.sensor.raycast(self.locator, origin, origin + rayTransformed*self.sensor.rayLength)

            if intersection is not None:
                d.addLine(origin, intersection, color=[1,0,0])
            else:
                d.addLine(origin, origin+rayTransformed*self.sensor.rayLength, color=colorMaxRange)

        vis.updatePolyData(d.getPolyData(), 'rays', colorByName='RGB255')


    def playTimerCallback(self):
        self.simulateOneStep(self.options['Sim']['dt'])

    def simulateOneStep(self, dt):


        verbose = True
        raycastData = self.sensor.raycastAllFromCurrentFrameLocation()

        if self.options['Sim']['usePursuitController']:
            L_fw, theta = self.planner.computePursuitPoint(raycastData, plot=True)
            carState = self.car.state
            controlInput = self.pursuitController.computeControlInput(carState, dt, L_fw, theta)

        if self.options['featureDetector']['detectCorners']:
            self.featureDetector.detectCorner(raycastData, plot=True, verbose=True)

        newState = self.car.simulateOneStep(controlInput, dt=dt)
        self.car.setFrameToState()

    def onPlayButton(self):

        if self.playTimer.isActive():
            self.playTimer.stop()
            return

        self.car.resetStateToFrameLocation()
        self.playTimer.start()

    def launchViewer(self):

        playButtonFps = 1.0/self.options['Sim']['dt']

        print "playButtonFPS", playButtonFps
        self.playTimer = TimerCallback(targetFps=playButtonFps)
        self.playTimer.callback = self.playTimerCallback

        panel = QtGui.QWidget()
        l = QtGui.QHBoxLayout(panel)

        playButton = QtGui.QPushButton('Play/Pause')
        playButton.connect('clicked()', self.onPlayButton)

        l.addWidget(playButton)

        w = QtGui.QWidget()
        l = QtGui.QVBoxLayout(w)
        l.addWidget(self.view)
        l.addWidget(panel)
        w.showMaximized()


        self.carFrame.connectFrameModified(self.updateDrawIntersection)
        self.updateDrawIntersection(self.carFrame)

        applogic.resetCamera(viewDirection=[0.2,0,-1])
        self.view.showMaximized()
        self.view.raise_()

        self.app.start()

    def testCrossTrackError(self):
        carState = np.copy(self.car.state)
        carState[self.car.idx['v']] = 5.0
        raycastData = self.sensor.raycastAllFromCurrentFrameLocation()
        wallsFound = self.wallDetector.findWalls(raycastData, addNoise=False)

        return self.controller.computeCrossTrackError(carState, wallsFound, verbose=True)

    def testPursuitController(self):
        L_fw, theta = self.planner.testFromCurrent()
        self.car.resetStateToFrameLocation()
        v_des = 5.0
        carState = self.car.state
        dt = 0.05
        u = self.pursuitController.computeControlInput(carState, dt, L_fw, theta, v_des, verbose=True)

    def testDetectFeature(self):
        raycastData = self.sensor.raycastAllFromCurrentFrameLocation()
        self.featureDetector.detectCorner(raycastData, plot=True, verbose=True)