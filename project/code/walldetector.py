__author__ = 'manuelli'

import ransac as RS
import ddapp.objectmodel as om
import numpy as np


from ddapp.debugVis import DebugData
import ddapp.visualization as vis



class WallDetector(object):

    def __init__(self, sensor):
        self.sensor = sensor
        self.initializeLinearModel()
        self.initializeOptions()

    def initializeLinearModel(self):
        n_inputs = 1
        n_outputs = 1
        input_columns = range(n_inputs) # the first columns of the array
        output_columns = [n_inputs+i for i in range(n_outputs)]
        debug = False
        self.model = RS.LinearLeastSquaresModelWithIntercept(input_columns,output_columns,debug=debug)

    def initializeOptions(self):
        self.options = {}
        self.options['ransacThresholdDistance'] = 10.0
        self.options['alternateThresholdDistance'] = 5.0
        self.options['minPointsForGoodFit'] = 4
        self.options['maxNumIterations'] = 50


    def findWalls(self, raycastDistance, addNoise=False):

        lidarReturns, lidarIdx = self.sensor.transformRaycastToLocalCoordinates(raycastDistance, discardMaxRange=True,
                                                                      addNoise=addNoise)
        verbose = False
        if verbose:
            print "num lidar hits", np.size(lidarIdx)
            print "lidarIdx", lidarIdx
        # lidarReturns is of shape 2 x numRays, in order to use it for
        # ransac need to transform it
        allData = lidarReturns.transpose()


        # run RANSAC algorithm
        debug = False
        remainingDataIdx = np.arange(np.size(lidarIdx))

        wallsFound = []
        counter = 0
        while ((np.size(remainingDataIdx) > self.options['minPointsForGoodFit']) and counter < 2):


            if verbose:
                counter+=1
                print "on iteration ", counter
                print "num points remaining = ", np.size(remainingDataIdx)
                print "remainingLidarDataIdx", lidarIdx[remainingDataIdx]

            ransac_fit, ransac_data = RS.ransac(allData[remainingDataIdx,:], self.model,
                                                2, self.options['maxNumIterations'],
                                                self.options['ransacThresholdDistance'],
                                                self.options['minPointsForGoodFit'],
                                                debug=debug, return_all=True)

            remainingDataError = self.model.get_error(allData[remainingDataIdx,:], ransac_fit)

            inliersIdx = np.where(remainingDataError < self.options['alternateThresholdDistance'])[0]
            idxToRemove = remainingDataIdx[inliersIdx]
            lidarRayIdx = lidarIdx[idxToRemove]
            angle = np.mean(self.sensor.angleGrid[lidarRayIdx])
            wallData = {'ransacFit': ransac_fit, 'lidarRayIdx': lidarRayIdx, 'angle':angle}
            remainingDataIdx = np.setdiff1d(remainingDataIdx, idxToRemove)

            if verbose:
                print "num points in ransac data", np.size(ransac_data['inliers'])
                print "num points that fit model", np.size(lidarRayIdx)
                modelError = self.model.get_error(allData, ransac_fit)
                print "error on entire dataset is", modelError
                print "lidarRayIdx", np.sort(lidarRayIdx)
            wallsFound.append(wallData)

        if verbose:
            print "num points remaining", np.size(remainingDataIdx)
        return wallsFound

    def testFindWalls(self):
        raycastDistance = self.sensor.raycastAllFromCurrentFrameLocation()

        wallsFound = self.findWalls(raycastDistance, addNoise=True)

        carTransform = om.findObjectByName('robot frame').transform
        d = DebugData()

        for wallData in wallsFound:
            intercept = wallData['ransacFit'][0]
            slope = wallData['ransacFit'][1]
            wallDirectionInCarFrame = np.array([1.0, slope, 0.0])
            wallPointInCarFrame = np.array([0.0, intercept, 0.0])

            # now need to transform these to world frame in order to plot them.
            wallPointWorldFrame = np.array(carTransform.TransformPoint(wallPointInCarFrame))
            wallDirectionWorldFrame = np.array(carTransform.TransformVector(wallDirectionInCarFrame))
            wallDirectionWorldFrame = 1/np.linalg.norm(wallDirectionWorldFrame) * wallDirectionWorldFrame
            lineLength = 15.0
            lineOrigin = wallPointWorldFrame - lineLength*wallDirectionWorldFrame
            lineEnd = wallPointWorldFrame + lineLength*wallDirectionWorldFrame

            d.addLine(lineOrigin, lineEnd, radius=0.3, color=[0,0,1])

        vis.updatePolyData(d.getPolyData(), 'line estimate', colorByName='RGB255')
        return wallsFound


