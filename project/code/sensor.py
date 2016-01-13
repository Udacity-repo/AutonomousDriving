import ddapp.vtkAll as vtk

import numpy as np
import ddapp.objectmodel as om


class SensorObj(object):

    def __init__(self, FOV=180.0, numRays=20, rayLength=8, noiseFraction=0.001):
        self.numRays = numRays
        self.rayLength = rayLength
        self.noiseFraction = noiseFraction

        FOVrad = FOV * np.pi/180.0
        self.angleMin = -FOVrad/2
        self.angleMax = FOVrad/2
        self.FOVDegrees = FOV

        self.angleGrid = np.linspace(self.angleMin, self.angleMax, self.numRays)
        self.angleSpacing = (self.angleMax - self.angleMin)/self.numRays

        self.rays = np.zeros((3,self.numRays))
        self.rays[0,:] = np.cos(self.angleGrid)
        self.rays[1,:] = np.sin(self.angleGrid)

    def setLocator(self, locator):
        self.locator = locator

    def raycastAll(self,frame):

        distances = np.zeros(self.numRays)

        origin = np.array(frame.transform.GetPosition())

        for i in range(0,self.numRays):
            ray = self.rays[:,i]
            rayTransformed = np.array(frame.transform.TransformNormal(ray))
            intersection = self.raycast(self.locator, origin, origin + rayTransformed*self.rayLength)
            if intersection is None:
                distances[i] = self.rayLength
            else:
                distances[i] = np.linalg.norm(intersection - origin)

        return distances

    def raycastAllFromCurrentFrameLocation(self):
        frame = om.findObjectByName('robot frame')
        return self.raycastAll(frame)

    def raycast(self, locator, rayOrigin, rayEnd):

        tolerance = 0.0 # intersection tolerance
        pt = [0.0, 0.0, 0.0] # data coordinate where intersection occurs
        lineT = vtk.mutable(0.0) # parametric distance along line segment where intersection occurs
        pcoords = [0.0, 0.0, 0.0] # parametric location within cell (triangle) where intersection occurs
        subId = vtk.mutable(0) # sub id of cell intersection

        result = locator.IntersectWithLine(rayOrigin, rayEnd, tolerance, lineT, pt, pcoords, subId)

        return pt if result else None

    def transformRaycastToLocalCoordinates(self, raycastDistance, discardMaxRange=True, addNoise=False):
        rays2D = self.rays[0:2,:] # this is 2 x numRays


        if discardMaxRange:
            tol = 1e-3
            idx = np.where(raycastDistance < (self.rayLength - tol))[0]

        else:
            idx = np.arange(self.numRays)

        if addNoise:
            raycastDistance = raycastDistance + self.noiseFraction*self.rayLength*(
                np.random.random(self.numRays) - 0.5)

        raycastLocal = np.multiply(self.rays[0:2,:], raycastDistance)
        raycastLocal = raycastLocal[:,idx]

        return raycastLocal, idx

    def transformRaycastTest(self):
        raycastDistance = self.raycastAllFromCurrentFrameLocation()
        return self.transformRaycastToLocalCoordinates(raycastDistance)

