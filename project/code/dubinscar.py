__author__ = 'manuelli'
import numpy as np
import scipy.integrate as integrate
from car import CarPlant


import ddapp.vtkAll as vtk



class DubinsCar(CarPlant):

    def __init__(self):

        x = 0
        y = 0
        v = 1.0
        theta = 0
        psi = 0
        self.idx = {'x':0,'y':1,'v':2, 'theta':3, 'psi': 4}
        self.state = np.array([x,y,v,theta,psi])
        self.L = 1.0
        self.psiMin = - np.deg2rad(30)
        self.psiMax = np.deg2rad(30)



    # you need t here since integrate.odeint expects it
    # however we don't actually use this t
    def dynamics(self, state, t, controlInput):

        dqdt = np.zeros_like(state)

        dqdt[self.idx['x']] = state[self.idx['v']]*np.cos(state[self.idx['theta']])
        dqdt[self.idx['y']] = state[self.idx['v']]*np.sin(state[self.idx['theta']])
        dqdt[self.idx['v']] = controlInput[0]
        dqdt[self.idx['theta']] = np.tan(state[self.idx['psi']])/self.L
        dqdt[self.idx['psi']] = controlInput[1]

        return dqdt

    def simulateOneStep(self, controlInput, startTime=0.0, dt=0.05):
        t = np.linspace(startTime, startTime+dt, 2)
        newState = integrate.odeint(self.dynamics, self.state, t, args=(controlInput,))
        self.state = newState[-1,:]
        self.state[self.idx['psi']] = np.clip(self.state[self.idx['psi']], self.psiMin, self.psiMax)
        return self.state

    def setFrameToState(self, state=None):
        if state is None:
            state = self.state

        x = state[self.idx['x']]
        y = state[self.idx['y']]
        theta = state[self.idx['theta']]
        t = vtk.vtkTransform()
        t.Translate(x,y,0.0)
        t.RotateZ(np.degrees(theta))
        self.frame.copyFrame(t)

    def resetStateToFrameLocation(self):
        (x,y,_) = self.frame.transform.GetPosition()
        (_,_,thetaDegrees) = self.frame.transform.GetOrientation()

        theta = np.deg2rad(thetaDegrees)
        v = 0
        psi = 0

        self.state[self.idx['x']] = x
        self.state[self.idx['y']] = y
        self.state[self.idx['v']] = v
        self.state[self.idx['theta']] = theta
        self.state[self.idx['psi']] = psi


