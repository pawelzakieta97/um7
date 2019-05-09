import numpy as np
import math
from numpy.linalg import inv, norm


"""generating matrix for cross product operation"""
def generateCPMatrix(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def rotateX(angle):
    return np.array([[1, 0, 0], [0, math.cos(angle), -math.sin(angle)], [0, math.sin(angle), math.cos(angle)]])

def rotateY(angle):
    return np.array([[math.cos(angle), 0, math.sin(angle)], [0, 1, 0], [-math.sin(angle), 0, math.cos(angle)]])

def rotateZ(angle):
    return np.array([[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])

"""rotates vector w around vector e"""
def rotateVector(w, e, angle):
    e = e / norm(e)
    r = w - np.dot(w, e) * e
    if norm(r) == 0:
        return w
    #print(r)
    virtual_x = e
    virtual_y = r / norm(r)
    virtual_z = np.cross(virtual_x, virtual_y)
    A = np.transpose(np.array([virtual_x, virtual_y, virtual_z]))
    #print(A)
    r1 = np.dot(rotateX(angle), np.array([0, norm(r), 0]))
    r0 = np.dot(A, r1)

    return r0 + np.dot(w, e) * e


"""converts axis-angle rotation into world coordinate-based rotation matrix"""
def AA2Matrix(e, angle):
    xColumn = rotateVector(np.array([1, 0, 0]), e, angle)
    yColumn = rotateVector(np.array([0, 1, 0]), e, angle)
    zColumn = rotateVector(np.array([0, 0, 1]), e, angle)
    return np.transpose(np.array([xColumn, yColumn, zColumn]))



class filter:
    def __init__(self):
        self.orientation = np.eye(3)
        self.k = 0.01
        self.dt = 0.005

    def update(self, gyro_measurement, acc_measurement):
        gyro_measurement = gyro_measurement * self.dt
        gyro_estiamte = np.dot(self.orientation, rotateX(gyro_measurement[0]))
        gyro_estiamte = np.dot(gyro_estiamte, rotateY(gyro_measurement[1]))
        gyro_estiamte = np.dot(gyro_estiamte, rotateZ(gyro_measurement[2]))

        #the direction of vertical vector in the sensor-based coordinate system (z axis is facing is facing down)
        gyro_grav = gyro_estiamte[2, :]
        acc_grav = -acc_measurement/norm(acc_measurement)
        axis_of_rotation = np.cross(acc_grav, gyro_grav)
        angle = math.asin(norm(axis_of_rotation))
        #print(gyro_grav, acc_grav, axis_of_rotation, angle)
        if np.dot(gyro_grav, acc_grav) < 0:
            angle += math.pi/2

        #print(AA2Matrix(axis_of_rotation, angle))
        updated_estimate = np.dot(gyro_estiamte, AA2Matrix(axis_of_rotation, angle*self.k))
        self.orientation = updated_estimate


f = filter()
f.update(np.array([0, 0, 0]), np.array([1, 0, -1]))