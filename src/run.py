import ctypes
import numpy as np
from numpy.ctypeslib import ndpointer
import socket
import time
import json
import math
HOST='192.168.1.123'
PORT= 23456
m = ctypes.CDLL('./libapriltags_demo.dylib')
s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   
s.connect((HOST,PORT))

#TODO:
toward2r = {}
toward2r['S'] = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
toward2r['E'] = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
toward2r['N'] = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
toward2r['W'] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

# %debug
#int init(double fx, double fy, double px, double py, double tag_size);

f = open('./tag_info.json', 'r')
json_str = f.read()
f.close()
world_coordinats = json.loads(json_str)
fx = world_coordinats['fx']
fy = world_coordinats['fy']
px = world_coordinats['px']
py = world_coordinats['py']
cam_id = world_coordinats['cam_id']
tag_size = world_coordinats['tag_size']
m.init(ctypes.c_int32(cam_id), ctypes.c_double(fx), ctypes.c_double(fy), ctypes.c_double(px), ctypes.c_double(py), ctypes.c_double(tag_size))
info_mat = np.zeros(12, dtype=np.double)
info_mat = np.ascontiguousarray(info_mat)
mask = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
while True:
    tag_id = m.run(ctypes.c_void_p(info_mat.ctypes.data))
    if tag_id:
        rot = info_mat[:9].reshape((3,3))
        rot = mask.dot(rot)
        trans = info_mat[9:].reshape((3,1))
        world_coordinate = np.array(world_coordinats[str(tag_id)]['locate'])
        toward_r = toward2r[world_coordinats[str(tag_id)]['toward']]
        relative_positon = (- rot.T.dot(trans).T).reshape((-1,))
        real_positon = toward_r.dot(relative_positon)  + world_coordinate.reshape(-1)
        euler_angle = rotationMatrixToEulerAngles(toward_r.dot(rot.T))
        heading = (euler_angle[2] + 1.5 * np.pi) % (2 * np.pi)
        if heading < 0:
            heading += 2 * np.pi
#         euler_angle = rotationMatrixToEulerAngles(rot)
        x_rob = real_positon[1]
        y_rob = 2.835 - real_positon[0]
        print('id: %2d\n\t'%(tag_id), x_rob, y_rob)
        print('\t', math.degrees(heading))
        pos = bytes('%.4f %.4f %.4f'%(np.abs(x_rob), np.abs(y_rob), heading), 'ascii')
        time.sleep(0.1)
        s.sendall(pos)
#         time.sleep(0.5)


