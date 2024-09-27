import os
import numpy as np
from plyfile import PlyData,PlyElement

def read_shader(file):
    with open(file) as f:
        return f.read()
    

def getPlyName(indice):
    return "frame_" + "%06d"%indice + ".ply"

def readPly(filename):
    ply = PlyData.read(filename)
    
    dt = ply['vertex'].dtype().descr
    dt_others = [d for d in dt if d[0] not in ["x","y","z"]]

    xyz = [ply["vertex"][p] for p in ["x","y","z"]]
    xyz = np.array(xyz).T

    others = [ply["vertex"][d[0]] for d in dt_others]
    others = np.array(others).T
    return xyz,others,dt_others

def OpenCVMtxToOpenGLMTx(mtx,width,height,zfar,znear):
    fx = mtx[0,0]
    fy = mtx[1,1]
    cx = mtx[0,2]
    cy = mtx[1,2]

    new_mtx = np.zeros((4,4),np.float32)

    new_mtx[0,0] = 2.0 * fx / width
    new_mtx[1,0] = 0.0
    new_mtx[2,0] = 0.0
    new_mtx[3,0] = 0.0

    new_mtx[0,1] = 0.0
    new_mtx[1,1] = -2.0 * fy / height
    new_mtx[2,1] = 0.0
    new_mtx[3,1] = 0.0

    new_mtx[0,2] = 2.0 * cx / width - 1.0
    new_mtx[1,2] = 1.0 - 2.0 * cy / height 
    new_mtx[2,2] = -(zfar + znear) / (znear - zfar)
    new_mtx[3,2] = 1.0


    new_mtx[0,3] = 0.0
    new_mtx[1,3] = 0.0
    new_mtx[2,3] = 2.0 * zfar * znear / (znear - zfar)
    new_mtx[3,3] = 0.0

    return new_mtx


def findPoseIdx(pose,traj):
    for j,p in enumerate(traj):
            if p.index == pose.index:
                return j
    return 0

def getPropertieIndex(dt_others,propertie):
    for i,d in enumerate(dt_others):
        if d[0] == propertie:
            return i
    return None

def getImagesTimestamp(folder):
    return np.loadtxt(os.path.join(folder,"timestamps.txt"))
