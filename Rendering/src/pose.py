import numpy as np
import quaternion

from .transform import Transform

class Pose:
    def __init__(self,T,timestamp=None,index=None):
        self.transform = T
        self.timestamp = timestamp
        self.index = index
    
    @staticmethod
    def fromTranslationQuaternion(u,q,timestamp=None,index=None):
        T = Transform.fromTranslationQuaternion(u,q)
        return Pose(T,timestamp,index)


    def interpolate(p1,p2,t):
        #Get position
        u1 = p1.transform.translation()
        u2 = p2.transform.translation()
        q1 = p1.transform.quaternion()
        q2 = p2.transform.quaternion()
        t1 = p1.timestamp
        t2 = p2.timestamp

        #Interpolate XYZ
        x = np.interp(t,[t1,t2],[u1[0],u2[0]])
        y = np.interp(t,[t1,t2],[u1[1],u2[1]])
        z = np.interp(t,[t1,t2],[u1[2],u2[2]])
        u = [x,y,z]

        #Interpolate Quaternion
        q = quaternion.slerp(q1,q2,t1,t2,t)

        return Pose.fromTranslationQuaternion(u,q,t)