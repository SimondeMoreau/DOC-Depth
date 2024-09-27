import numpy as np
import quaternion

class Transform:
    def __init__(self,T):
        self.T = T


    def translation(self):
        """
        Returns translation vector block from SE3 (R|t) 4x4 matrix.
        """
        return self.T[0:3, 3]

    def rotation(self):
        """
        Returns rotation matrix block vector from SE3 (R|t) 4x4 matrix.
        """
        return self.T[0:3, 0:3]

        
    def quaternion(self):
        """
        Returns rotation matrix block vector from SE3 (R|t) 4x4 matrix.
        """
        return quaternion.from_rotation_matrix(self.rotation())

    @staticmethod
    def fromTranslationQuaternion(u,q):
        R = quaternion.as_rotation_matrix(q)
        #Ridig Transform
        T = np.eye(4)
        T[0:3,0:3] = R
        T[0:3, 3] = u
        return Transform(T)

    @staticmethod
    def fromTranslationRotation(u,R):
        #Ridig Transform
        T = np.eye(4)
        T[0:3,0:3] = R
        T[0:3, 3] = u
        return Transform(T)

    def matrix(self):
        """
        Returns the SE3 matrix.
        """
        return self.T