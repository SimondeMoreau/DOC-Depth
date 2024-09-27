import numpy as np
from plyfile import PlyData,PlyElement
import quaternion
from .pose import Pose

class Trajectory:

    @staticmethod
    def subSample(trajectory,min_dist):

        output = [trajectory[0]]
        
        for i in range(1,len(trajectory)):
            if(np.linalg.norm(trajectory[i].transform.translation()-output[-1].transform.translation())>=min_dist):
                output.append(trajectory[i])

        return output
    
    @staticmethod
    def matchTraj(ref_traj,tomatch_traj):#,method="dist"):
        output = []
        i=0
        N = len(tomatch_traj)
        for p in ref_traj:
            u_ref = p.transform.translation()
            # if(method=="dist"):
            #     while i<N-2 and np.linalg.norm(u_ref-tomatch_traj[i].T.translation()) > np.linalg.norm(u_ref-tomatch_traj[i+1].T.translation()):
            #         i+=1
            # else:
            while i<N-2 and abs(p.timestamp-tomatch_traj[i].timestamp) > abs(p.timestamp-tomatch_traj[i+1].timestamp):
                i+=1
            output.append(tomatch_traj[i])

        return output
    
    @staticmethod
    def readTrajectory(filename):
        traj = []
        ply = PlyData.read(filename)
        ply_props = [p.name for p in ply['vertex'].properties]
        i=0
        for vertex in ply['vertex']:
            #Retrive position
            u = np.array([vertex['x'], vertex['y'], vertex['z']])
            q = quaternion.from_float_array([vertex['q_w'],vertex['q_x'],vertex['q_y'],vertex['q_z']])
            timestamp = vertex['timestamp'] 
            index = vertex['indices'] if 'indices' in ply_props else None
            
            if timestamp == 0:
                timestamp = i

            #Compute pose
            P = Pose.fromTranslationQuaternion(u,q,timestamp,index)

            #Store
            traj.append(P)
            i+=1

        return traj
    
    @staticmethod
    def writeTrajectory(traj,output_file):
        #formating values
        values = []
        for p in traj:
            u = p.transform.translation()
            q = p.transform.quaternion()
            values.append((u[0],u[1],u[2],q.w,q.x,q.y,q.z,p.timestamp,p.index))
            #print(p.timestamp)

        #writing vertex
        vertex = np.array(values,dtype=[('x','f8'),('y','f8'),('z','f8'),('q_w','f8'),('q_x','f8'),('q_y','f8'),('q_z','f8'),('timestamp','f8'),('indices','i4')])
        el = PlyElement.describe(vertex,"vertex")
        
        #saving to file
        if not output_file.endswith(".ply"):
            output_file += ".ply"
        PlyData([el]).write(output_file)

    @staticmethod
    def interpolateFromTimestamp(ref_trajectory,timestamps):
        #lidar poses
        i=1
        pl1 = ref_trajectory[i-1]
        pl2 = ref_trajectory[i]

        N = len(ref_trajectory)
        traj = []

        for j,t in enumerate(timestamps):        
            #Finding matching timestamp between lidar and camera
            while(pl2.timestamp < t) and i<N-1:
                i+=1
                pl1 = ref_trajectory[i-1]
                pl2 = ref_trajectory[i]

            #print("Interpolation : ", i, pl1.timestamp, t, pl2.timestamp)
            #Interpolating
            p = Pose.interpolate(pl1,pl2,t)
            p.index=j
            traj.append(p)

        return traj
    
    @staticmethod
    def cutDistance(trajectory,max_distance,starting_index=0,before_distance=0):
        i = starting_index
        traj = [trajectory[i]]
        dist = 0
        while(i>0 and dist<before_distance):
            dist += np.linalg.norm(trajectory[i].transform.translation()-trajectory[i-1].transform.translation())
            i-=1
            traj.insert(0,trajectory[i])

        idx = len(traj)-1
        dist = 0
        i = starting_index
        N = len(trajectory)
        while i<N-1 and dist<max_distance:
            dist += np.linalg.norm(trajectory[i].transform.translation()-trajectory[i+1].transform.translation())
            i+=1
            traj.append(trajectory[i])
        
        if before_distance>0:
            return traj,idx
        return traj