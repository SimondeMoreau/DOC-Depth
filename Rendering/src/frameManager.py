import numpy as np
import os
import quaternion

from .trajectory import Trajectory
from .utils import *

class FrameManager:

    def __init__(self,traj_cam_f,traj_lidar_f,frame_folder,pts_per_frame,cam_step,dist_merge,max_depth,dist_past,classif_property_name,crop_norm):
        self.traj_cam = Trajectory.readTrajectory(traj_cam_f)
        self.traj_lidar = Trajectory.readTrajectory(traj_lidar_f)
        self.frame_folder = frame_folder
        self.pts_per_frame = pts_per_frame
        self.cam_step = cam_step
        self.dist_merge = dist_merge
        self.max_depth = max_depth
        self.dist_past = dist_past
        self.classif_property_name = classif_property_name
        self.crop_norm = crop_norm

        # Verify timestamps of the trajectories are chronological
        self.traj_cam = self.clean_traj_timestamp(self.traj_cam)
        self.traj_lidar = self.clean_traj_timestamp(self.traj_lidar)

        # Subsample trajectories
        self.traj_cam = Trajectory.subSample(self.traj_cam,cam_step)
        self.traj_lidar = Trajectory.subSample(self.traj_lidar, dist_merge)

        # Match trajectories
        self.traj_match = Trajectory.matchTraj(self.traj_cam,self.traj_lidar)


        self.lidar_frames = {}
        self.dynamic_frame = None


    def loadFrames(self,i):
        # Find lidar frames for the current camera pose
        matched_idx = findPoseIdx(self.traj_match[i],self.traj_lidar)
        traj_depth,_ = Trajectory.cutDistance(self.traj_lidar,self.max_depth,matched_idx,self.dist_past) # Cut the trajectory to keep only the points in the depth
        #traj_depth = Trajectory.subSample(traj_depth,args.dist_merge)


        # Load frames
        needed_frames = []
        for pose in traj_depth:
            needed_frames.append(pose.index)
            if pose.index not in self.lidar_frames.keys():
                self.lidar_frames[pose.index] = readPly(os.path.join(self.frame_folder,getPlyName(pose.index)))
            
                # Read the point cloud
                ply_basename = getPlyName(pose.index)
                ply_filename = os.path.join(self.frame_folder,ply_basename)

                trame,meta,dt_meta = readPly(ply_filename)

                # Get static points
                # GENERAL
                # trame = self.getStatic(trame,meta,dt_meta)
                
                #KITTI
                if abs(pose.index - self.traj_lidar[-1].index) < 5: # Last frames we want to keep the full point cloud to keep maximum density
                    trame = self.getStatic(trame,meta,dt_meta,no_mask=True)
                else:
                    trame = self.getStatic(trame,meta,dt_meta)

                # Get the point cloud in the right size
                trame = self.getSizedPointcloud(trame)

                # Save the point cloud
                self.lidar_frames[pose.index] = {
                    "trame": trame,
                    "pose": pose
                }



        # Delete frames not needed anymore
        to_delete = []
        for frame in self.lidar_frames.keys():
            if frame not in needed_frames:
                to_delete.append(frame)
        for frame in to_delete:
            del self.lidar_frames[frame]


        # Load dynamic frame
        ply_basename = getPlyName(self.traj_lidar[matched_idx].index)
        ply_filename = os.path.join(self.frame_folder,ply_basename)

        trame,others,dt_others = readPly(ply_filename)

        # Get dynamic points
        trame = self.getDynamic(trame,others,dt_others)

        # Get the point cloud in the right size
        trame = self.getSizedPointcloud(trame)

        self.dynamic_frame = {
            "trame": trame,
            "pose": self.traj_lidar[matched_idx]
        }


    def getCameraPose(self,i):
        return self.traj_cam[i]

    def getStaticFrames(self):
        return self.lidar_frames.values()

    def getDynamicFrame(self):
        return self.dynamic_frame

    def getStatic(self,trame,others,dt_others,no_mask=False):
        # Retrieve the classification
        idx_classid = getPropertieIndex(dt_others,self.classif_property_name)
        mask_static = others[:,idx_classid]<100                  
        mask_floor = others[:,idx_classid]<50

        trame = trame[mask_static]
        mask_floor = mask_floor[mask_static]
        
        # TODO: Find a better way to manage the crop
        # GENERAL
        # crop_mask = (np.abs(trame[:,0])<30)  | (mask_floor & (np.abs(trame[:,0])<30))
        # trame = trame[crop_mask]

        #FOR KITTI
        if no_mask:
            crop_mask = np.ones(trame.shape[0],dtype=bool)
        else:
            crop_mask = (~mask_floor) | (mask_floor & (np.abs(trame[:,0])<20))  
        trame = trame[crop_mask]

        norm = np.linalg.norm(trame,axis=1)
        mask = norm<self.crop_norm                
        trame = trame[mask]

        return trame

    def getDynamic(self,trame,others,dt_others):
        idx_classid = getPropertieIndex(dt_others,self.classif_property_name)
        mask_moving = others[:,idx_classid]>=100

        pts_moving = trame[mask_moving]

        norm = np.linalg.norm(pts_moving,axis=1)
        mask = ~((norm<2.5) & (pts_moving[:,2]>=-1))
        pts_moving = pts_moving[mask]

        return pts_moving
    
    def getCurrentTrame(self,i):
        # Load current frame
        matched_idx = findPoseIdx(self.traj_match[i],self.traj_lidar)
        ply_basename = getPlyName(self.traj_lidar[matched_idx].index)
        ply_filename = os.path.join(self.frame_folder,ply_basename)

        trame,others,dt_others = readPly(ply_filename)

        # Get the point cloud in the right size
        trame = self.getSizedPointcloud(trame)

        return {
            "trame": trame,
            "pose": self.traj_lidar[matched_idx]
        }


    def getSizedPointcloud(self,trame):
        pts_full = np.zeros((self.pts_per_frame,3))
        pts_full[:trame.shape[0]] = trame
        return pts_full


    def clean_traj_timestamp(self,traj):
        deleted = []
        # Verify timestamps of the trajectory are chronological
        j=1
        while j < len(traj):
            if(traj[j].timestamp<traj[j-1].timestamp):
                deleted.append(j)
                del traj[j]
                j-=1
            j+=1
        if len(deleted)>0:
            print("Error in trajectory, timestamps are not chronological")
            print("Deleting frames", deleted)
        return traj


    def len(self):
        return len(self.traj_cam)

