import argparse
import glob
import json
import natsort
# import open3d as o3d

from src.pose import Pose

from src.trajectory import Trajectory

from src.utils import getImagesTimestamp

def parseProgramArgs():
    parser = argparse.ArgumentParser(description='Camera trajectory extraction.')
    parser.add_argument('trajectory', help='Lidar trajectory odometry')
    parser.add_argument('images', help='Folder with images')
    parser.add_argument('dst', help='Output file')
    return parser.parse_args()

def computeCameraTrajectory(folder,traj_lidar):
    #timestamps of images
    timestamps = getImagesTimestamp(folder)
    
    #Interpolate camera pose from lidar trajectory
    traj = Trajectory.interpolateFromTimestamp(traj_lidar,timestamps)

    return traj

def main():
    args = parseProgramArgs()
    
    traj_lidar = Trajectory.readTrajectory(args.trajectory)
    traj_cam = computeCameraTrajectory(args.images,traj_lidar)

    print("Writing Camera trajectory to ", args.dst)
    Trajectory.writeTrajectory(traj_cam,args.dst)

    print("Lidar trajectory length : ", len(traj_lidar))
    print("Lidar timestamps start/end :", traj_lidar[0].timestamp,traj_lidar[-1].timestamp)
    print("Camera trajectory length : ", len(traj_cam))
    print("Camera timestamps start/end :", traj_cam[0].timestamp,traj_cam[-1].timestamp)

if __name__== "__main__":

    main()
