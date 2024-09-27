import os
import open3d as o3d
import glob
import natsort
import numpy as np
from .calib_lidar_camera import CalibLiDARCamera

class LidarOuster():
    def __init__(self,output_dir,hostname,port):
        
        #Configure lidar connexion
        self.lidar_hostname = hostname
        self.lidar_port = port
        self.use_sensor = True

        if(type(self.lidar_hostname)==type(None)):
            self.use_sensor = False

        if(self.use_sensor):
            from ouster import client
            #Retrieve Scan iterator
            self.metadata,self.scans_it = client.Scans.sample(self.lidar_hostname,1,self.lidar_port)
            self.xyzlut = client.XYZLut(self.metadata)

        #Current scan sample
        self.scan = None

        #Create output directory
        self.output_dir = output_dir
        if(not os.path.exists(output_dir)):
            os.makedirs(output_dir)


    #Get a scan from the scan iterator
    def sample(self):
        if self.use_sensor:
            self.scan = next(self.scans_it)[0] 
    

    def get_pcd(self):
        if self.use_sensor:
            xyz = self.xyzlut(self.scan.field(client.ChanField.RANGE))
            #Use Open3D to save to PLY ====> Should change for plyfile
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1,3))
            return pcd

    #Save current scan to ply
    def save(self,filename):
        if not self.use_sensor:
            return

        if(type(self.scan)==type(None)):
            print("There is no current scan")
            return

        if(type(filename)==int):
            filename = str(filename)


        if(filename==""):
            print("Please enter a filename")
            return

        if(not filename.endswith(".ply")):
            filename += ".ply"

        #Get coordinates 
        xyz = self.xyzlut(self.scan.field(client.ChanField.RANGE))

        #Use Open3D to save to PLY ====> Should change for plyfile
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1,3))
        scan_path = os.path.join(self.output_dir , filename)
        o3d.io.write_point_cloud(scan_path, pcd) 
    
    def get_next_file_nb(self):
            
        last_files = natsort.natsorted(glob.glob(os.path.join(self.output_dir,"*.ply")))
        if len(last_files)==0:
            return 1
        return int(os.path.basename(last_files[-1]).split(".")[0])+1

    def pick_points(self,pcd):
        print("")
        print(
            "1) Please pick one points one the plane using [shift + left click]"
        )
        print("   Press [shift + right click] to undo point picking")
        print("2) Afther picking points, press esc for close the window")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)
        view_ctl = vis.get_view_control()
        #TODO : Find a more generic way to set the view
        view_ctl.set_up((0, 0, 1))  # set the positive direction of the x-axis as the up direction
        #view_ctl.set_up((0, -1, 0))  # set the negative direction of the y-axis as the up direction
        #view_ctl.set_front((1, 0, 0))  # set the positive direction of the x-axis toward you
        view_ctl.set_front((-1, 0, 0))  # set the positive direction of the x-axis toward you
        view_ctl.set_lookat((0, 0, 0))  # set the original point as the center point of the window
        view_ctl.set_zoom(0.001)  # set the original point as the center point of the window
        vis.run()  # user picks points
        vis.destroy_window()
        print("")
        return vis.get_picked_points()

    def pick_planes(self):
        paths = natsort.natsorted(glob.glob(os.path.join(self.output_dir,"*.ply")))
        for path in paths:
            print("Working on file : ", path)
            name = os.path.basename(path).split(".")[0]

            #Retrieving file 
            pcd = o3d.io.read_point_cloud(path)
            points = np.asarray(pcd.points)

            #Filtrering point in front of the sensor
            # points_front = points[(points[:,0]>0) & (points[:,1]>-3) & (points[:,1]<3)]
            points_front = points
            pcd_front = o3d.geometry.PointCloud()
            pcd_front.points = o3d.utility.Vector3dVector(points_front)


            #Get point from user
            picked_points = self.pick_points(pcd_front)
            while len(picked_points)>1:
                print("Please pick 1 points")
                picked_points = self.pick_points(pcd_front)
            
            if(len(picked_points)==0):
                exit()

            picked_coord = np.asarray(pcd_front.select_by_index(picked_points).points)
            
            dist_to_plane = 1000000 #Distance picked point to estimated plane
            dist_to_point = 0.9 #Threshold for first filtering (m)
            while(dist_to_plane>0.05): #Threshold for plane fit acceptance
                dist_to_point-=0.1
                plane_points = []

                #Keeping only point in the dist_to_point threshold
                for point in points_front:
                    if np.linalg.norm(picked_coord[0]-point)<dist_to_point:
                        plane_points.append(point)


                pcd_plane = o3d.geometry.PointCloud()
                pcd_plane.points = o3d.utility.Vector3dVector(np.array(plane_points))
                #Applying Ransac for plane fitting 
                plane_model, inliers = pcd_plane.segment_plane(distance_threshold=0.01,
                                                        ransac_n=3,
                                                        num_iterations=1000)

                [a, b, c, d] = plane_model
                print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
                
                dist_to_plane = np.dot([a,b,c],picked_coord.T)[0] + d
                
            print(f"Dist to plane : ", dist_to_plane)

            #Showing result
            inlier_cloud = pcd_plane.select_by_index(inliers)
            inlier_cloud.paint_uniform_color([1.0, 0, 0])
            #outlier_cloud = pcd_plane.select_by_index(inliers, invert=True)
            #TODO : Find a more generic way to set the view
            o3d.visualization.draw_geometries([pcd,inlier_cloud],zoom=0.001,
                                        #front=[1,0,0],
                                        front=[-1,0,0],
                                        lookat=[0,0,0],
                                        up=[0,0,1])

            #Saving normal and point
            n = np.array([a,b,c])
            p = np.array(inlier_cloud.points)[0]
            to_save = np.array([n,p])
            np.savetxt(os.path.join(self.output_dir,name+".plane.txt"),to_save)
        print("All plane extracted !")
        
    def calib_lidar_camera(self,camera_calib):
        if not os.path.exists(camera_calib):
            print("You need to calibrate the camera first")
            return
        calib = CalibLiDARCamera()
        calib.readOpenCVIntrinsics(camera_calib)
        calib.readLidarPlanePoses(self.output_dir)
        calib.computeExtrinsicCalibration()
        calib.saveCalibExtrinsic()
