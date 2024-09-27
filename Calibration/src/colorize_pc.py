import argparse
import glob
import json
import natsort
import open3d as o3d

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

class CameraModel:
    def __init__(self):
        self.opencv_calib_ = None
        self.mtx_ = None
        self.dist_ = None

        self.new_mtx_ = None
        self.roi_ = None

    @staticmethod
    def fromOpenCVIntrinsics(src_fpath):
        model = CameraModel()

        with open(src_fpath, 'r') as f:
            model.opencv_calib_ = json.load(f)

            model.mtx_ = np.array(model.opencv_calib_['mtx'])
            model.dist_ = np.array(model.opencv_calib_['dist'])

            print('Read camera intrinsics from {}'.format(src_fpath))
            print('mtx:\n{}\n'.format(model.mtx_))

        return model

    def computeOptimalMTXAndUndistort(self, src_img):
        h,  w = src_img.shape[:2]
        self.new_mtx_, self.roi_ = cv.getOptimalNewCameraMatrix(self.mtx_, self.dist_, (w,h), 1, (w,h))

        dst_img = cv.undistort(src_img, self.mtx_, self.dist_, None, self.new_mtx_)
        x, y, w, h = self.roi_
        dst_img = dst_img[y:y+h, x:x+w]

        print('Computed optimal MTX and rectified image')
        print('roi: {} {} {} {}'.format(x,y,w,h))
        print('new mtx:\n{}\n'.format(self.new_mtx_))

        return dst_img


def parseProgramArgs():
    parser = argparse.ArgumentParser(description='LiDAR - Camera extrinsic calibration.')
    parser.add_argument('pc', help='input PC')
    parser.add_argument('img', help='Input image')
    parser.add_argument('-i', '--intrinsic', help='Camera instirinsics output by OpenCV in JSON format', default='calib.json')
    parser.add_argument('-e', '--extrinsic', help='Extrinsic calibration: LiDAR pose in camera frame', default='calib_lidar_ref_cam.txt')
    parser.add_argument('-v', '--vis', help='Vizualisation', action="store_true")
    parser.add_argument('-o', '--dst', help='Output file')
    return parser.parse_args()

def colorize_from_file(pc,img,intrinsic,extrinsic,dst,visu=False):
    # 1. Read PC
    #--------------------------------------------------------------------------#
    pcd = o3d.io.read_point_cloud(pc)
    # 2. Read image
    #--------------------------------------------------------------------------#
    img = cv.imread(img)
    colorize(pcd,img,intrinsic,extrinsic,dst,visu)


def colorize(pcd,img,intrinsic,extrinsic,dst,visu=False):

    # 1. Read PC
    #--------------------------------------------------------------------------#
    pcd.paint_uniform_color(np.ones(3))

    print('Read point cloud.')
    print('num points: {}\n'.format(len(pcd.points)))

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)


    # 2. Read image
    #--------------------------------------------------------------------------#
    print('Read image')
    print('shape: {}\n'.format(img.shape))


    # 3. Read camera model and rectify iamge
    #--------------------------------------------------------------------------#
    cam_model = CameraModel.fromOpenCVIntrinsics(intrinsic)

    img = cam_model.computeOptimalMTXAndUndistort(img)


    # 4. Read camera - lidar calibration
    #--------------------------------------------------------------------------#
    C = np.loadtxt(extrinsic)
    print('Read camera lidar extrinsic calibration')
    print('{}\n'.format(C))


    # 5. Colorize PC
    #--------------------------------------------------------------------------#
    N = points.shape[0]
    W = img.shape[1]
    H = img.shape[0]
    p = np.ones(4)

    new_points = []
    new_colors = []

    for i in range(N):
        # point in lidar coords in homogeonous coordinates
        p[0:3] = points[i,:]
        # point in camera coords
        p_cam = C @ p
        z_cam = p_cam[2]

        if z_cam > 0.05:
            # point in normalized device coords
            p_ndc = np.array([p_cam[0] / z_cam, p_cam[1] / z_cam, 1.0])
            # projected point
            p_proj = cam_model.new_mtx_ @ p_ndc

            x = int(p_proj[0])
            y = int(p_proj[1])

            if x >= 0 and x < W and y >= 0 and y < H:
                col = img[y,x]

                colors[i] = 1.0 / 255.0 * col
                colors[i][0], colors[i][2] = colors[i][2], colors[i][0]

                new_points.append(points[i,:])
                new_colors.append(colors[i])


    # 5. Colorize PC
    #--------------------------------------------------------------------------#
    if not type(dst)==type(None):

        o3d.io.write_point_cloud(dst, pcd)
        print('Written colorized PC to {}\n'.format(dst))

    if visu:
        pcd_color = o3d.geometry.PointCloud()
        pcd_color.points = o3d.utility.Vector3dVector(new_points)
        pcd_color.colors = o3d.utility.Vector3dVector(new_colors)
        o3d.visualization.draw_geometries([pcd_color],zoom=0.001,
                                                    front=[-1,0,0],
                                                    lookat=[0,0,0],
                                                    up=[0,0,1])


def set_depth(depth,y,x,v):
    W = depth.shape[1]
    H = depth.shape[0]
    if x >= 0 and x < W and y >= 0 and y < H:
        depth[y,x] = v
    return depth

def depth(pcd,img,intrinsic,extrinsic):

    # 1. Read PC
    #--------------------------------------------------------------------------#
    pcd.paint_uniform_color(np.ones(3))

    print('Read point cloud.')
    print('num points: {}\n'.format(len(pcd.points)))

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)


    # 2. Read image
    #--------------------------------------------------------------------------#
    print('Read image')
    print('shape: {}\n'.format(img.shape))


    # 3. Read camera model and rectify iamge
    #--------------------------------------------------------------------------#
    cam_model = CameraModel.fromOpenCVIntrinsics(intrinsic)

    img = cam_model.computeOptimalMTXAndUndistort(img)


    # 4. Read camera - lidar calibration
    #--------------------------------------------------------------------------#
    C = np.loadtxt(extrinsic)
    print('Read camera lidar extrinsic calibration')
    print('{}\n'.format(C))


    # 5. Colorize PC
    #--------------------------------------------------------------------------#
    N = points.shape[0]
    W = img.shape[1]
    H = img.shape[0]
    p = np.ones(4)

    depth = -np.ones((H,W))

    for i in range(N):
        # point in lidar coords in homogeonous coordinates
        p[0:3] = points[i,:]
        # point in camera coords
        p_cam = C @ p
        z_cam = p_cam[2]

        if z_cam > 0.05:
            # point in normalized device coords
            p_ndc = np.array([p_cam[0] / z_cam, p_cam[1] / z_cam, 1.0])
            # projected point
            p_proj = cam_model.new_mtx_ @ p_ndc

            x = int(p_proj[0])
            y = int(p_proj[1])

            if x >= 0 and x < W and y >= 0 and y < H:
                #Compute depth
                dist = np.linalg.norm(p_cam)
                
                #Taking nearest point
                if(depth[y,x]!=-1):
                    dist = min(depth[y,x], dist)

                #Interpolate around pixel
                # for u in range(-5,6):
                #     for v in range(-5,6):
                #         depth = set_depth(depth,y+u,x+v,dist)
                depth = set_depth(depth,y,x,dist)
    #Showing result
    plt.figure()
    plt.imshow(depth,cmap='turbo')
    plt.colorbar()
    plt.title("Depth Estimation")
    plt.show()



if __name__== "__main__":

    args = parseProgramArgs()
    colorize_from_file(args.pc,args.img,args.intrinsic,args.extrinsic,args.dst,args.vis)
