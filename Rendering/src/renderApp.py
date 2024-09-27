import numpy as np

from glumpy import app, gloo, gl,glm
from glumpy.transforms import PanZoom, Position,Trackball
from glumpy.ext import png
from glumpy.app import clock

import json

from .frameManager import FrameManager

from .utils import *

import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
import cv2



class RenderApp:

    def __init__(self, width, height, shader_file_vertex, shader_file_fragment, pts_per_frame,
                traj_cam, traj_lidar, frame_folder, cam_step, dist_merge, max_depth, min_depth, dist_past,
                classif_property_name, intrinsic_file, extrinsic_file, max_pts_size, min_pts_size, max_pts_size_dyn, min_pts_size_dyn,
                start_index, output_mode, output, crop_norm, scan_mode):
        self.index = start_index
        self.width = width
        self.height = height
        self.shader_file_vertex = shader_file_vertex
        self.shader_file_fragment = shader_file_fragment
        self.pts_per_frame = pts_per_frame
        self.max_depth = max_depth
        self.min_depth = min_depth #TODO: add as argument
        self.max_pts_size = max_pts_size
        self.min_pts_size = min_pts_size
        self.max_pts_size_dyn = max_pts_size_dyn
        self.min_pts_size_dyn = min_pts_size_dyn
        self.output_mode = output_mode # depth or image or kitti
        self.output = output
        self.scan_mode = scan_mode

        if self.output is not None:
            os.makedirs(self.output,exist_ok=True)
 
        # read the shaders
        vertex = self.read_shader(shader_file_vertex)
        fragment = self.read_shader(shader_file_fragment)

        # Create the window
        self.window = app.Window(width, height, color=(1,1,1,1))
        self.window_var = gloo.Program(vertex, fragment, count=pts_per_frame)

        self.frameManager = FrameManager(traj_cam, traj_lidar, frame_folder, pts_per_frame, cam_step, dist_merge, max_depth, dist_past, classif_property_name, crop_norm)


        # Read camera intrinsic
        self.K = np.array(json.load(open(intrinsic_file))["mtx"])

        # Read camera and lidar extrinsic
        self.C = np.loadtxt(extrinsic_file)


        self.setWindowVar()

        @self.window.event
        def on_init():
            gl.glEnable(gl.GL_DEPTH_TEST)

        @self.window.event
        def on_draw(dt):
            self.update(dt)

    def setWindowVar(self):
        self.window_var["points"] = np.zeros((self.pts_per_frame, 3))
        self.window_var["max_dist"] = self.max_depth
        self.window_var["T"] = np.eye(4)
        self.window_var["C"] = self.C.T
        self.window_var["K"] = OpenCVMtxToOpenGLMTx(self.K,self.width,self.height,self.max_depth,self.min_depth).T
        self.window_var["max_pts_size"] = self.max_pts_size
        self.window_var["min_pts_size"] = self.min_pts_size


    def read_shader(self,file):
        with open(file) as f:
            return f.read()

    def update(self,dt):
        self.window.clear()


        # Render the frames
        if self.scan_mode:
            self.renderScanDepth()
        else:
            self.renderDenseDepth()

        # Save the output
        if self.output is not None:
            pose_cam = self.frameManager.getCameraPose(self.index)
            filename = os.path.join(self.output,str(str(pose_cam.index).zfill(6)))
            self.saveOutput(filename)  

        self.printStatus()
        self.index += 1
        #TODO: add a stop condition
    
    def renderDenseDepth(self):
        # Load the frames
        self.frameManager.loadFrames(self.index)

        # Render static frames
        self.window_var["max_pts_size"] = self.max_pts_size
        self.window_var["min_pts_size"] = self.min_pts_size
        frames = self.frameManager.getStaticFrames()
        pose_cam = self.frameManager.getCameraPose(self.index)
        pcam = pose_cam.transform.matrix()
        for i,frame in enumerate(frames):
            self.renderFrame(pcam,frame)

        # Render dynamic frame
        self.window_var["max_pts_size"] = self.max_pts_size_dyn
        self.window_var["min_pts_size"] = self.min_pts_size_dyn
        dynamic_frame = self.frameManager.getDynamicFrame()
        if dynamic_frame is not None:
            self.renderFrame(pcam,dynamic_frame)

    def renderScanDepth(self):
        # Render current pose
        self.window_var["max_pts_size"] = self.max_pts_size
        self.window_var["min_pts_size"] = self.min_pts_size

        frame = self.frameManager.getCurrentTrame(self.index)
        pose_cam = self.frameManager.getCameraPose(self.index)
        pcam = pose_cam.transform.matrix()
        self.renderFrame(pcam,frame)


    def renderFrame(self,pcam,frame):
        trame = frame["trame"]
        plidar = frame["pose"].transform.matrix()

        self.window_var["points"] = trame
        self.window_var["T"] = (np.linalg.inv(pcam) @ plidar).T

        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthRange(0.0, 1.0)
        gl.glDepthFunc(gl.GL_LESS)
        self.window_var.draw(gl.GL_POINTS)

    def saveOutput(self,filename):
        if self.output_mode.lower() == "image":
            self.saveImage(filename)
        elif self.output_mode.lower() == "kitti":
            self.saveKitti(filename)
        else:
            self.saveDepth(filename)
        
    def saveImage(self,filename):
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        gl.glReadPixels(0, 0, self.width, self.height,
                        gl.GL_RGB, gl.GL_UNSIGNED_BYTE, rgb)
        rgb = rgb[::-1,:]
        rgb = rgb.reshape((self.height,self.width,3))

        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGRA)
        
        invalid_mask = np.all(rgb==255,axis=-1)
        rgb[invalid_mask,3] = 0


        if not filename.endswith(".png") or not filename.endswith(".jpg"):
            filename += ".png"

        cv2.imwrite(filename,rgb)


    def saveDepth(self,filename):
        framebuffer = np.zeros((self.height, self.width), dtype=np.float32)
        gl.glReadPixels(0, 0, self.width, self.height,
                        gl.GL_DEPTH_COMPONENT, gl.GL_FLOAT, framebuffer)
        framebuffer = framebuffer[::-1,:]
        framebuffer = framebuffer.reshape((self.height,self.width))
        invalid_mask = framebuffer == 1

        im = framebuffer.astype("float32") * self.max_depth
        im[invalid_mask] = np.inf

        if not filename.endswith(".exr"):
            filename += ".exr"

        cv2.imwrite(filename,im,[cv2.IMWRITE_EXR_TYPE, cv2.IMWRITE_EXR_TYPE_HALF])

    def saveKitti(self,filename):
        framebuffer = np.zeros((self.height, self.width), dtype=np.float32)
        gl.glReadPixels(0, 0, self.width, self.height,
                        gl.GL_DEPTH_COMPONENT, gl.GL_FLOAT, framebuffer)
        framebuffer = framebuffer[::-1,:]
        framebuffer = framebuffer.reshape((self.height,self.width))
        invalid_mask = framebuffer == 1

        im = framebuffer.astype("float32") * self.max_depth
        im[invalid_mask] = np.inf

        if not filename.endswith(".png"):
            filename += ".png"

        im = (im * 256).astype(np.uint16)
        cv2.imwrite(filename,im)


    def printStatus(self):
        fps = "{:.2f}".format(clock.get_fps())
        print("FPS: ", fps, "| Image : ", self.index, "/", self.frameManager.len())

    def run(self):
        app.run()
