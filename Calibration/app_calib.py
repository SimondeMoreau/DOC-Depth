import cv2 as cv
import argparse
from src.camera import CameraOpenCV
from src.lidar import LidarOuster
from src.colorize_pc import colorize,depth

class AppCalib:

    def __init__(self,camera,lidar,calibration_filename):
        self.camera = camera
        self.lidar = lidar
        self.calibration_filename = calibration_filename
        
        self.record = False
        self.display_help = True

        self.use_lidar = True
        if(type(lidar)==type(None)):
            print("Starting without lidar")
            self.use_lidar = False
             
        self.print_help()

    #Switch between display mode and record mode
    def toggle_record(self, event=None, x=None, y=None, flags=None, param=None):
        if event == cv.EVENT_LBUTTONDOWN or event == None:
            self.record = not self.record

    def print_help(self):
        print("Click or Space : Capture Frame")
        print("c : Calibrate")
        print("u : Undistort first captured frame")
        print("r : Undistort current frame")
        print("p : Pick planes in lidar scans")
        print("l : Compute extrinsic lidar and camera matrix")
        print("t : Try a point cloud colorization from calibration")
        print("d : Try a depth esimation from calibration")
        print("h : Show this help")
        print("Escape : Exit App")

    def show_help(self,frame):
        font = cv.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        color = (0, 0, 0)
        thickness = 1
        frame = cv.putText(frame, 'Click or Space : Capture Frame', (5,15), font, fontScale, color, thickness, cv.LINE_AA)
        frame = cv.putText(frame, 'c : Calibrate Camera', (5,30), font, fontScale, color, thickness, cv.LINE_AA)
        frame = cv.putText(frame, 'u : Undistort first captured frame', (5,45), font, fontScale, color, thickness, cv.LINE_AA)
        frame = cv.putText(frame, 'r : Undistort current frame', (5,60), font, fontScale, color, thickness, cv.LINE_AA)
        frame = cv.putText(frame, 'p : Pick planes in lidar scans', (5,75), font, fontScale, color, thickness, cv.LINE_AA)
        frame = cv.putText(frame, 'l : Compute extrinsic lidar and camera matrix', (5,92), font, fontScale, color, thickness, cv.LINE_AA)
        frame = cv.putText(frame, 't : Try a point cloud colorization from calibration', (5,107), font, fontScale, color, thickness, cv.LINE_AA)
        frame = cv.putText(frame, 'd : Try a depth esimation from calibration', (5,122), font, fontScale, color, thickness, cv.LINE_AA)
        frame = cv.putText(frame, 'h : Show/Hide this help', (5,137), font, fontScale, color, thickness, cv.LINE_AA)
        frame = cv.putText(frame, 'Escape : Exit app', (5,152), font, fontScale, color, thickness, cv.LINE_AA)
        return frame

    
    def show_frame(self):
        showframe = cv.resize(self.camera.get_frame(), None, fx=0.8, fy=0.8, interpolation=cv.INTER_AREA)
        if self.display_help:
            showframe = self.show_help(showframe) 
        cv.imshow('Record Calibration', showframe)
        cv.setMouseCallback('Record Calibration', self.toggle_record, [0])

    def compare_frame(self,dst,ori):
        cv.imshow("Undistorded Image", dst)
        cv.imshow("Original Image", ori)

    def record_frame(self):
        retchess, corners = self.camera.find_chessboard(fast=True)
        # If found, add object points, image points (after refining them)
        if retchess == True:
            nb = self.camera.get_next_file_nb()
            self.camera.save(nb)
            print("Saving at nb = ", nb)
            if self.use_lidar:
                self.lidar.save(nb)
            self.camera.draw_chessboard()
            self.show_frame()
            cv.waitKey(1000)
            self.record = False

    def user_action(self):
            c = cv.waitKey(1)
            if c == ord(' '):
                self.toggle_record()
                return True
            if c == ord('c'):
                self.camera.calibrate(self.calibration_filename)
                self.show_frame()
                return True
            if c == ord('u'):
                dst,ori = self.camera.undistort()
                self.compare_frame(dst,ori)
                self.show_frame()
            if c == ord('r'):
                dst,ori = self.camera.undistort(True)
                self.compare_frame(dst,ori)
                self.show_frame()
                return True
            if c == ord('p'):
                self.lidar.pick_planes()
                self.show_frame()
                return True
            if c == ord('l'):
                self.lidar.calib_lidar_camera(self.calibration_filename)
                self.show_frame()
                return True 
            if c == ord('t'):
                self.camera.sample()
                self.lidar.sample()
                colorize(self.lidar.get_pcd(),self.camera.get_frame(),self.calibration_filename,"calib_lidar_ref_cam.txt",None,True)
                self.show_frame()
                return True
            if c == ord('d'):
                self.camera.sample()
                self.lidar.sample()
                depth(self.lidar.get_pcd(),self.camera.get_frame(),self.calibration_filename,"calib_lidar_ref_cam.txt")
                self.show_frame()
                return True
            if c == ord('h'):
                self.display_help = not self.display_help
                self.print_help()
                return True
            if c == 27: #Escape
                return False
            return True

    def run(self):
        self.camera.sample()
        self.show_frame()
        
        while True:
            self.camera.sample()
            self.show_frame()
            if self.record: 
                if self.use_lidar:
                    self.lidar.sample()
                self.record_frame()
                
            if not self.user_action():
                break

            if cv.getWindowProperty('Record Calibration', cv.WND_PROP_VISIBLE) < 1: # window closed
                break
            

        self.camera.release_cam()
        cv.destroyAllWindows()

    def compute_intrinsic(self):
        self.camera.calibrate(self.calibration_filename)

    def pick_planes(self):
        self.lidar.pick_planes()

    def compute_extrinsic(self):
        self.lidar.calib_lidar_camera(self.calibration_filename)


def parseProgramArgs():
    parser = argparse.ArgumentParser(description='LiDAR - Camera extrinsic calibration.')
    parser.add_argument('-d', '--video-device', help='OpenCV camera device number', default=4)
    parser.add_argument('-W', '--width', help='Camera image width', default=1920)
    parser.add_argument('-H', '--height', help='Camera image height', default=1080)
    parser.add_argument('-i', '--img_dir', help='Camera image output directory', default='images')
    parser.add_argument('-s', '--scan_dir', help='Lidar scan output directory', default='lidar_scans')
    parser.add_argument('-l', '--lidar-hostname', help='Lidar hostname', default='os-122320000354.local')
    parser.add_argument('-p', '--lidar-port', help='Lidar port', default=7502)
    parser.add_argument('-m', '--square-size', help='Size of square in chessboard (mm)', default=117.2)
    parser.add_argument('-n', '--no-lidar', help="Don't use lidar", action='store_true')
    parser.add_argument('-c', '--chessboard-size', help="Number of chessboard corners w,h. Example : -c 9 6", nargs='+', type=int, default=[9,6])
    parser.add_argument('-o', '--calibration-filename', help="Output camera calibration file", default='calib.json')
    parser.add_argument('--compute-intrinsic', help="Only compute camera intrinsic from files", action="store_true")
    parser.add_argument('--pick-planes', help="Only pick lidar planes from files", action="store_true")
    parser.add_argument('--compute-extrinsic', help="Only compute lidar camera extrinsic from files", action="store_true")
    
    return parser.parse_args()


if __name__ == "__main__":
    args = parseProgramArgs()

    no_lidar = args.no_lidar
    if(args.compute_intrinsic or args.pick_planes or args.compute_extrinsic):
        camera = CameraOpenCV(None,args.width,args.height,args.img_dir,args.chessboard_size, args.square_size)
        no_lidar = True
    else:
        camera = CameraOpenCV(args.video_device,args.width,args.height,args.img_dir,args.chessboard_size,args.square_size)
    
    lidar = None
    if(not no_lidar):
        print("Lidar port", args.lidar_port)
        lidar = LidarOuster(args.scan_dir,args.lidar_hostname,args.lidar_port)
    else:
        lidar = LidarOuster(args.scan_dir,None,None)

    app = AppCalib(camera,lidar,args.calibration_filename)

    
    if(args.compute_intrinsic):
        app.compute_intrinsic()
    elif(args.compute_extrinsic):
        app.compute_extrinsic()
    elif(args.pick_planes):
        app.pick_planes()
    else:
        app.run()
