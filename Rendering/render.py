from src.renderApp import RenderApp
import argparse

def parseProgramArgs():
    parser = argparse.ArgumentParser(description='Depth Map Estimation in Camera point of view using Lidar data')
    parser.add_argument('lidar_trajectory', help='Lidar trajectory odometry')
    parser.add_argument('camera_trajectory', help='Camera trajectory')
    parser.add_argument('frames', help='Folder with lidar frames')
    parser.add_argument('-o', '--output', help='Output directory')    
    parser.add_argument('--output-mode', help='Output mode [depth,image,kitti]', default="depth")

    parser.add_argument('--width', help='Width of the image', default=1920,type=int)
    parser.add_argument('--height', help='Height of the image', default=1080,type=int)
    parser.add_argument('--shader-vertex', help='Vertex shader', default="shaders/vertex.c")
    parser.add_argument('--shader-fragment', help='Fragment shader', default="shaders/fragment_ellipsis.c")
    parser.add_argument('--pts-per-frame', help='Number of points per frame', default=150000,type=int)
    parser.add_argument('--cam-step', help='Distance step between camera poses', default=1.0,type=float)
    parser.add_argument('--dist-merge', help='Distance between aggregated frames in the depth', default=0.01,type=float)
    parser.add_argument('--max-depth', help='Maximum distance considered in the depth (z_far)' , default=100,type=float)
    parser.add_argument('--min-depth', help='Minimum distance considered in the depth (z_near)', default=0.1,type=float)
    parser.add_argument('--dist-past', help='Maximum distance before the current pose considered in the depth for the merge' , default=0.001,type=float)
    parser.add_argument('--classif-property-name', help='Classif static property', default="classid",type=str)
    parser.add_argument('--intrinsic', help='Intrinsic camera calibration file', default="calib.json")
    parser.add_argument('--extrinsic', help='Extrinsic camera and lidar calibration file', default="calib_lidar_ref_cam.txt")
    parser.add_argument('--max-pts-size', help='Maximum size of the static points', default=50.0,type=float)    
    parser.add_argument('--min-pts-size', help='Minimum size of the static points', default=3.0,type=float)
    parser.add_argument('--max-pts-size-dyn', help='Maximum size of the dynamic points', default=80,type=float)    
    parser.add_argument('--min-pts-size-dyn', help='Minimum size of the dynamic points', default=35,type=float)
    parser.add_argument('--start-index', help='Start frame', default=0,type=int)
    parser.add_argument('--crop-norm', help='Max point norm in merged frames', default=300.0,type=float)
    parser.add_argument('--scan-only', help='Project only current LiDAR frame', action='store_true')

    return parser.parse_args()

if __name__ == '__main__':
    args = parseProgramArgs()
    
    app = RenderApp(args.width,args.height,args.shader_vertex,args.shader_fragment,
                    args.pts_per_frame,args.camera_trajectory,args.lidar_trajectory, args.frames,
                    args.cam_step,args.dist_merge,args.max_depth,args.min_depth,args.dist_past,args.classif_property_name,
                    args.intrinsic,args.extrinsic,args.max_pts_size,args.min_pts_size,args.max_pts_size_dyn,args.min_pts_size_dyn, 
                    args.start_index, args.output_mode, args.output, args.crop_norm, args.scan_only
                    )

    app.run()