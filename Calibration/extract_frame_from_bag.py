import os 
import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np
import argparse
import glob
from plyfile import PlyData, PlyElement


def writePly(filename, pc):
    has_i = False
    if pc.shape[1] == 4:
        has_i = True

    if has_i:
        vertex = np.zeros(len(pc), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity','f4')])
        vertex['intensity'] = pc[:,3]
    else:
        vertex = np.zeros(len(pc), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    vertex['x'] = pc[:,0]
    vertex['y'] = pc[:,1]
    vertex['z'] = pc[:,2]
    el = PlyElement.describe(vertex, 'vertex')
    PlyData([el]).write(filename)
    
def extract_one_image(bag_file, topic, output_dir, t):
    theora = topic.endswith("compressed")
    output_dir = os.path.join(output_dir, "images")
    bag = rosbag.Bag(bag_file, 'r')
    bridge = CvBridge()
    count = len(glob.glob1(output_dir, "*.jpg"))
    try: 
        topic, msg, t = next(bag.read_messages(topics=[topic], start_time=t))
    except StopIteration:
        print("No messages in topic %s" % topic)
    
    cv_image = get_im(msg, bridge, theora)
    filename = os.path.join(output_dir, "%i.jpg" % count)
    cv2.imwrite(filename, cv_image)
        
    bag.close()
    return cv_image

def extract_one_lidar(bag_file, topic, output_dir, t):
    output_dir = os.path.join(output_dir, "lidar_scans")
    bag = rosbag.Bag(bag_file, 'r')
    count = len(glob.glob1(output_dir, "*.ply"))
    try: 
        topic, msg, t = next(bag.read_messages(topics=[topic], start_time=t))
    except StopIteration:
        print("No messages in topic %s" % topic)
    
    pc = read_pc_msg(msg)    

    filename = os.path.join(output_dir, "%i.ply" % count)
    writePly(filename, pc)
    
    bag.close()
    return pc

def read_pc_msg(msg):
    # convert to numpy array msg.data
    point_step = msg.point_step
    offset_xyz = [0, 4, 8]
    offset_intensity = -1
    #print(msg.fields)
    for f in msg.fields:
        if f.name == "x":
            offset_xyz[0] = f.offset
        elif f.name == "y":
            offset_xyz[1] = f.offset
        elif f.name == "z":
            offset_xyz[2] = f.offset
        elif f.name == "intensity":
            offset_intensity = f.offset
    pc = []
    nb_points = int(len(msg.data) / point_step)
    for i in range(nb_points):
        offset = i * point_step
        x = np.frombuffer(msg.data, dtype=np.float32, count=1, offset=offset + offset_xyz[0])
        y = np.frombuffer(msg.data, dtype=np.float32, count=1, offset=offset + offset_xyz[1])
        z = np.frombuffer(msg.data, dtype=np.float32, count=1, offset=offset + offset_xyz[2])
        if offset_intensity > 0:
            i = np.frombuffer(msg.data, dtype=np.float32, count=1, offset=offset + offset_intensity)
            pc.append([x, y, z, i])
        else:
            pc.append([x,y,z])
    
    pc = np.array(pc).reshape(-1, len(pc[0]))
    return pc


def get_im(msg, bridge, theora=False):
    if theora:
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    else:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    return cv_image

def get_lidar_im(msg):
    pc = read_pc_msg(msg)
    #Fake intrinsics TODO: find a general way to handle projection
    K = np.array([[1614.5606189613611, 0.0, 940.140768866003], 
                  [0.0, 1614.303435111037, 523.0433416805166], 
                  [0.0, 0.0, 1.0]])
    T = np.array([[0.0, -1.0, 0.0], 
                  [0.0, 0.0, -1.0], 
                  [1.0, 0.0, 0.0]])
    intensity = None
    if pc.shape[1] == 4: # intensity
        intensity = pc[:,3]
        pc = pc[:,:3]

    # Project to image plane
    px = (T @ pc.T).T
    px = (K @ px.T).T

    # Get normalized coordinates
    z = px[:,2]
    px = px / z.reshape(-1, 1)
    mask = z > 0
    px = px[mask]
    z = z[mask]
    if intensity is not None:
        intensity = intensity[mask]

    # Filter out points outside of image
    px = px[:, :2].astype(np.int32)
    mask = (px[:,0] >= 0) & (px[:,0] < 1920) & (px[:,1] >= 0) & (px[:,1] < 1080)
    px = px[mask]
    z = z[mask]
    if intensity is not None:
        intensity = intensity[mask]

    v_max = z.max() if intensity is None else intensity.max()
    im = np.ones((1080, 1920), dtype=np.uint8) * 255

    for i in range(-1,2): # 3x3 kernel for densification
        for j in range(-1,2):
            coor = px + np.array([i, j])
            mask = (coor[:,0] >= 0) & (coor[:,0] < 1920) & (coor[:,1] >= 0) & (coor[:,1] < 1080)
            coor = coor[mask]
            z_d = z[mask]
            if intensity is not None:
                intensity_d = intensity[mask]
                v = (intensity_d / v_max * 255).astype(np.uint8)
            else:
                v = (z_d / v_max * 255).astype(np.uint8)
                
            im[coor[:,1], coor[:,0]] = np.minimum(im[coor[:,1], coor[:,0]], v)

    im[im == 255] = 0
    im = im.astype(np.uint8)
    im = cv2.applyColorMap(im, cv2.COLORMAP_VIRIDIS)

    return im
    



def show_frames(bag_file, topic_im,topic_lidar, output_dir, show_lidar):
    bag = rosbag.Bag(bag_file, 'r')
    count = 0
    if not show_lidar:
        bridge = CvBridge()
        theora = topic_im.endswith("compressed")
        topic = topic_im
        iter = bag.read_messages(topics=[topic])
    else:
        topic = topic_lidar
        iter = bag.read_messages(topics=[topic])

    ts = []
    pause = True
    
    while True:
        try:
            topic, msg, t = next(iter)
            if len(ts)==0 or t>ts[-1]:
                ts.append(t)
        except StopIteration:
            break

        if not show_lidar:
            cv_image = get_im(msg, bridge, theora)
        else:
            cv_image = get_lidar_im(msg)
        cv2.imshow('ExtractFrames', cv_image)
        if not pause:
            key = cv2.waitKey(1)
        else:
            key = cv2.waitKey(100)
            while key==-1:
                key = cv2.waitKey(100)
                if cv2.getWindowProperty('ExtractFrames', cv2.WND_PROP_VISIBLE) < 1: # window closed
                    break
        if key == 27:  # escape key
            break
        elif key == 32:  # space bar
            pause = not pause
        elif key == 81:  # left arrow key
            iter = bag.read_messages(topics=[topic], start_time=ts[count-1])
            count -= 1
            continue
        elif key == 115:  # 's' key
            print("Save frame %i" % count)
            extract_one_image(bag_file, topic_im, output_dir, ts[count])
            extract_one_lidar(bag_file, topic_lidar, output_dir, ts[count])
        count += 1

        if cv2.getWindowProperty('ExtractFrames', cv2.WND_PROP_VISIBLE) < 1: # window closed
            break
    bag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract frames from a ROS bag.')
    parser.add_argument('bag_file', help='input ROS bag')
    parser.add_argument('topic_im', help='topic image to extract')
    parser.add_argument('topic_lidar', help='topic lidar to extract')
    parser.add_argument('output_dir', help='output directory')
    parser.add_argument('--show-lidar', action='store_true', help='Show images from lidar scans instead of camera')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    os.makedirs(args.output_dir + "/images", exist_ok=True)
    os.makedirs(args.output_dir + "/lidar_scans", exist_ok=True)

    show_frames(args.bag_file, args.topic_im,args.topic_lidar ,args.output_dir, args.show_lidar)
