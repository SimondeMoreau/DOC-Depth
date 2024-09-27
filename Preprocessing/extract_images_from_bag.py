import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import argparse
import json

def parseProgramArgs():
    parser = argparse.ArgumentParser(description='Extract images from bag file.')
    parser.add_argument('bag', help='Rosbag file')
    parser.add_argument('topic', help='Image topic')
    parser.add_argument('output_dir', help='Output directory')
    parser.add_argument('--intrinsic', help='Camera intrinsics file to undistort images', default=None)
    return parser.parse_args()

if __name__ == '__main__':

    args = parseProgramArgs()

    if(not os.path.exists(args.output_dir)):
        os.makedirs(args.output_dir)

    undistort = False
    if args.intrinsic is not None:
        intrinsic = json.load(open(args.intrinsic))
        mtx = np.array(intrinsic['mtx'])
        dist = np.array(intrinsic['dist'])
        undistort = True

    bag = rosbag.Bag(args.bag)
    timestamps = []

    is_theora = args.topic.endswith("compressed")
    bridge = CvBridge()
    image_topic = bag.read_messages(args.topic)
    for k, b in enumerate(image_topic):
        msg = b.message
        if is_theora:
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


        if undistort:
            h,  w = cv_image.shape[:2]
            newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
            cv_image = cv2.undistort(cv_image, mtx, dist, None, newcameramtx)
            x,y,w,h = roi
            cv_image = cv_image[y:y+h, x:x+w]

        filename = os.path.join(args.output_dir,"%06d"%k +".jpg")
        timestamps.append(int(str(b.timestamp))/1e9)
        cv2.imwrite(filename, cv_image)
        print("Saving : ", filename)

    bag.close()

    if undistort:
        new_calib = {
            "mtx": newcameramtx.tolist(),
            "dist": [0] * len(dist),
        }
        filename = os.path.join(args.output_dir,"calib_undistort.json")
        print("Saving undistorted intrinsic in ", filename)
        with open(filename, 'w') as outfile:
            json.dump(new_calib, outfile)

    filename = os.path.join(args.output_dir,"timestamps.txt")
    print("Saving timestamps in ", filename)
    np.savetxt(filename,np.array(timestamps))

    print('Process complete, image saved : ', k)