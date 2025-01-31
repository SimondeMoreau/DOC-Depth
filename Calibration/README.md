## Calibration
The purpose of the calibration is to obtain simultaneously the intrisic matrix of the camera and the extrinsic calibration between the LiDAR and the camera.

## Harware
To perform this calibration, you need a camera, a LiDAR and a [large checkerboard target](https://github.com/opencv/opencv/blob/4.x/doc/pattern.png).
You'll need to perform a calibration process by recording multiple images of the checkerboard in front of the sensors with various locations and orientations. 

## Using Ouster LiDAR


If you have a Ouster LiDAR and a USB camera you can use our app_calib.py to perform a real-time recording of synchronized calibration frame of the two sensors. Start the tool using :

```bash
    python app_calib.py -d [video_device] -l [lidar_hostname] -p [lidar_port] -m [checkerboard_square_size] -c [checkerboard_size]
```

Then, you can click or use "Space" to trigger a recording, it will take a picture when it detect the checkerboard. 

## From Bag 

If you don't have a Ouster, prefer recording a bag with both your LiDAR and camera and run our tool to choose and extract relevant frames. 

```bash
    python extract_frame_from_bag.py [bag_file] [topic_im] [topic_lidar] [output_dir]
```

You can start/stop using "Space", use the arrows to navigate, press "s" to extract the current frame.

## Pick LiDAR planes

After your frames has been extracted, use the following tool to extract LiDAR frames.

```bash
    python app_calib.py --pick-planes
```

You must click on the target plane (one point) using "Shift+Click" and then type "Echap" for each extracted frames.

## Compute calibration

Finally, you can use the following commands to compute both calibration :

```bash
    python app_calib.py --compute-intrinsic
```


```bash
    python app_calib.py --compute-extrinsic
```

Congrats ! 