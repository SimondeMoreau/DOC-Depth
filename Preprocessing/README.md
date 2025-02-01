# Preprocess recorded bag files
After recording, we must process the file before the rendering.

The rendering takes as input a set of classified frames and a trajectory. You can use [Exwayz](https://www.exwayz.fr/) software to generate these. For academic usage, you can apply to Exwayz academic program. 

You can use any low-drift SLAM software. File formats needed for the rest of the pipeline will be added soon.

## SLAM
Use the following command to run the SLAM on your bag and aggregate the LiDAR frames:

```bash
    ./exwayz_slam --bag [bag_file] -o [output_directory]
```

## DOC
To apply the dynamic object classification (DOC) please use this command :

```bash
    ./exwayz_map_cleaner --ply [frames_directory] -t [traj_odometry.ply file]
```

## Extract images from bag
We must extract each images from the bag for the downstream task and retrieve the timestamp for synchronisation. Please use the one of the following command: 

```bash
    python extract_images_from_bag.py [bag_file] [image_topic] [output_dir]
```

Or this one, if you want to undistort images :

```bash
    python extract_images_from_bag.py [bag_file] [image_topic] [output_dir] --intrinsic [calib.json]
```

## Interpolate camera trajectory
Since we assume only a software synchronisation between sensors, we must estimate the LiDAR position at each camera timestamp using:

```bash
    python camera_trajectory.py [traj_odometry.ply file] [images_dir] [output_file.ply]
```

You can now apply our composite rendering!