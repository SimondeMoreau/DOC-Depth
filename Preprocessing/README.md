# Preprocess recorded bag files

## SLAM

TODO

```bash
    ./exwayz_slam --bag [bag_file] -o [output_directory]
```

## DOC

TODO

```bash
    ./exwayz_map_cleaner(_2) --ply [frames_directory] -t [traj_odometry.ply file]
```

## Extract images from bag

TODO

```bash
    python extract_images_from_bag.py [bag_file] [image_topic] [output_dir]
```

If you want to undistort images :

```bash
    python extract_images_from_bag.py [bag_file] [image_topic] [output_dir] --intrinsic [calib.json]
```

## Interpolate camera trajectory

TODO 

```bash
    python camera_trajectory.py [traj_odometry.ply file] [images_dir] [output_file.ply]
```



TODO : Deal with redondant src directory