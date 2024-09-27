## Calibration

lorem 
We need to get intrinsic and extrinsic calib for camera and LiDAR... 


## Harware

Explain setup ?

## Using Ouster LiDAR

```bash
    python app_calib.py -d [video_device] -l [lidar_hostname] -p [lidar_port] -m [checkerboard_square_size] -c [checkerboard_size]
```

TODO : Explain how to use the app

## From Bag 


```bash
    python extract_frame_from_bag.py [bag_file] [topic_im] [topic_lidar] [output_dir]
```

TODO : Explain how to use the app (ie use arrow/space to play and press "s" to save)


## Pick LiDAR planes

```bash
    python app_calib.py --pick-planes
```

TODO : Explain picking phase

## Compute calibration

```bash
    python app_calib.py --compute-intrinsic
```


```bash
    python app_calib.py --compute-extrinsic
```


Congrats ! 