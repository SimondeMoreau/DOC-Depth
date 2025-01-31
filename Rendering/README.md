## Rendering

To apply DOC-Depth composite rendering to the pre-processed dataset, use the following command :


```bash
    python render.py [traj_odometry.ply file] [traj_camera.ply file] [lidar_frame_directory] --intrinsic [calib.json file] --extrinsic [calib_lidar_ref_cam.txt file] -o [output_directory]
```


For more options see : 
```bash
    python render.py -h
```

You should now have your dataset with fully dense depth annotations !