for /l %%i in (1, 1, 20) do (
    python colorize_pc.py ..\lidar_scans\%%i.ply ..\images\%%i.jpg -o ..\lidar_scans_colorized\%%i.ply
)