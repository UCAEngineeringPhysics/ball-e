import pyrealsense2 as rs

# Configure and start pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Define pixel coordinates (x, y)
        x, y = 320, 240  # Example: center of image

        # Get depth (Z) in meters
        depth_in_meters = depth_frame.get_distance(x, y)

        # Get intrinsics of depth stream
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        # Convert 2D pixel to 3D point (X, Y, Z in meters)
        point = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth_in_meters)

        print(f"3D World Coordinates (X, Y, Z): {point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f} meters")
finally:
    pipeline.stop()   
