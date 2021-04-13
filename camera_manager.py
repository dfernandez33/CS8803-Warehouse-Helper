import pyrealsense2 as rs
import numpy as np


class CameraManager:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.depth_scale = self.device.first_depth_sensor().get_depth_scale()
        # this is used to align the depth and color images since they are captured
        # from slightly different viewports
        self.aligner = rs.align(rs.stream.color)
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

    def get_frames(self, align=False):
        depth_frame = None
        color_frame = None
        while not depth_frame or not color_frame:
            frames = self.pipeline.wait_for_frames()
            if align:
                frames = self.aligner.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            # Convert images to numpy arrays
            curr_depth_frame = np.asanyarray(depth_frame.get_data())
            curr_rgb_frame = np.asanyarray(color_frame.get_data())
        
        return curr_rgb_frame, curr_depth_frame
