#!/usr/bin/env python3

import os
import sys
import cv2
import numpy as np
import torch
import tensorrt as trt
import time

# ROS imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
# from cv_bridge import CvBridge
import message_filters

# Add the external directories to the python path
script_dir = os.path.dirname(os.path.realpath(__file__))
multi_cam_dir = os.path.abspath(os.path.join(script_dir, '..', '..', '..', '..'))
external_dir = os.path.join(multi_cam_dir, 'external')
sys.path.insert(0, external_dir)

# Foundation Stereo imports
try:
    from FoundationStereo.scripts import tensorrt_engine, downscale_image
except ImportError as e:
    tensorrt_engine, downscale_image = None, None
    print(f"Warning: Could not import from FoundationStereo.scripts: {e}")


def make_divisible(dimension, n):
    """Round dimension to nearest multiple of n"""
    return round(dimension / n) * n

def rosimg_to_numpy(msg):
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    buf = memoryview(msg.data)

    if enc in ('bgr8', 'rgb8'):
        arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 3)
    elif enc in ('mono8', '8uc1'):
        arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, w)
        # promote to 3ch BGR because your preprocessing expects 3 channels
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")
    return arr

class FoundationStereoNode(Node):
    def __init__(self):
        super().__init__('foundation_stereo_node')
        
        # Declare parameters
        self.declare_parameter('height', 1080)
        self.declare_parameter('width', 1440)
        self.declare_parameter('focal_length', 858.0)
        self.declare_parameter('baseline', 0.06)
        self.declare_parameter('scale_factor', 0.3)  # Approximate scale factor for input images
        self.declare_parameter('vit_size', "small")  # Model sizes: [small, large]

        self.focal_length = float(self.get_parameter('focal_length').value)
        self.baseline = float(self.get_parameter('baseline').value)

        # Calculate the target dimensions for the model input
        target_scale = self.get_parameter('scale_factor').value
        original_height = self.get_parameter('height').value
        original_width = self.get_parameter('width').value
        self.target_height = int(original_height * target_scale)
        self.target_width = int(original_width * target_scale)

        # Adjust to be divisible by 224
        self.final_height = make_divisible(self.target_height, 224)
        self.final_width = make_divisible(self.target_width, 224)

        # Compute the actual scale factors
        self.scale_h = self.final_height / original_height
        self.scale_w = self.final_width / original_width
        self.get_logger().info(
            f"Input image will be resized from "
            f"{original_width}x{original_height} to "
            f"{self.final_width}x{self.final_height} "
            f"(scale factors: w={self.scale_w:.3f}, h={self.scale_h:.3f})"
        )

        # Find an available model given the target dimensions
        # fs_672x896_vit-small_iters16.plan
        models_dir = os.path.join(external_dir, 'FoundationStereo', 'pretrained_models')

        # From the available models, find one that matches the input sizes and vit_size
        vit_size = self.get_parameter('vit_size').value
        models_list = os.listdir(models_dir)

        for model_name in models_list:
            if model_name.endswith(".plan") and str(self.final_width) in model_name and str(self.final_height) in model_name and vit_size in model_name:
                self.model_path = os.path.join(models_dir, model_name)
                self.get_logger().info(f"Using model: {model_name}")
                break
        else:
            self.get_logger().warn("No suitable model found.")
            self.model_path = None

        # Load the model
        if self.model_path:
            self.model = self.load_model(self.model_path)
            self.get_logger().info(f'Loaded model from {self.model_path}')
        else:
            self.get_logger().warn(f'No valid model path provided: {self.model_path}')

        # Initialize CV bridge
        # self.bridge = CvBridge()
        
        # Create synchronized subscribers
        self.left_image_sub = message_filters.Subscriber(
            self, Image, '/firefly_left/image_rect_mono')
        self.right_image_sub = message_filters.Subscriber(
            self, Image, '/firefly_right/image_rect_mono')
        
        # Synchronize messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_image_sub, self.right_image_sub],
            queue_size=10,
            slop=0.1,
            allow_headerless=False
        )
        self.ts.registerCallback(self.stereo_callback)
        
        # Create publisher
        self.disparity_pub = self.create_publisher(
            DisparityImage, '/firefly/disparity_custom', 10)
        
        self.get_logger().info('Foundation Stereo Node initialized')
    
    def load_model(self, model_path):
        if model_path:
            if tensorrt_engine is None:
                self.get_logger().error('tensorrt_engine module not available')
                return None
            with open(model_path, 'rb') as f:
                engine_data = f.read()
            engine = trt.Runtime(trt.Logger(trt.Logger.WARNING)).deserialize_cuda_engine(engine_data)
            model = tensorrt_engine.Engine(engine)
            return model
        else:
            self.get_logger().error(f'No valid model path provided: {model_path}')
            return None
    
    def preprocess_image(self, cv_image):
        # resize if needed first (OpenCV is fast)
        if self.scale_h != 1.0 or self.scale_w != 1.0:
            cv_image = cv2.resize(cv_image, (self.final_width, self.final_height), interpolation=cv2.INTER_AREA)

        # ensure 3 channels
        if cv_image.ndim == 2:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        chw = cv_image.transpose(2, 0, 1)                       # (3,H,W)
        chw = np.ascontiguousarray(chw, dtype=np.float32)       # contiguous, float32
        # if your .plan expects [0,1] input, uncomment:
        # chw *= (1.0/255.0)
        return chw[np.newaxis, ...]                              # (1,3,H,W)

    def compute_disparity(self, left_img, right_img):
        """Run model inference"""
        if self.model is None:
            self.get_logger().error('Model not loaded')
            return None
        
        # start_time = time.time()
        # torch.cuda.synchronize()
        disparity = self.model.run([left_img, right_img])[0]
        # torch.cuda.synchronize()
        # elapsed = time.time() - start_time
        
        # self.get_logger().info(
        #     f'Inference time: {elapsed*1000:.1f} ms ({1.0/elapsed:.1f} Hz)',
        #     throttle_duration_sec=1.0
        # )
        
        return disparity.squeeze()  # Return HxW array
    
    def scale_disparity(self, disparity, scale):
        """Scale disparity map back to original resolution with sparse values
        
        This creates a sparse disparity map where only the upscaled pixel locations
        have values, and everything in between is NaN. No interpolation is performed.
        Uses vectorized operations for speed.
        """
        sh, sw = scale
        H, W = disparity.shape
        out_h = int(round(H * sh))
        out_w = int(round(W * sw))
        
        # Create output array filled with NaN
        scaled_disparity = np.full((out_h, out_w), np.nan, dtype=np.float32)
        
        # Create coordinate grids for all pixels
        v_coords, u_coords = np.mgrid[0:H, 0:W]
        
        # Calculate output coordinates (vectorized)
        v_out = np.round(v_coords * sh).astype(np.int32)
        u_out = np.round(u_coords * sw).astype(np.int32)
        
        # Create mask for valid disparities
        valid_mask = disparity > 0
        
        # Apply bounds checking
        bounds_mask = (v_out >= 0) & (v_out < out_h) & (u_out >= 0) & (u_out < out_w)
        final_mask = valid_mask & bounds_mask
        
        # Get valid coordinates and disparity values
        v_valid = v_out[final_mask]
        u_valid = u_out[final_mask]
        d_valid = disparity[final_mask] * sw
        
        # Assign values (vectorized)
        scaled_disparity[v_valid, u_valid] = d_valid
        
        return scaled_disparity
    
    def stereo_callback(self, left_image, right_image):
        """Process synchronized stereo pair"""
        try:
            # Convert ROS images to OpenCV
            # left_cv = self.bridge.imgmsg_to_cv2(left_image, desired_encoding='bgr8')
            # right_cv = self.bridge.imgmsg_to_cv2(right_image, desired_encoding='bgr8')
            left_cv  = rosimg_to_numpy(left_image)
            right_cv = rosimg_to_numpy(right_image)

            print(f'Left image shape: {left_cv.shape}, Right image shape: {right_cv.shape}')
            
            # Preprocess
            t0 = time.time()
            left_img = self.preprocess_image(left_cv)
            right_img = self.preprocess_image(right_cv)

            # Check if preprocessing was successful
            print(f'Preprocessed left shape: {left_img.shape}, right shape: {right_img.shape}')

            # Compute disparity
            t1 = time.time()
            disparity = self.compute_disparity(left_img, right_img)

            if disparity is None:
                return
            
            # # Debug: check what we got
            # self.get_logger().info(
            #     f'Disparity shape: {disparity.shape}, dtype: {disparity.dtype}, type: {type(disparity)}',
            #     throttle_duration_sec=2.0
            # )
            
            # Scale the disparity back to original size
            t2 = time.time()
            if self.scale_h != 1.0 or self.scale_w != 1.0:
                disparity = self.scale_disparity(disparity, (1/self.scale_h, 1/self.scale_w))
            t3 = time.time()
            self.get_logger().info(f"prep {(t1-t0)*1e3:.1f} ms | infer {(t2-t1)*1e3:.1f} ms | post {(t3-t2)*1e3:.1f} ms", throttle_duration_sec=0.5)
            
            # Create disparity message
            disparity_msg = DisparityImage()
            disparity_msg.header = left_image.header
            
            # Set image data
            disparity_msg.image.header = left_image.header
            disparity_msg.image.height = disparity.shape[0]
            disparity_msg.image.width = disparity.shape[1]
            disparity_msg.image.encoding = '32FC1'
            disparity_msg.image.is_bigendian = False
            disparity_msg.image.step = disparity.shape[1] * 4  # 4 bytes per float32
            
            # Convert to float32 and flatten
            disparity_float32 = disparity.astype(np.float32)
            disparity_msg.image.data = disparity_float32.tobytes()
            
            # Set disparity parameters
            disparity_msg.f = self.focal_length
            disparity_msg.t = self.baseline
            disparity_msg.valid_window.x_offset = 0
            disparity_msg.valid_window.y_offset = 0
            disparity_msg.valid_window.width = disparity.shape[1]
            disparity_msg.valid_window.height = disparity.shape[0]
            disparity_msg.min_disparity = float(np.min(disparity[disparity > 0])) if np.any(disparity > 0) else 0.0
            disparity_msg.max_disparity = float(np.max(disparity))
            disparity_msg.delta_d = 1.0
            
            # Publish
            self.disparity_pub.publish(disparity_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing stereo images: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = FoundationStereoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
