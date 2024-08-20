import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
# Import math Library
import math 

class PointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('point_cloud_visualizer')
        self.pc_subscription = self.create_subscription(
            PointCloud2,
            '/visual_slam/vis/observations_cloud',
            self.pc_listener_callback,
            10
        )
        self.image_subscription = self.create_subscription(
            Image,
            #'/front_stereo_camera/left_rgb/image_raw',
            '/camera/infra1/image_rect_raw',
            self.image_listener_callback,
            10
        )
        self.image_publisher = self.create_publisher(Image, '/visual_slam/camera', 10)
        self.bridge = CvBridge()
        self.points = None
        self.image = None

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pc_listener_callback(self, msg):
        # Convert PointCloud2 message to a structured numpy array
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points_list:
            return

        # Manually extract x, y, z coordinates
        x = np.array([point[0] for point in points_list], dtype=np.float32)
        y = np.array([point[1] for point in points_list], dtype=np.float32)
        z = np.array([point[2] for point in points_list], dtype=np.float32)

        # Combine x, y, z into a single numpy array
        points = np.stack((x, y, z), axis=-1)

        # Debugging: print the shape and first few elements of points
        self.get_logger().info(f'Point cloud array shape: {points.shape}')
        self.get_logger().info(f'First few points: {points[:5]}')

        # Check if the array is 2-dimensional
        if points.ndim != 2 or points.shape[1] != 3:
            self.get_logger().error('Unexpected point cloud array shape.')
            return

        self.points = points

        # Try to draw keypoints if image is already received
        if self.image is not None:
            self.draw_keypoints()

    def image_listener_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Try to draw keypoints if points are already received
        if self.points is not None:
            self.draw_keypoints()

    def draw_keypoints(self):
        try:
            # Lookup the transform from the camera frame to the base link
            transform = self.tf_buffer.lookup_transform('camera_link', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'TF Lookup Exception: {e}')
            return

        # Define intrinsic parameters 
        ## the fx, fy should be = focal length*(imgshape[0 or 1]/2)/np.tan(FOV in radians/2)
        fx = 1.6 * (self.image.shape[1]/2) / (np.tan(np.deg2rad(121.5)/2))  # Focal length in x (pixels)
        fy = 1.6 * (self.image.shape[0]/2) / (np.tan(np.deg2rad(73.5)/2))   # Focal length in y (pixels)
        
        cx = self.image.shape[1] / 2.0            # Principal point x (pixels)
        cy = self.image.shape[0] / 2.0            # Principal point y (pixels)

        self.get_logger().info(f'Intrinsic parameters: fx={fx}, fy={fy}, cx={cx}, cy={cy}')
        
        # Extract x, y, z coordinates
        ## coordinates rotation: x is -y, y is -z, z is x.
        x, y, z = -1*self.points[:, 1], -1*self.points[:, 2], self.points[:, 0]
        distance = np.sqrt(x*x+y*y+z*z)
        
        # Debugging: print some of the raw 3D points
        self.get_logger().info(f'Raw 3D points: {self.points[:5]}')
        
        # Avoid division by zero by setting a minimum value for z
        ## clipping z will just remove half of the points, the correct way is to search for zeros and replace them with epsilon.
        z[z==0] =1e-3
        #z = np.clip(z, 1e-3, None)
        
        # Project 3D points to 2D image space using intrinsic parameters
        u = (fx * x / z) + cx
        v = (fy * y / z) + cy
        
        
        # Clip u and v to be within image bounds
        u = np.clip(u, 0, self.image.shape[1] - 1)
        v = np.clip(v, 0, self.image.shape[0] - 1)
        
        # Debugging: print the projected 2D points
        self.get_logger().info(f'Projected 2D u coordinates: {u[:5]}')
        self.get_logger().info(f'Projected 2D v coordinates: {v[:5]}')

        # Define the radius of the keypoints
        keypoint_radius = 2  # Adjust as needed
        
        # Generate random colors for each keypoint in RGB and convert to BGR
        np.random.seed(0)  # For reproducibility
        colors_rgb = np.random.randint(0, 256, size=(len(u), 3), dtype=np.uint8)
        colors_bgr = colors_rgb[:, ::-1]  # Convert RGB to BGR
        
        # Draw keypoints on the image with random colors
        for i in range(len(u)):
            xi = int(u[i])
            yi = int(v[i])
            color = tuple(map(int, colors_bgr[i]))  # Explicitly convert to tuple of integers

            if 0 <= xi < self.image.shape[1] and 0 <= yi < self.image.shape[0]:
                try:
                    cv2.circle(self.image, (xi, yi), keypoint_radius, color, -1)
                except Exception as e:
                    self.get_logger().error(f'Error drawing keypoint at ({xi}, {yi}) with color {color}: {e}')
        
        # Convert the image back to a ROS Image message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
            self.image_publisher.publish(img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Resize window to 20cm x 20cm
        # Assuming a DPI of 96
        width = int(20 * 96 / 2.54)  # Convert cm to pixels
        height = width  # 20cm x 20cm, so width and height are the same

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()