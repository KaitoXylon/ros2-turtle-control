import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion, Vector3
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import ColorRGBA

class WorldMapper(Node):
    def __init__(self):
        super().__init__('world_mapper')
        
        # Camera calibration
        calib_file = "/home/ashra/ros2_ws/src/turtle_controls/turtle_control/calibration_data.npz"
        calib_data = np.load(calib_file)
        self.camera_matrix = calib_data["camera_matrix"]
        self.dist_coeffs = calib_data["dist_coeffs"]

        # ArUco setup
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        self.marker_length = 0.118  # meters

        # Camera setup
        self.cap = cv2.VideoCapture("http://192.168.0.100:8080/video")
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")
            return

        # TF setup
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'world_markers', 10)

        # Timer
        self.timer = self.create_timer(1.0 / 15.0, self.detect_and_map)
        
        # World state
        self.world_markers = {}
        self.marker_observations = {}
        
        # Mode: 0=detection only, 1=with IMU tracking
        self.mode = 0  # Start with detection only
        
        self.get_logger().info(" World Mapper started - Detection Mode")
        self.get_logger().info("Run 'ros2 run your_package imu_simulator' to enable full tracking")

    def detect_and_map(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("no camera.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            # YOUR VISUALIZATION CODE
            y_offset = 40  
            for idx, (marker_id, rvec, tvec) in enumerate(zip(ids, rvecs, tvecs)):
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
                dist = np.linalg.norm(tvec)
                angle = np.arctan2(tvec[0][0], tvec[0][2])
                bearing_deg = np.degrees(angle)

                text = f"ID: {marker_id[0]} | Dist: {dist:.2f} m | Bearing: {bearing_deg:.1f} deg"
                cv2.putText(
                    frame,
                    text,
                    (10, y_offset + idx * 30), 
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                self.get_logger().info(text)

            # Process markers for world mapping
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                tvec = tvecs[i][0]  
                rvec = rvecs[i][0]  

                # Get CORRECT marker pose in camera frame
                marker_pose_camera = self.get_correct_marker_pose(tvec, rvec)
                
                # Update world map based on mode
                if self.mode == 1 and self.check_imu_available():
                    # Full tracking with IMU
                    self.update_world_map_with_imu(marker_id, marker_pose_camera)
                else:
                    # Detection-only mode (relative to camera)
                    self.update_world_map_detection_only(marker_id, marker_pose_camera)

        # Display mode info on frame
        mode_text = "MODE: IMU TRACKING" if self.mode == 1 else "MODE: DETECTION ONLY"
        cv2.putText(frame, mode_text, (10, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow("ArUco Detection - World Mapping", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Quitting...")
            self.destroy_node()
        elif key == ord('m'):
            self.toggle_mode()
            
        # Publish visualization
        self.publish_world_visualization()

    def get_correct_marker_pose(self, tvec, rvec):
        """
        CORRECTED: Get marker pose in proper camera coordinates
        OpenCV returns: X right, Y down, Z forward from camera
        """
        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # CORRECT TRANSFORMATION:
        # tvec is in camera frame: [X, Y, Z] where:
        # X: right from camera, Y: down from camera, Z: forward from camera
        # We want: X right, Y down, Z forward (same as camera frame)
        
        pose = {
            'position': [
                tvec[0],   # X: right from camera (CORRECT)
                tvec[1],   # Y: down from camera (CORRECT) 
                tvec[2]    # Z: forward from camera (CORRECT)
            ],
            'orientation': self.rotation_matrix_to_quaternion(rotation_matrix)
        }
        
        return pose

    def update_world_map_detection_only(self, marker_id, marker_pose_camera):
        """
        Detection-only mode: Place markers relative to camera at origin
        This shows correct distances but in camera-relative space
        """
        # For detection-only mode, we assume camera is at world origin
        # This demonstrates the detection works with correct distances
        
        camera_pose_world = {
            'position': [0.0, 0.0, 0.0],  # Camera at origin
            'orientation': [0.0, 0.0, 0.0, 1.0]  # No rotation
        }
        
        # Transform marker to world coordinates (camera at origin)
        marker_pose_world = self.transform_pose_to_world(marker_pose_camera, camera_pose_world)
        
        if marker_id not in self.world_markers:
            self.world_markers[marker_id] = marker_pose_world
            self.marker_observations[marker_id] = 1
            dist = np.linalg.norm(marker_pose_camera['position'])
            self.get_logger().info(f"Marker {marker_id} at {dist:.2f}m from camera")
        else:
            # Smooth update
            old_pose = self.world_markers[marker_id]
            obs_count = self.marker_observations[marker_id]
            
            new_pos = [
                (old_pose['position'][0] * obs_count + marker_pose_world['position'][0]) / (obs_count + 1),
                (old_pose['position'][1] * obs_count + marker_pose_world['position'][1]) / (obs_count + 1),
                (old_pose['position'][2] * obs_count + marker_pose_world['position'][2]) / (obs_count + 1)
            ]
            
            self.world_markers[marker_id] = {
                'position': new_pos,
                'orientation': marker_pose_world['orientation']
            }
            self.marker_observations[marker_id] = obs_count + 1

    def update_world_map_with_imu(self, marker_id, marker_pose_camera):
        """Full tracking mode with IMU camera pose"""
        try:
            camera_pose_world = self.get_camera_pose_in_world()
            
            if camera_pose_world is None:
                self.get_logger().warn("IMU not available - switch to detection mode")
                self.mode = 0
                return
            
            marker_pose_world = self.transform_pose_to_world(marker_pose_camera, camera_pose_world)
            
            if marker_id not in self.world_markers:
                self.world_markers[marker_id] = marker_pose_world
                self.marker_observations[marker_id] = 1
                world_dist = np.linalg.norm(marker_pose_world['position'])
                self.get_logger().info(f" Marker {marker_id} at world position: {world_dist:.2f}m from origin")
            else:
                old_pose = self.world_markers[marker_id]
                obs_count = self.marker_observations[marker_id]
                
                new_pos = [
                    (old_pose['position'][0] * obs_count + marker_pose_world['position'][0]) / (obs_count + 1),
                    (old_pose['position'][1] * obs_count + marker_pose_world['position'][1]) / (obs_count + 1),
                    (old_pose['position'][2] * obs_count + marker_pose_world['position'][2]) / (obs_count + 1)
                ]
                
                self.world_markers[marker_id] = {
                    'position': new_pos,
                    'orientation': marker_pose_world['orientation']
                }
                self.marker_observations[marker_id] = obs_count + 1
                
        except Exception as e:
            self.get_logger().warn(f"Failed IMU update for marker {marker_id}: {str(e)}")

    def transform_pose_to_world(self, pose_camera, camera_pose_world):
        """CORRECTED: Transform pose from camera frame to world frame"""
        # Extract positions and orientations
        pos_cam = np.array(pose_camera['position'])  # Marker in camera frame
        orient_cam = np.array(pose_camera['orientation'])
        pos_cam_world = np.array(camera_pose_world['position'])  # Camera in world
        orient_cam_world = np.array(camera_pose_world['orientation'])
        
        # Convert quaternions to rotation matrices
        R_cam_world = R.from_quat([
            orient_cam_world[0], orient_cam_world[1], 
            orient_cam_world[2], orient_cam_world[3]
        ]).as_matrix()
        
        R_marker_cam = R.from_quat([
            orient_cam[0], orient_cam[1], orient_cam[2], orient_cam[3]
        ]).as_matrix()
        
        # CORRECT transformation:
        # World position = Camera world position + Camera rotation * Marker camera position
        pos_world = pos_cam_world + R_cam_world @ pos_cam
        
        # World orientation = Camera world orientation * Marker camera orientation
        R_world = R_cam_world @ R_marker_cam
        orient_world = R.from_matrix(R_world).as_quat()  # [x, y, z, w]
        
        return {
            'position': pos_world.tolist(),
            'orientation': [orient_world[0], orient_world[1], orient_world[2], orient_world[3]]
        }

    def get_camera_pose_in_world(self):
        """Get camera pose from IMU simulator"""
        try:
            transform = self.tf_buffer.lookup_transform('world', 'camera_link', rclpy.time.Time())
            
            return {
                'position': [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ],
                'orientation': [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ]
            }
        except Exception as e:
            return None

    def check_imu_available(self):
        """Check if IMU transform is available"""
        try:
            self.tf_buffer.lookup_transform('world', 'camera_link', rclpy.time.Time())
            if self.mode == 0:
                self.mode = 1
                self.get_logger().info("🔄 Switching to IMU TRACKING mode")
            return True
        except:
            if self.mode == 1:
                self.mode = 0
                self.get_logger().info("🔄 Switching to DETECTION ONLY mode")
            return False

    def toggle_mode(self):
        """Toggle between detection only and IMU tracking"""
        if self.mode == 0 and self.check_imu_available():
            self.mode = 1
            self.get_logger().info("🔄 Manually switched to IMU TRACKING mode")
        else:
            self.mode = 0
            self.get_logger().info("🔄 Manually switched to DETECTION ONLY mode")

    def publish_world_visualization(self):
        """Publish markers in world frame"""
        marker_array = MarkerArray()
        
        for marker_id, marker_pose in self.world_markers.items():
            # Marker visualization
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "world_markers"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position = Point(
                x=marker_pose['position'][0],
                y=marker_pose['position'][1],
                z=marker_pose['position'][2]
            )
            marker.pose.orientation = Quaternion(
                x=marker_pose['orientation'][0],
                y=marker_pose['orientation'][1],
                z=marker_pose['orientation'][2],
                w=marker_pose['orientation'][3]
            )
            
            marker.scale = Vector3(x=0.1, y=0.1, z=0.01)
            
            # Color based on distance for verification
            distance = np.linalg.norm(marker_pose['position'])
            if distance < 1.0:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green (close)
            elif distance < 2.0:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)  # Yellow (medium)
            else:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red (far)
            
            marker_array.markers.append(marker)
            
            # Text label with distance
            text_marker = Marker()
            text_marker.header.frame_id = "world"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "marker_labels"
            text_marker.id = marker_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position = Point(
                x=marker_pose['position'][0],
                y=marker_pose['position'][1],
                z=marker_pose['position'][2] + 0.15
            )
            text_marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            text_marker.scale = Vector3(x=0.0, y=0.0, z=0.05)
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"ID: {marker_id}\nDist: {distance:.2f}m"
            marker_array.markers.append(text_marker)
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert rotation matrix to quaternion"""
        return R.from_matrix(rotation_matrix).as_quat()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WorldMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()