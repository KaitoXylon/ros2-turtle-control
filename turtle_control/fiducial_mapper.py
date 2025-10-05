#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray


class FiducialMapper(Node):
    def __init__(self):
        super().__init__('fiducial_mapper')
        
        # Load calibration data
        calib_file = "/home/xylon/ros2_ws/src/mars_rover_nav/calibration_data.npz"
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

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Marker visualization publisher
        self.marker_pub = self.create_publisher(MarkerArray, 'fiducial_markers', 10)

        # Timer for detection and TF publishing
        self.timer = self.create_timer(1.0 / 15.0, self.detect_and_publish)
        self.get_logger().info("Fiducial Mapper node started.")

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame received from camera.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        marker_array = MarkerArray()

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i in range(len(ids)):
                # FIX: Convert numpy int to Python int
                marker_id = int(ids[i][0])
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]

                # Convert rotation vector to rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                
                # Convert rotation matrix to quaternion
                quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

                # Publish TF for each detected marker
                self.publish_marker_tf(marker_id, tvec, quaternion)

                # Create visualization marker
                marker = self.create_marker_msg(marker_id, tvec, quaternion)
                marker_array.markers.append(marker)

                # Log detection
                self.get_logger().info(f"Marker {marker_id} at: ({tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f})")

            # Publish marker array for visualization
            self.marker_pub.publish(marker_array)

            # Display for debugging
            aruco.drawDetectedMarkers(frame, corners, ids)
            for rvec, tvec in zip(rvecs, tvecs):
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

        cv2.imshow("Fiducial Mapping", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Shutting down...")
            self.destroy_node()

    def rotation_matrix_to_quaternion(self, R):
        # Convert rotation matrix to quaternion
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = math.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

        return [qx, qy, qz, qw]

    def publish_marker_tf(self, marker_id, translation, quaternion):
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = f'marker_{marker_id}'
        
        # Set translation (convert from camera coordinates to ROS coordinates)
        t.transform.translation.x = translation[2]
        t.transform.translation.y = -translation[0]
        t.transform.translation.z = -translation[1]
        
        # Set rotation
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def create_marker_msg(self, marker_id, translation, quaternion):
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fiducial_markers"
        
        # FIX: Ensure marker_id is a regular Python int
        marker.id = int(marker_id)
        
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = translation[2]
        marker.pose.position.y = -translation[0]
        marker.pose.position.z = -translation[1]
        
        # Set orientation
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        
        # Set scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        
        # Set color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Set lifetime
        marker.lifetime.sec = 1
        
        return marker

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FiducialMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()