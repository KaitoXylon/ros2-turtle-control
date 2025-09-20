import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Load calibration data
        calib_file = "/home/xylon/ros2_ws/src/mars_rover_nav/calibration_data.npz"
        calib_data = np.load(calib_file)
        self.camera_matrix = calib_data["camera_matrix"]
        self.dist_coeffs = calib_data["dist_coeffs"]

        # ArUco dictionary & params
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # Marker real-world length (meters)
        self.marker_length = 0.118  

        # Open camera
        self.cap = cv2.VideoCapture("http://192.168.0.103:8080/video")

        if not self.cap.isOpened():
            self.get_logger().error("❌ Cannot open camera.")
            return

        # Run detection at ~30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.detect_callback)
        self.get_logger().info("✅ ArUco detector node started.")

    def detect_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Frame capture failed.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for marker_id, rvec, tvec in zip(ids, rvecs, tvecs):
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
                dist = np.linalg.norm(tvec)
                cv2.putText(
                    frame,
                    f"ID: {marker_id[0]} Dist: {dist:.2f} m",
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                # Log to ROS2 console
                self.get_logger().info(f"Marker {marker_id[0]} distance: {dist:.2f} m")

        cv2.imshow("ArUco Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("🛑 Quitting...")
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
