import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        
        calib_file = "/home/xylon/ros2_ws/src/mars_rover_nav/calibration_data.npz"
        calib_data = np.load(calib_file)
        self.camera_matrix = calib_data["camera_matrix"]
        self.dist_coeffs = calib_data["dist_coeffs"]

        
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        
        self.marker_length = 0.118  

    
        self.cap = cv2.VideoCapture("http://192.168.0.104:8080/video")

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera.")
            return

        
        self.timer = self.create_timer(1.0 / 15.0, self.detect_callback)
        self.get_logger().info("ArUco detector node started.")

    def detect_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("no camera.")
            return
    

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            -
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

            y_offset += len(ids) * 30  
            for i in range(len(ids)):
                for j in range(i + 1, len(ids)):
                    dist_between = np.linalg.norm(tvecs[i][0] - tvecs[j][0])
                    text = f"ID {ids[i][0]} <-> ID {ids[j][0]}: {dist_between:.2f} m"
                    cv2.putText(
                        frame,
                        text,
                        (10, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 0, 0),
                        2,
                    )
                    y_offset += 25
                    self.get_logger().info(text)

            cv2.imshow("ArUco Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("Quitting...")
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
