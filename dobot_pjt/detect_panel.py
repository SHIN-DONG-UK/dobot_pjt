import socket
import threading
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import time

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)

class Config:
    """상수 및 설정"""
    ROI_POSITION = (181, 99, 490, 252)  # ROI 좌표
    SOCKET_PORT = 12345  # 소켓 서버 포트
    SOCKET_HOST = '0.0.0.0'  # 모든 인터페이스에서 접속 허용
    CAMERA_RESOLUTION = (640, 480)  # 카메라 해상도
    CAMERA_FPS = 30  # 카메라 FPS


class DetectPanelNode(Node):
    def __init__(self):
        super().__init__('detect_panel_node')

        self.yolo_model = None
        self.pipeline = None
        self.server_socket = None
        self.bridge = None

        # 다중 클라이언트 관리를 위한 리스트와 락
        self.client_sockets = []
        self.clients_lock = threading.Lock()

        # 상태 관리 변수
        self.object_detected = False  # 이전 오브젝트 감지 여부
        self.last_command = None      # 마지막으로 보낸 명령

        self.detection_start_time = None  # 객체 감지 시작 시간
        self.detection_duration_threshold = 1.0  # 1초 이상 유지될 경우 명령 전송

        self.init_model()
        self.init_camera()
        self.init_socket()
        self.init_ros()

    def init_model(self):
        """YOLOv5 모델 초기화"""
        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/donguk/sdu_ws/src/PJT.pt')
        self.yolo_model.eval()

    def init_camera(self):
        """RealSense 카메라 초기화"""
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, *Config.CAMERA_RESOLUTION, rs.format.bgr8, Config.CAMERA_FPS)
        self.pipeline.start(config)

    def init_socket(self):
        """소켓 서버 초기화"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((Config.SOCKET_HOST, Config.SOCKET_PORT))
        self.server_socket.listen(5)
        self.get_logger().info(f'Socket server listening on port {Config.SOCKET_PORT}')
        self.client_thread = threading.Thread(target=self.accept_clients)
        self.client_thread.daemon = True
        self.client_thread.start()

    def init_ros(self):
        """ROS2 노드 초기화"""
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def accept_clients(self):
        """클라이언트 연결 대기 및 관리"""
        while True:
            try:
                client_socket, addr = self.server_socket.accept()
                with self.clients_lock:
                    self.client_sockets.append(client_socket)
                self.get_logger().info(f'Client connected from {addr}')
            except Exception as e:
                self.get_logger().error(f'Error accepting client: {str(e)}')
                break

    def send_command(self, command):
        """모든 클라이언트로 명령 전송"""
        with self.clients_lock:
            disconnected_clients = []
            for client_socket in self.client_sockets:
                try:
                    client_socket.sendall(command.encode('utf-8') + b'\n')
                except (BrokenPipeError, ConnectionResetError):
                    disconnected_clients.append(client_socket)
                    
            # 연결이 끊긴 클라이언트 제거
            for client_socket in disconnected_clients:
                self.client_sockets.remove(client_socket)
                client_socket.close()
                self.get_logger().info('Client disconnected')

    def get_center_color(self, image):
        """이미지 중심 영역의 평균 색상 반환"""
        height, width = image.shape[:2]
        center_y, center_x = height // 2, width // 2
        sample_size = min(width, height) // 4
        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)
        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size // 2)
        center_region = image[start_y:end_y, start_x:end_x]
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        return np.mean(hsv_region, axis=(0, 1))

    def process_frame(self, color_image):
        """프레임 처리 및 객체 탐지"""
        roi_x1, roi_y1, roi_x2, roi_y2 = Config.ROI_POSITION
        roi_image = color_image[roi_y1:roi_y2, roi_x1:roi_x2]
        results = self.yolo_model(roi_image)

        detection_results = []
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = map(int, result[:6])

            # 객체의 좌표를 원본 이미지 기준으로 변환
            abs_x1 = roi_x1 + x1
            abs_y1 = roi_y1 + y1
            abs_x2 = roi_x1 + x2
            abs_y2 = roi_y1 + y2

            label = self.yolo_model.names[class_id]

            # 결과 저장
            detection_results.append((label, abs_x1, abs_y1, abs_x2, abs_y2, confidence))

        return detection_results

    def annotate_frame(self, color_image, detection_results):
        """탐지 결과를 이미지에 시각화"""
        roi_x1, roi_y1, roi_x2, roi_y2 = Config.ROI_POSITION

        # ROI 영역을 사각형으로 표시
        cv2.rectangle(color_image, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 255), 2)  # 노란색 ROI

        # 탐지된 객체 정보 시각화
        for label, x1, y1, x2, y2, confidence in detection_results:
            # 객체 테두리 및 라벨 표시
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 초록색 테두리
            text = f"{label} {confidence:.2f}"
            cv2.putText(color_image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return color_image

    def handle_detection(self, detection_results):
        """
        탐지 결과를 처리하여 명령 전송
        - 1초 이상 객체가 유지되었을 경우 명령 전송
        """
        if detection_results:  # 감지된 오브젝트가 있을 경우
            if self.detection_start_time is None:
                # 감지가 처음 시작된 시간 기록
                self.detection_start_time = time.time()

            elapsed_time = time.time() - self.detection_start_time
            if elapsed_time >= self.detection_duration_threshold:
                # 객체가 1초 이상 유지된 경우
                if not self.object_detected:
                    self.send_command('1')  # 모터 실행
                    print("Object detected for 1 second: Starting motor")
                    self.object_detected = True

                for label, _, _, _, _, _ in detection_results:
                    if label == 'back_panel' and self.last_command != 'BACK_PANEL':
                        self.send_command('3')  # 서보모터 왼쪽
                        self.last_command = 'BACK_PANEL'
                        print("Back panel detected for 1 second: Moving servo to left")
                    elif label == 'board_panel' and self.last_command != 'BOARD_PANEL':
                        self.send_command('5')  # 서보모터 오른쪽
                        self.last_command = 'BOARD_PANEL'
                        print("Board panel detected for 1 second: Moving servo to right")
        else:  # 감지된 오브젝트가 없을 경우
            self.detection_start_time = None  # 감지 시간 초기화
            if self.object_detected:
                self.send_command('2')  # 모터 정지
                print("No object detected: Stopping motor")
                self.object_detected = False
                self.last_command = None

    def timer_callback(self):
        """주기적으로 호출: 프레임 처리, 시각화 및 명령 전송"""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        detection_results = self.process_frame(color_image)

        # 탐지 결과에 따른 명령 처리
        self.handle_detection(detection_results)

        # 탐지 결과를 이미지에 시각화
        annotated_image = self.annotate_frame(color_image, detection_results)

        # 이미지 퍼블리시
        ros_image_message = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        self.image_publisher.publish(ros_image_message)

    def destroy_node(self):
        """리소스 해제"""
        self.pipeline.stop()
        with self.clients_lock:
            for client_socket in self.client_sockets:
                client_socket.close()
        self.server_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectPanelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == '__main__':
    main()