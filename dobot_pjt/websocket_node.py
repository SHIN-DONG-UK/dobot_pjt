import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped  # 좌표 정보를 위한 메시지 타입
import websocket
import json
import threading

SERVER_URL = '192.168.26.42:8000'  # 서버 주소

class WebSocketClientNode(Node):
    def __init__(self):
        super().__init__('websocket_node')
        self.ws = None
        self.ws_thread = None
        self.detection_result = None
        self.joint_states = None
        self.tcp_coordinates = None

        # WebSocket 연결 시작
        self.start_websocket_connection()

        # /detected_results, /dobot_joint_states, /dobot_TCP 토픽 구독
        self.create_subscription(String, '/detected_result', self.detected_results_callback, 10)
        self.create_subscription(JointState, '/dobot_joint_states', self.dobot_joint_states_callback, 10)
        self.create_subscription(PoseStamped, '/dobot_TCP', self.dobot_tcp_callback, 10)

    def start_websocket_connection(self):
        """WebSocket 연결을 스레드로 시작"""
        self.ws_thread = threading.Thread(target=self.connect)
        self.ws_thread.start()

    def connect(self):
        """WebSocket 서버와 연결"""
        uri = f"ws://{SERVER_URL}/socket/ws/robodk/"
        self.ws = websocket.WebSocket()
        try:
            self.ws.connect(uri)
            self.get_logger().info("WebSocket connection established!")

            # 수신 메시지 처리
            while True:
                message = self.ws.recv()
                self.handle_message(message)
        except Exception as e:
            self.get_logger().error(f"WebSocket Error: {e}")
        finally:
            self.ws.close()
            self.get_logger().info("WebSocket connection closed.")

    def handle_message(self, message):
        """수신된 메시지 처리"""
        try:
            data = json.loads(message)
            self.get_logger().info(f"Received data: {data}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode message: {e}")

    def send_message(self, message):
        """WebSocket을 통해 메시지 전송"""
        if self.ws:
            try:
                self.ws.send(json.dumps(message))
                self.get_logger().info(f"Sent message: {message}")
            except Exception as e:
                self.get_logger().error(f"Failed to send message: {e}")

    def detected_results_callback(self, msg):
        """/detected_results 토픽에서 String 메시지를 수신하고 데이터 저장"""
        self.detection_result = msg.data
        self.compose_and_send_message()

    def dobot_joint_states_callback(self, msg):
        """/dobot_joint_states 토픽에서 JointState 메시지를 수신하고 데이터 저장"""
        self.joint_states = {
            'name': list(msg.name),  # 배열을 리스트로 변환
            'position': list(msg.position),  # 배열을 리스트로 변환
            'velocity': list(msg.velocity),  # 배열을 리스트로 변환
            'effort': list(msg.effort)  # 배열을 리스트로 변환
        }
        self.compose_and_send_message()

    def dobot_tcp_callback(self, msg):
        """/dobot_TCP 토픽에서 PoseStamped 메시지를 수신하고 데이터 저장"""
        self.tcp_coordinates = {
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }
        self.compose_and_send_message()

    def compose_and_send_message(self):
        """모든 데이터를 합쳐 WebSocket으로 전송"""
        if self.detection_result is not None and self.joint_states is not None and  self.tcp_coordinates is not None:
            message = {
                'type': 'sensor_data',
                'detection_result': self.detection_result,
                'joint_states': self.joint_states,
                'tcp_coordinates': self.tcp_coordinates
            }
            self.send_message(message)
            self.detection_result = None
            self.joint_states = None
            self.tcp_coordinates = None

def main(args=None):
    rclpy.init(args=args)
    websocket_node = WebSocketClientNode()

    try:
        rclpy.spin(websocket_node)
    except KeyboardInterrupt:
        websocket_node.get_logger().info('Shutting down WebSocket client node')
    finally:
        if websocket_node.ws:
            websocket_node.ws.close()
        websocket_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()