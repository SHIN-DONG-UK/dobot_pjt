import json
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import SuctionCupControl, ExecuteHomingProcedure


class Config:
    """
    상수를 관리하는 Config 클래스
    """
    JSON_FILE_PATH = "/home/ssafy/work/Smart_TV_production_line_automation_factory_solution/ROS2+Dobot+D435i+YOLO/src/smart_tv_factory_solution/tasks/dobot_tasks.json"  # JSON 파일 경로
    SUCTION_WAIT_TIME = 3  # 흡착컵 동작 대기 시간
    MOVE_WAIT_TIME = 3  # 이동 후 대기 시간
    PTP_DEFAULT_VELOCITY = 0.9  # 기본 이동 속도 비율
    PTP_DEFAULT_ACCELERATION = 0.5  # 기본 가속도 비율


class DobotController(Node):
    """
    Dobot Magician 로봇 제어 클래스
    - PTP(Point-to-Point) 이동 명령 실행
    - 그리퍼(흡착컵) 제어
    - Homing 서비스 실행
    """
    def __init__(self):
        super().__init__('dobot_controller')
        # PTP Action Client 설정
        self.ptp_action_client = ActionClient(self, PointToPoint, '/PTP_action')
        # 흡착컵(Suction Cup) 서비스 설정
        self.suction_service_client = self.create_client(SuctionCupControl, '/dobot_suction_cup_service')
        # Homing 서비스 설정
        self.homing_service_client = self.create_client(ExecuteHomingProcedure, '/dobot_homing_service')

    def send_ptp_goal(self, motion_type, target_pose):
        """
        PTP(Point-to-Point) 이동 명령을 보냄
        :param motion_type: 이동 유형 (1: PTP Motion)
        :param target_pose: 목표 위치 [x, y, z, r]
        """
        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = motion_type
        goal_msg.target_pose = target_pose
        goal_msg.velocity_ratio = Config.PTP_DEFAULT_VELOCITY
        goal_msg.acceleration_ratio = Config.PTP_DEFAULT_ACCELERATION

        self.get_logger().info(f'Sending PTP goal: {goal_msg}')
        self.ptp_action_client.wait_for_server()
        future = self.ptp_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.ptp_goal_response_callback)

    def ptp_goal_response_callback(self, future):
        """
        PTP Goal에 대한 응답 처리
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('PTP Goal rejected.')
            return
        self.get_logger().info('PTP Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.ptp_result_callback)

    def ptp_result_callback(self, future):
        """
        PTP 명령 실행 결과 처리
        """
        result = future.result().result
        self.get_logger().info(f'PTP Result: {result}')

    def set_suction(self, enable: bool):
        """
        흡착컵 제어 명령을 보냄
        :param enable: 흡착 활성화(True) 또는 비활성화(False)
        """
        req = SuctionCupControl.Request()
        req.enable_suction = enable

        # 서비스 사용 가능 여부 확인
        while not self.suction_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/dobot_suction_cup_service not available, waiting...')

        future = self.suction_service_client.call_async(req)
        future.add_done_callback(self.suction_response_callback)

    def suction_response_callback(self, future):
        """
        흡착컵 제어 명령에 대한 응답 처리
        """
        try:
            response = future.result()
            self.get_logger().info(f'Suction response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def execute_homing(self):
        """
        Homing 서비스 호출
        """
        req = ExecuteHomingProcedure.Request()

        # 서비스 사용 가능 여부 확인
        self.get_logger().info('Waiting for Homing service...')
        while not self.homing_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/dobot_homing_service not available, waiting...')

        self.get_logger().info('Calling Homing service...')
        future = self.homing_service_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # 응답 처리
        if future.result() is not None:
            self.get_logger().info(f'Homing service completed successfully: {future.result()}')
        else:
            self.get_logger().error('Homing service failed.')


def execute_task(dobot_controller, task):
    """
    단일 작업(Task)을 실행
    :param dobot_controller: DobotController 객체
    :param task: 실행할 작업(Task) 데이터
    """
    dobot_controller.get_logger().info(f"Starting Task: {task['name']}")
    for step in task["steps"]:
        # 이동 명령 처리
        if step["event"] == "move":
            dobot_controller.send_ptp_goal(
                motion_type=step["motion_type"],
                target_pose=step["position"]
            )
            time.sleep(Config.MOVE_WAIT_TIME)
        # 흡착컵 제어 처리
        elif step["event"] == "gripper":
            dobot_controller.set_suction(step["state"] == "on")
            time.sleep(Config.SUCTION_WAIT_TIME)


def main():
    """
    Dobot 작업 실행 메인 함수
    """
    # ROS 2 초기화
    rclpy.init()
    dobot_controller = DobotController()

    try:
        # Homing 서비스 실행
        dobot_controller.execute_homing()

        # JSON 파일에서 작업(Task) 데이터 읽기
        with open(Config.JSON_FILE_PATH, "r") as file:
            tasks = json.load(file)["tasks"]

        # 각 작업(Task) 실행
        for task in tasks:
            execute_task(dobot_controller, task)

        # Homing 서비스 실행
        dobot_controller.execute_homing()

    except KeyboardInterrupt:
        # 사용자가 Ctrl+C로 실행 중단 시
        dobot_controller.get_logger().info("Shutting down Dobot controller.")
    except Exception as e:
        # 기타 예외 처리
        dobot_controller.get_logger().error(f"Error: {e}")
    finally:
        # ROS 2 노드 종료 및 정리
        dobot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()