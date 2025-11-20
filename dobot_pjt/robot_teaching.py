import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from threading import Thread
import time

class RobotPositionSaver(Node):
    def __init__(self):
        super().__init__('robot_position_saver')
        self.tcp_position = None
        self.tcp_subscription = self.create_subscription(PoseStamped, '/dobot_TCP', self.tcp_callback, 10)
        self.file = open('robot_positions.csv', 'a')
        self.file.write("timestamp,tcp_position\n")
        self.running = True

    def tcp_callback(self, msg):
        self.tcp_position = (msg.pose.position.x * 1000, msg.pose.position.y * 1000, msg.pose.position.z * 1000, 0.0)

    def save_current_position(self):
        timestamp = self.get_clock().now().to_msg().sec
        tcp_position_str = str(self.tcp_position) if self.tcp_position else "None"
        self.file.write(f"{timestamp},{tcp_position_str}\n")
        self.get_logger().info("Position saved.")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def user_input_thread(position_saver):
    while position_saver.running:
        user_input = input("Press Enter to save, 'del' to delete, or 'exit': ")
        if user_input.lower() in ["exit"]:
            position_saver.running = False
            break
        elif user_input.lower() in ["del", "delete", "d"]:
            delete_last_line()
        else:
            position_saver.save_current_position()

def delete_last_line():
    file_path = 'robot_positions.csv'
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
        with open(file_path, 'w') as file:
            file.writelines(lines[:-1])
    except FileNotFoundError:
        print("File not found.")

def main(args=None):
    rclpy.init(args=args)
    position_saver = RobotPositionSaver()
    input_thread = Thread(target=user_input_thread, args=(position_saver,), daemon=True)
    input_thread.start()

    try:
        while rclpy.ok() and position_saver.running:
            rclpy.spin_once(position_saver, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        position_saver.running = False
        input_thread.join()
        position_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()