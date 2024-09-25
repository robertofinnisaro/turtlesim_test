import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.pose_sub_target = self.create_subscription(Pose, '/mytarget/pose', self.update_target_pose, 10)
        self.pose_sub_robot = self.create_subscription(Pose, '/robot/pose', self.update_robot_pose, 10)

        self.target_pose = Pose()
        self.robot_pose = Pose()
        self.distance_tolerance = 0.1
        self.rate = self.create_rate(10)  # 10 Hz

        self.get_logger().info("Waiting for target and robot poses...")

    def update_target_pose(self, data):
        self.target_pose = data

    def update_robot_pose(self, data):
        self.robot_pose = data
        self.move_robot()

    def move_robot(self):
        # Compute the distance to the target
        distance = self.euclidean_distance(self.target_pose, self.robot_pose)

        if distance >= self.distance_tolerance:
            # Compute velocities
            vel_msg = Twist()
            vel_msg.linear.x = float(self.linear_vel(self.target_pose, self.robot_pose))
            vel_msg.angular.z = float(self.angular_vel(self.target_pose, self.robot_pose))
            self.vel_pub.publish(vel_msg)
        else:
            # Stop the robot when the target is reached
            self.get_logger().info("Target reached!")
            self.stop_robot()

    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)
        self.destroy_node()
        rclpy.shutdown()

    def euclidean_distance(self, target_pose, robot_pose):
        return sqrt(pow((target_pose.x - robot_pose.x), 2) + pow((target_pose.y - robot_pose.y), 2))

    def linear_vel(self, target_pose, robot_pose, constant=1.5):
        return constant * self.euclidean_distance(target_pose, robot_pose)

    def steering_angle(self, target_pose, robot_pose):
        return atan2(target_pose.y - robot_pose.y, target_pose.x - robot_pose.x)

    def angular_vel(self, target_pose, robot_pose, constant=6):
        angle_error = self.steering_angle(target_pose, robot_pose) - robot_pose.theta
        angle_error = (angle_error + 3.14) % (2 * 3.14) - 3.14  # Normalize angle error to [-pi, pi]
        return constant * angle_error

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
