import rclpy
from rclpy.node import Node
import os
import random

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.delete_turtles()
        self.spawn_turtle('mytarget')
        self.spawn_turtle('robot')

    def delete_turtles(self):
        self.get_logger().info("Deleting all turtles")
        os.system("ros2 service call /kill turtlesim/srv/Kill \"{name: 'turtle1'}\"")

    def spawn_turtle(self, name):
        # Generate random coordinates for the turtle
        x = round(random.uniform(0.5, 10.5), 2)
        y = round(random.uniform(0.5, 10.5), 2)
        self.get_logger().info(f"Spawning turtle '{name}' at position ({x}, {y})")
        os.system(f"ros2 service call /spawn turtlesim/srv/Spawn \"{{x: {x}, y: {y}, theta: 0.0, name: '{name}'}}\"")

def main(args=None):
    rclpy.init(args=args)
    spawner = TurtleSpawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
