import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

     return LaunchDescription([
     Node(package='turtlesim',
         executable='turtlesim_node',
         output='screen'),
     Node(package='turtlesim_test', 
         executable='spawn',
         output='screen'),
     Node(package='turtlesim_test', 
         executable='move',
         output='screen'),    
    ])
