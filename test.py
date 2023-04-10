import subprocess
from time import sleep
import time
from subprocess import call


# go forward
motor_forward = subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: 1}'",
    shell=True), subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 '{data: 1}'",
    shell=True)

sleep(10)

# stop moving
motor_stop = subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: 0}'",
    shell=True), subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 '{data: 0}'",
    shell=True)

sleep(3)

# # go backwards
motor_backwards = subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: -1}'",
    shell=True), subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 '{data: -1}'",
    shell=True)

sleep(10)

# go right
motor_right = subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 '{data: 1}'",
    shell=True), subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: 0}'",
    shell=True)

sleep(10)


# stop moving
motor_stop = subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: 0}'",
    shell=True), subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 '{data: 0}'",
    shell=True)

sleep(10)

# go forward
motor_forward = subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: 1}'",
    shell=True), subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 '{data: 1}'",
    shell=True)

sleep(10)

# go right
motor_right = subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 '{data: 1}'",
    shell=True), subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: 0}'",
    shell=True)

sleep(1)

# go left
motor_left = subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 '{data: 0}'",
    shell=True), subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: 1}'",
    shell=True)

sleep(1)

# # go backwards
motor_backwards = subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: -1}'",
    shell=True), subprocess.Popen(
    "ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 '{data: -1}'",
    shell=True)

sleep(10)
