import os
# import subprocess
from time import sleep

# sensor = "ros2 topic echo /hadabot/wheel_radps_left"

# output = subprocess.check_output(
#     "ros2 topic echo /hadabot/wheel_radps_left", shell=True)

# print(output)

os.system(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: 1}'")
sleep(3)
os.system(
    "ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 '{data: 0}'")
