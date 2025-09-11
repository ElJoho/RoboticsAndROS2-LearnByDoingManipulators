# Clase 70 Euler to quaternion service

terminal1
ros2 pkg create --build-type ament_cmake arduinobot_utils
colcon build
. install/setup.bash
colon build

sudo apt-get install ros-humble-tf-transformations
sudo pip3 install transforms3d

Terminal1


joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ . install/setup.bash 
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 run arduinobot_utils angle_conversion.py 
[INFO] [1757549861.356830500] [angle_conversion_service_server]: Angle Conversion Services are ready
[INFO] [1757550041.944106167] [angle_conversion_service_server]: Requested to convert euler angles roll: -0.5, pitch: 0.0, yaw: 1.5 into a quaternion.
[INFO] [1757550041.945269233] [angle_conversion_service_server]: Corresponding quaternion x: -0.18102272310184675, y: -0.1686401280111165, z: 0.6604482617060496, w: 0.7089424338792562
[INFO] [1757550209.930997568] [angle_conversion_service_server]: Requested to convert quaternion x: 0.0, y: 0.0, z: 0.0, w: 1.0
[INFO] [1757550209.933441524] [angle_conversion_service_server]: Corresponding euler angles roll: 0.0, pitch: -0.0, yaw: 0.0
^CTraceback (most recent call last):

Terminal2


joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ . install/setup.bash
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 service list
/angles_converter/describe_parameters
/angles_converter/get_parameter_types
/angles_converter/get_parameters
/angles_converter/list_parameters
/angles_converter/set_parameters
/angles_converter/set_parameters_atomically
/euler_to_quaternion
/quaternion_to_euler
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 service call /euler_to_quaternion 
arduinobot_msgs/srv/EulerToQuaternion  -r                                     --rate
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 service call /euler_to_quaternion arduinobot_msgs/srv/EulerToQuaternion 
-r                                     --rate                                 roll:\ 0.0\^Jpitch:\ 0.0\^Jyaw:\ 0.0\
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 service call /euler_to_quaternion arduinobot_msgs/srv/EulerToQuaternion "roll: -0.5
pitch: 0.0
yaw: 1.5"
requester: making request: arduinobot_msgs.srv.EulerToQuaternion_Request(roll=-0.5, pitch=0.0, yaw=1.5)

response:
arduinobot_msgs.srv.EulerToQuaternion_Response(x=-0.18102272310184675, y=-0.1686401280111165, z=0.6604482617060496, w=0.7089424338792562)

joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 service call /quaternion_to_euler arduinobot_msgs/srv/QuaternionToEuler 
-r                                      --rate                                  x:\ 0.0\^Jy:\ 0.0\^Jz:\ 0.0\^Jw:\ 0.0\
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 service call /quaternion_to_euler arduinobot_msgs/srv/QuaternionToEuler "x: 0.0
y: 0.0
z: 0.0
w: 1.0" 
requester: making request: arduinobot_msgs.srv.QuaternionToEuler_Request(x=0.0, y=0.0, z=0.0, w=1.0)

response:
arduinobot_msgs.srv.QuaternionToEuler_Response(roll=0.0, pitch=-0.0, yaw=0.0)

joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ 

