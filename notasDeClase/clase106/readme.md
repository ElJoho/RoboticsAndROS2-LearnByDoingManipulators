# CLASE 106

TERMINAL1
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 run  arduinobot_firmware simple_serial_transmitter --ros-args -p port:=/dev/ttyUSB0
[INFO] [1761333924.921933336] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761333925.922590471] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761333926.922623786] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761333927.922751470] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761333928.922676762] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761333929.922593265] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761333930.924021451] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761333931.923155254] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761333939.928584439] [simple_serial_transmitter]: New message received, publishing on serial port: 25
[INFO] [1761333940.929495977] [simple_serial_transmitter]: New message received, publishing on serial port: 25
[INFO] [1761333950.075651020] [simple_serial_transmitter]: New message received, publishing on serial port: 90
[INFO] [1761333951.076209689] [simple_serial_transmitter]: New message received, publishing on serial port: 90
[INFO] [1761333968.873895012] [simple_serial_transmitter]: New message received, publishing on serial port: 180
[INFO] [1761333969.874786340] [simple_serial_transmitter]: New message received, publishing on serial port: 180
[INFO] [1761333970.874856331] [simple_serial_transmitter]: New message received, publishing on serial port: 180
[INFO] [1761333971.874793842] [simple_serial_transmitter]: New message received, publishing on serial port: 180
[INFO] [1761333972.874726475] [simple_serial_transmitter]: New message received, publishing on serial port: 180
[INFO] [1761333973.874705564] [simple_serial_transmitter]: New message received, publishing on serial port: 180
[INFO] [1761333974.874879245] [simple_serial_transmitter]: New message received, publishing on serial port: 180



TERMINAL2
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic list
/parameter_events
/rosout
/serial_transmitter
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic pub /serial_transmitter std_msgs/msg/String "data: '0'"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='0')

publishing #2: std_msgs.msg.String(data='0')

publishing #3: std_msgs.msg.String(data='0')

publishing #4: std_msgs.msg.String(data='0')

publishing #5: std_msgs.msg.String(data='0')

publishing #6: std_msgs.msg.String(data='0')

publishing #7: std_msgs.msg.String(data='0')

publishing #8: std_msgs.msg.String(data='0')

^Cjoho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic pub /serial_transmitter std_msgs/msg/String "data: '25'"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='25')

publishing #2: std_msgs.msg.String(data='25')

^Cjoho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic pub /serial_transmitter std_msgs/msg/String "data: '90'"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='90')

publishing #2: std_msgs.msg.String(data='90')

^Cjoho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic pub /serial_transmitter std_msgs/msg/String "data: '180'"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='180')

publishing #2: std_msgs.msg.String(data='180')

publishing #3: std_msgs.msg.String(data='180')

publishing #4: std_msgs.msg.String(data='180')

publishing #5: std_msgs.msg.String(data='180')

^Cpublishing #6: std_msgs.msg.String(data='180')

publishing #7: std_msgs.msg.String(data='180')

^Cjoho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ 
