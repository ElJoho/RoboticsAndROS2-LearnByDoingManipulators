Terminal1
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ . install/setup.bash
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobotjoho@joho-X550DP:~/Documejoho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoijoho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnBjoho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 run arduinobot_firmware simple_serial_transmitter --ros-args -p port:=/dev/ttyUSB0
[INFO] [1761256110.833093678] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256111.833812681] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256112.833876865] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256113.833814286] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256114.833916843] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256115.834025142] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256116.833858041] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256117.834051653] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256118.834237805] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256119.834326534] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256120.834383846] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256121.834354002] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256122.834246292] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256123.834408354] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256124.834123888] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256125.833891617] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256126.833978011] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256127.834580114] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256128.833997881] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256129.833967050] [simple_serial_transmitter]: New message received, publishing on serial port: 1
[INFO] [1761256137.211027572] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761256138.211917376] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761256139.211640939] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761256140.211902214] [simple_serial_transmitter]: New message received, publishing on serial port: 0
[INFO] [1761256141.211871473] [simple_serial_transmitter]: New message received, publishing on serial port: 0


Terminal2
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic list
/parameter_events
/rosout
/serial_transmitter
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic pub /serial_transmitter std_msgs/msg/String "data: '1'"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='1')

publishing #2: std_msgs.msg.String(data='1')

publishing #3: std_msgs.msg.String(data='1')

publishing #4: std_msgs.msg.String(data='1')

publishing #5: std_msgs.msg.String(data='1')

publishing #6: std_msgs.msg.String(data='1')

publishing #7: std_msgs.msg.String(data='1')

publishing #8: std_msgs.msg.String(data='1')

publishing #9: std_msgs.msg.String(data='1')

publishing #10: std_msgs.msg.String(data='1')

publishing #11: std_msgs.msg.String(data='1')

publishing #12: std_msgs.msg.String(data='1')

publishing #13: std_msgs.msg.String(data='1')

publishing #14: std_msgs.msg.String(data='1')

publishing #15: std_msgs.msg.String(data='1')

publishing #16: std_msgs.msg.String(data='1')

publishing #17: std_msgs.msg.String(data='1')

publishing #18: std_msgs.msg.String(data='1')

publishing #19: std_msgs.msg.String(data='1')

publishing #20: std_msgs.msg.String(data='1')

^Cjoho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic pub /serial_transmitter std_msgs/msg/String "data: '0'"
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='0')

publishing #2: std_msgs.msg.String(data='0')

publishing #3: std_msgs.msg.String(data='0')

publishing #4: std_msgs.msg.String(data='0')

publishing #5: std_msgs.msg.String(data='0')

