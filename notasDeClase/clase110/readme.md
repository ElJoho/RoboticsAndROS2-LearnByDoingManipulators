TERMINAL1
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ . install/setup.bash 
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 run arduinobot_cpp_examples simple_lifecycle_node 
[INFO] [1761454149.777635197] [simple_lifecycle_node]: Lifecycle node on_configure() called.
[INFO] [1761454344.967216649] [simple_lifecycle_node]: Lifecycle node on_activate() called.
[INFO] [1761454346.967802512] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454346.968045984] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454347.086206425] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454348.086114594] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454349.086216857] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454350.086259964] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454351.086215070] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454352.086197554] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454396.751964515] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454397.753543770] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454398.753902246] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454399.753069277] [simple_lifecycle_node]: Lifecycle node heard: Hi
[INFO] [1761454400.226456626] [simple_lifecycle_node]: Lifecycle node on_deactivate() called.

TERMINAL2
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 lifecycle nodes
/simple_lifecycle_node
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 lifecycle get /simple_lifecycle_node
unconfigured [1]
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic list
/parameter_events
/rosout
/simple_lifecycle_node/transition_event
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 lifecycle list /simple_lifecycle_node
- configure [1]
	Start: unconfigured
	Goal: configuring
- shutdown [5]
	Start: unconfigured
	Goal: shuttingdown
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 lifecycle set /simple_lifecycle_node configure
Transitioning successful
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 lifecycle get /simple_lifecycle_node 
inactive [2]
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 lifecycle list /simple_lifecycle_node 
- cleanup [2]
	Start: inactive
	Goal: cleaningup
- activate [3]
	Start: inactive
	Goal: activating
- shutdown [6]
	Start: inactive
	Goal: shuttingdown
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 lifecycle set /simple_lifecycle_node activate
Transitioning successful
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 lifecycle list /simple_lifecycle_node
- deactivate [4]
	Start: active
	Goal: deactivating
- shutdown [7]
	Start: active
	Goal: shuttingdown
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 lifecycle set /simple_lifecycle_node deactivate
Transitioning successful
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManip

TERMINAL3
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic list
/chatter
/parameter_events
/rosout
/simple_lifecycle_node/transition_event
joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ ros2 topic pub /chatter std_msgs/msg/String "data: 'Hi'" 
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Hi')

publishing #2: std_msgs.msg.String(data='Hi')

publishing #3: std_msgs.msg.String(data='Hi')

publishing #4: std_msgs.msg.String(data='Hi')

publishing #5: std_msgs.msg.String(data='Hi')

publishing #6: std_msgs.msg.String(data='Hi')

publishing #7: std_msgs.msg.String(data='Hi')

publishing #8: std_msgs.msg.String(data='Hi')

publishing #9: std_msgs.msg.String(data='Hi')

publishing #10: std_msgs.msg.String(data='Hi')

publishing #11: std_msgs.msg.String(data='Hi')

publishing #12: std_msgs.msg.String(data='Hi')

joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ $ ros2 topipub /chatter std_msgs/msg/String "data: 'Hi'" 
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='Hi')

publishing #2: std_msgs.msg.String(data='Hi')

publishing #3: std_msgs.msg.String(data='Hi')

publishing #4: std_msgs.msg.String(data='Hi')

publishing #5: std_msgs.msg.String(data='Hi')

publishing #6: std_msgs.msg.String(data='Hi')

publishing #7: std_msgs.msg.String(data='Hi')

publishing #8: std_msgs.msg.String(data='Hi')

joho@joho-X550DP:~/Documents/RoboticsAndROS2-LearnByDoingManipulators/arduinobot_ws$ 
