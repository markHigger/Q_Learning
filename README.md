Project can be found at: https://github.com/markHigger/Q_Learning/

This is a ROS Melodic package for task HW2 task 2 submission
For this assignment the A ros robot should follow a wall. In this case it is a righ-wall following robot that has a snake-like path. It must use both Q and SARSA reinforcement learning to determain the policy and exicute that policy
Requirements:
	Gazebo
	HCR Stingray Simulator
	Numpy
    Pickle

<br/>Contents:
	<br/> - scripts/q_trainer.py
        <br/> This is the script that was used to determain the policy using a standard Q-table reinforcement learning. It is recomended to run this for at least 8-10hrs for best results. 
		<br/>-Publishers:
			<br/>/triton_lidar/vel_cmd - velocity and angular velocity control for the robot
		<br/>-Subscribers: 
			<br/>/scan - Has lidar data which is used to determain state
    <br/>scripts/q_follower.py
        <br/>This is the script that runs a predetermained policy for a robot to follow a wall. The policy used is 'Q_q.pickle'
		<br/>-Publishers:
			<br/>/triton_lidar/vel_cmd - velocity and angular velocity control for the robot
		<br/>-Subscribers: 
			<br/>/scan - Has lidar data which is used to determain state
    <br/>scripts/sarsa_trainer.py
       <br/> This is the script that was used to determain the policy using a standard Q-table reinforcement learning. It is recomended to run this for at least 10-12 hrs for best results. 
		<br/>-Publishers:
			<br/>/triton_lidar/vel_cmd - velocity and angular velocity control for the robot
		<br/>-Subscribers: 
			<br/>/scan - Has lidar data which is used to determain state
    <br/>scripts/sarsa_follower.py
        <br/>This is the script that runs a predetermained policy for a robot to follow a wall. The policy used is 'Q_sarsa.pickle'
		<br/>-Publishers:
			<br/>/triton_lidar/vel_cmd - velocity and angular velocity control for the robot
		<br/>-Subscribers: 
			<br/>/scan - Has lidar data which is used to determain state
	<br/>scripts/naive_q_follow.py
		<br/>Part f HW2 T1 - This python script runs a node which runs takes the lidar postion from the stringray node and determains the current stant of the robot and finds the proper action for that state using manually defined Q-values. It then piublishes this action back to the stingray robot node.
		<br/>-Publishers:
			<br/>/triton_lidar/vel_cmd - velocity and angular velocity control for the robot
		<br/>-Subscribers: 
			<br/>/scan - Has lidar data which is used to determain state
	launch/q_follow.launch
		<br/>Contains roslaunch file to run Stingray node, Gazebo, and the q_follower node. 
    launch/sarsa_follow.launch
		<br/>Contains roslaunch file to run Stingray node, Gazebo, and the sarsa_follower node. 

<br/>Compilation
	<br/>cd ~/catkin_ws
	<br/>source ./devel/setup.bash
	<br/>catkin_make
