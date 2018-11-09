1) INSTALL PCAN PROPERLY
	https://github.com/felixduvallet/allegro-hand-ros
	Installing the PCAN driver
	Before using the hand, you must install the pcan drivers. This assumes you have a peak-systems pcan to usb adapter.
    	- Install these packages in the "home" directory. Change "INDIGO"  for "KINETIC"
   	$ sudo apt-get install libpopt-dev ros-indigo-libpcan

    	- Download latest drivers in the home directory: http://www.peak-system.com/fileadmin/media/linux/index.htm#download
        - Install the drivers from peak folder.
        $ cd ~/peak-linux-driver-8.6.0
	$ make clean; make NET=NO_NETDEV_SUPPORT
	$ sudo make install
	$ sudo /sbin/modprobe pcan

	- Test that the interface is installed properly with:
	$ cat /proc/pcan
          You should see some stuff streaming.
	
	- When the hand is connected, you should see pcanusb0 or pcanusb1 in the list of available interfaces:
	$ ls -l /dev/pcan*
	- If you do not see any available files, you may need to run:
 	- sudo ./driver/pcan_make_devices 2
	from the downloaded pcan folder: this theoretically creates the devices files if the system has not done it automatically.

2) MAKE FULL INSTALLATION
	- Follow instructions to create an original allegro_hand_ros_catkin as in the website
		https://github.com/simlabrobotics/allegro_hand_ros_catkin
		Can install original or the one from https://github.com/felixduvallet/allegro-hand-ros
			1. Download the package:    
		            git clone https://github.com/simlabrobotics/allegro_hand_ros_catkin.git    
			2. Build the package          
				$ cd allegro_hand_ros_catkin
				$ catkin_make
	- Change the "zero.yaml" in /allegro_hand_ros_catkin/src/allegro_hand_parameters according to the specific hand encoders offset
	- Change /allegro_hand_ros_catkin/src/allegro_hand_driver/src/AllegroHandDrv.cpp from previous limits to allow more power at the thumb and other joints. Set to:
		#define PWM_LIMIT_ROLL 250.0*1.5
		#define PWM_LIMIT_NEAR 450.0*1.5
		#define PWM_LIMIT_MIDDLE 300.0*1.5
		#define PWM_LIMIT_FAR 400*1.5 // 190.0*1.5

		#define PWM_LIMIT_THUMB_ROLL 400*1.5// 350.0*1.5
		#define PWM_LIMIT_THUMB_NEAR 400*1.5 // 350.0*1.5 // joint 2
		#define PWM_LIMIT_THUMB_MIDDLE 400*1.5 //180.0*1.5 // joint 1 
		#define PWM_LIMIT_THUMB_FAR 400.0*1.5 //180.0*1.5 // joint 0

		#define PWM_LIMIT_GLOBAL_8V 800.0 // maximum: 1200
		#define PWM_LIMIT_GLOBAL_24V 500.0

	- Download "allegro_hrii" and copy it in allegro_hand_ros_catkin/src
		https://github.com/Virginia5632/allegro_hrii
	- Do >>git clone https://bitbucket.org/robot-learning/ll4ma_kdl.git to get the library for the robot from Balakumar Sundaralingam work into the main src file of "/allegro_hand_ros_catkin"

2) GO TO RIGHT DIRECTORY AND SOURCE PROPERLY
	$ cd ~/allegro_hand_ros_catkin and do 
	$ gedit ~/.bashrc
	Uncomment the line source /home/virginia/allegro_hand_ros_catkin/devel/setup.bash
	$ source /home/virginia/allegro_hand_ros_catkin/devel/setup.bash
	$ catkin_make

3) RUN BASIC CODE 
	$ roslaunch allegro_hand_controllers allegro_hand.launch HAND:=right CONTROLLER:=grasp
	Try different grasps to check it works  
     
4) TRY NEW NODES
	- terminal 1:
		>> roslaunch allegro_hand_controllers allegro_hand.launch HAND:=right CONTROLLER:=impedance
	- terminal 2 (Home position with high stiffness):
		>> rostopic pub /allegroHand_0/impedance_cmd allegro_hand_controllers/StiffControl "header:
		seq: 0
		stamp: {secs: 0, nsecs: 0}
		frame_id: ''
		name: ['joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0','joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0','joint_8.0', 'joint_9.0', 'joint_10.0', 			'joint_11.0','joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0']
		DesPos: [0,   -0.1745,    0.7854,    0.7854 , 0,-0.1745,0.7854,    0.7854,    0.0873,   -0.0873,0.8727, 0.7854,    0.8727 ,   0.4363,    0.2618,    0.7854]
		DesKp: [600.0, 600.0, 600.0, 1000.0,600.0, 600.0, 600.0, 1000.0,600.0, 600.0, 600.0, 1000.0,1000.0, 1000.0, 1000.0, 600.0]
		DesKd: [15.0, 20.0, 15.0, 15.0, 15.0, 20.0, 15.0, 15.0,15.0, 20.0, 15.0, 15.0,30.0, 20.0, 20.0, 15.0]"

		(Grasp position with low stiffness in index):
		>> rostopic pub /allegroHand_0/impedance_cmd allegro_hand_controllers/StiffControl "header:
		seq: 0
		stamp: {secs: 0, nsecs: 0}
		frame_id: ''
		name: ['joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0','joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0','joint_8.0', 'joint_9.0', 'joint_10.0', 			'joint_11.0','joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0']
		DesPos: [0,   -0.1745,    0.7854,    0.7854 , 0,-0.1745,0.7854,    0.7854,    0.0873,   -0.0873,0.8727, 0.7854,    0.8727 ,   0.4363,    0.2618,    0.7854]
		DesKp: [600.0, 600.0, 600.0, 1000.0,600.0, 600.0, 600.0, 1000.0,600.0, 600.0, 600.0, 1000.0,1000.0, 1000.0, 1000.0, 600.0]
		DesKd: [15.0, 20.0, 15.0, 15.0, 15.0, 20.0, 15.0, 15.0,15.0, 20.0, 15.0, 15.0,30.0, 20.0, 20.0, 15.0]"


////////////////////////////////////////////
/////////////////// EXTRAS /////////////////
////////////////////////////////////////////

- TO GIT

	$ cd src/allegro_hrii/ --> Go to package folder
	$ git add -A --> Add everything inside the folder to commit later
	$ git commit -m "first commit"   --> commit
	$ git remote add origin https://github.com/Virginia5632/allegro_hrii.git
	$ git push -u origin master  --> push to the web

	$ git gui -->  wizard to commit and push easier (FIRST go to "allegro_hrii" folder)

	$ git stash --> To go back to website version
	$ git stash pop --> To undo stash

- NEW NODE 
	Inside ~/allegro_hrii src and include (files: allegro_node_impedance.cpp, allegro_node_impedance.h)
	The allegro node either does pd control as it was done in the allegro_node_pd, or it uses the kp, kd and q_des published inside the topic /allegroHand/impedance_cmd
- NEW MESSAGE
	StiffControl following the guide here: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv


- USE IMPEDANCE CONTROLLER IN MATLAB
	1. Create new message for impedance
		- Create folder called "allegro_hrii"
		- Inside create file "package.xml" with the following code
			<package>
			<name>allegro_hrii</name>
			<version>1.1.01</version>
		
			<build_depend>std_msgs</build_depend>
			<run_depend>std_msgs</run_depend>
	
			<build_depend>message_generation</build_depend>
		 	<exec_depend>message_runtime</exec_depend>
			</package>
		- Inside "allegro_hrii" create subflored called "msg"
		- Inside "msg" create "StiffControl.msg" with the following code:
			Header header
	
			string[] name
			float64[] DesPos
			float64[] DesKp
			float64[] DesKd
		- Generate messages 
			>> rosgenmsg('path to main folder')	
		- Follow instructions to add javapath and folders (Note:/home/virginia/.matlab/R2017a/javaclasspath.txt)
	2. Initialize ROS, susbscribers, publishers, and messages:	
		rosinit
		masterHost = 'localhost';
		node_1 = robotics.ros.Node('node_1', masterHost);
		%%
		ActualQ = rossubscriber('/allegroHand_0/joint_states');
		pause(2)
		DesQpub = rospublisher('/allegroHand_0/joint_cmd');
		pause(2)
		DesQmsg = rosmessage(DesQpub);
		DesImp_pub = rospublisher('/allegroHand/impedance_cmd','allegro_hrii/StiffControl');
		pause(2) % Wait to ensure publisher is registered
		DesImp_msg = rosmessage(DesImp_pub);
	3. To publish:
		DesKd=sqrt(DesKp./m1);
		DesImp_msg.DesPos = qini; 
		DesImp_msg.DesKp = DesKp;
		DesImp_msg.DesKd = DesKd;
		send(DesImp_pub,DesImp_msg);
	4. To read:
		scan = receive(ActualQ,0.1);
		scanQ=scan.Position;

- USING GRAVITY COMPENSATION
	- If you want to use the gravity compensation from the allegro code, you have to girst make the corresponding function public
		1) Go to /home/virginia/allegro_hand_ros_catkin/src/bhand/include/bhand
		2) Change void CalculateGravity(); from "Private" to "Public"


	






