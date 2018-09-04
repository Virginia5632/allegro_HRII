To have the full installation:
- Follow instructions to create an original allegro_hand_ros_catkin as in the website

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

- Donwload "allegro_hrii" and copy it in allegro_hand_ros_catkin/src

- Do >>git clone https://bitbucket.org/robot-learning/ll4ma_kdl.git to get the library for the robot from Balakumar Sundaralingam work into the main src file of "/allegro_hand_ros_catkin"


- To git:

cd allegro_hrii/ --> Go to package folder
git add -A --> Add everything inside the folder to commit later
git commit -m "first commit"   --> commit
git remote add origin https://github.com/Virginia5632/allegro_hrii.git
git push -u origin master  --> push to the web

git gui -->  wizard to commit and push easier (FIRST go to "allegro_hrii" folder)

git stash --> To go back to website version
git stash pop --> To undo stash

////////////////////////////////////////////////
/////////// SOME NOTES FORM THE PAST ///////////
////////////////////////////////////////////////

/////// INSTALL AND USE ALLEGRO HAND //////////
--- Install and use the Allegro Hand ROS Node called allegro_hand_ros_catkin ---
         (https://github.com/simlabrobotics/allegro_hand_ros_catkin)

-- Installing the PCAN driver

Before using the hand, you must install the pcan drivers. This assumes you have a peak-systems pcan to usb adapter.

    1. Install these packages:

    sudo apt-get install libpopt-dev ros-indigo-libpcan

    2. Download latest drivers: http://www.peak-system.com/fileadmin/media/linux/index.htm#download

    3. Install the drivers:

            make clean
            make NET=NO_NETDEV_SUPPORT
            sudo make install
            sudo /sbin/modprobe pcan

    4. Test that the interface is installed properly with:

            cat /proc/pcan

       You should see some stuff streaming.

    5. When the hand is connected, you should see pcanusb0 or pcanusb1 in the list of available interfaces:

            ls -l /dev/pcan*

        If you do not see any available files, you may need to run:

            sudo ./driver/pcan_make_devices 2

        from the downloaded pcan folder: this theoretically creates the
        devices files if the system has not done it automatically.
        
        
-- Installing the ROS node

    1. Download the package:
    
            git clone https://github.com/simlabrobotics/allegro_hand_ros_catkin.git
    
    2. Build the package
            
            cd allegro_hand_ros_catkin
            catkin_make


/////// NEW NODE AND MESSAGE ///////

1. We installed the package I was using also in Virginia's computer
2. We created a new node inside allegro_hand_controllers called allegro_node_impedance (files: allegro_node_impedance.cpp, allegro_node_impedance.h)
3. We created a new message type called StiffControl following the guide here: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
4. The allegro node either does pd control as it was done in the allegro_node_pd, or it uses the kp, kd and q_des published inside the topic /allegroHand_0/impedance_cmd

Test the controller:

- terminal 1:
>> roslaunch allegro_hand_controllers allegro_hand.launch HAND:=right CONTROLLER:=impedance

- terminal 2 (Home position with high stiffness):
>> rostopic pub /allegroHand_0/impedance_cmd allegro_hand_controllers/StiffControl "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0','joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0','joint_8.0', 'joint_9.0', 'joint_10.0', 'joint_11.0','joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0']
DesPos: [0,   -0.1745,    0.7854,    0.7854 , 0,-0.1745,0.7854,    0.7854,    0.0873,   -0.0873,0.8727, 0.7854,    0.8727 ,   0.4363,    0.2618,    0.7854]
DesKp: [600.0, 600.0, 600.0, 1000.0,600.0, 600.0, 600.0, 1000.0,600.0, 600.0, 600.0, 1000.0,1000.0, 1000.0, 1000.0, 600.0]
DesKd: [15.0, 20.0, 15.0, 15.0, 15.0, 20.0, 15.0, 15.0,15.0, 20.0, 15.0, 15.0,30.0, 20.0, 20.0, 15.0]"

(Grasp position with low stiffness in index):
>> rostopic pub /allegroHand_0/impedance_cmd allegro_hand_controllers/StiffControl "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['joint_0.0', 'joint_1.0', 'joint_2.0', 'joint_3.0','joint_4.0', 'joint_5.0', 'joint_6.0', 'joint_7.0','joint_8.0', 'joint_9.0', 'joint_10.0', 'joint_11.0','joint_12.0', 'joint_13.0', 'joint_14.0', 'joint_15.0']
DesPos: [0,   -0.1745,    0.7854,    0.7854 , 0,-0.1745,0.7854,    0.7854,    0.0873,   -0.0873,0.8727, 0.7854,    0.8727 ,   0.4363,    0.2618,    0.7854]
DesKp: [600.0, 600.0, 600.0, 1000.0,600.0, 600.0, 600.0, 1000.0,600.0, 600.0, 600.0, 1000.0,1000.0, 1000.0, 1000.0, 600.0]
DesKd: [15.0, 20.0, 15.0, 15.0, 15.0, 20.0, 15.0, 15.0,15.0, 20.0, 15.0, 15.0,30.0, 20.0, 20.0, 15.0]"



