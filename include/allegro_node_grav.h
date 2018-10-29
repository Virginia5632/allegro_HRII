#ifndef __ALLEGRO_NODE_GRAV_H__
#define __ALLEGRO_NODE_GRAV_H__


#include "/home/virginia/allegro_hand_ros_catkin/src/allegro_hand_controllers/src/allegro_node.h"

#include "allegro_hrii/StiffControl.h" //VRG

#include "bhand/BHand.h"

#include <stdio.h>
#include "ros/ros.h"

//#include "sensor_msgs/JointState.h"
//#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h" //VRG

#include "ll4ma_kdl/manipulator_kdl/robot_kdl_bck.h"
#include <ros/package.h>

using namespace manipulator_kdl_bck;

// Joint-space PD control of the Allegro hand.
//
// Allows you to save a position and command it to the hand controller.
// Controller gains are loaded from the ROS parameter server.
class AllegroNodeImpedance : public AllegroNode {

 public:
  AllegroNodeImpedance();

  ~AllegroNodeImpedance();

  // Main spin code: just waits for messages.
  void doIt(bool polling = false);

  // Uses the String received command to set the hand into its home
  // position, or saves the grasp in order to go into PD control mode. Also
  // can turn the hand off.
  void libCmdCallback(const std_msgs::String::ConstPtr &msg);
  void setJointCallback(const sensor_msgs::JointState &msg);  
  void setImpedanceCallback(const allegro_hrii::StiffControl &msg);  //VRG

  // Loads all gains and initial positions from the parameter server.
  void initController(const std::string &whichHand);

  // PD control happens here.
  void computeDesiredTorque();
  BHand *pBHand = NULL;
  
  protected:
  // Handles defined grasp commands (std_msgs/String).
  ros::Subscriber lib_cmd_sub;

  // Subscribe to desired joint states, only so we can set control_hand_ to true
  // when we receive a desired command.
  ros::Subscriber joint_cmd_sub;
  
  // Added VRG
  // Subscribe to desired impedance states
  ros::Subscriber impedance_cmd_sub;
  //ros::Subscriber RotArmEE_cmd_sub;
  ros::Subscriber franka_states;
  // Added VRG
  ros::Publisher gravity_mine_pub; //VRG 
  std_msgs::Float64MultiArray gravity_mine; //VRG
  
  // If this flag is true, the hand will be controlled (either in joint position
  // or joint torques). If false, desired torques will all be zero.
  bool control_hand_ = false;
  
  bool imp_received=false;  //VRG If this flag is set the hand will be controlled in impedance
  
};


// Added VRG
const std::string DESIRED_IMPEDANCE_TOPIC = "allegroHand/impedance_cmd";
//const std::string ARM_EE_TOPIC = "allegroHand/RotArmEE_cmd";
const std::string GRAVITY_STATE_TOPIC = "allegroHand/gravity_state";
// Added VRG

class allegroKDL
{
 public:
  allegroKDL(vector<double> g_vec);// constructor- builds kdl chain

    // Dynamic functions:
  // gravity compensation
   vector<double> get_G(vector<double> q);
  // Yay! function overloading!
  vector<double> get_G(int idx,vector<double> q);
 
 private:
  vector<double> _g_vec;//={0.0,0.0,-9.8};
  string _urdf_file;// = ros::package::getPath("ll4ma_robots_description");
  
  vector<string> _ee_names;//={"palm_link","index_tip","middle_tip","ring_tip","thumb_tip"};
  vector<string> _base_names;//={"base_link","base_link","base_link","base_link","base_link"};
  robotKDL* _allegro_kdl;
  vector<double> _q_finger;
  vector<double> _tau_g_finger;
  vector<double> _tau_g;
  double delta_q;
  
};

vector<double> myGrav(double* q);




#endif  // __ALLEGRO_NODE_GRAV_H__
