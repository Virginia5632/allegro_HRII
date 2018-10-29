#include "allegro_node_grav.h"

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

using namespace std;

// Default parameters.
double k_p[DOF_JOINTS] =
        {
                // Default P Gains for PD Controller, loaded if
                // 'gains_pd.yaml' file is not loaded.
                600.0, 600.0, 600.0, 1000.0, 600.0, 600.0, 600.0, 1000.0,
                600.0, 600.0, 600.0, 1000.0, 1000.0, 1000.0, 1000.0, 600.0
        };

double k_d[DOF_JOINTS] =
        {
                // Default D Gains for PD Controller, loaded if
                // 'gains_pd.yaml' file is not loaded.
                15.0, 20.0, 15.0, 15.0, 15.0, 20.0, 15.0, 15.0,
                15.0, 20.0, 15.0, 15.0, 30.0, 20.0, 20.0, 15.0
        };
        
double q_des[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0,-0.1745,0.7854,0.7854,0,-0.1745,0.7854,0.7854,0.0873,-0.0873,0.8727,0.7854,0.8727,0.4363,0.2618,0.7854
        };        

double home_pose[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };

std::string pGainParams[DOF_JOINTS] =
        {
                "~gains_pd/p/j00", "~gains_pd/p/j01", "~gains_pd/p/j02",
                "~gains_pd/p/j03",
                "~gains_pd/p/j10", "~gains_pd/p/j11", "~gains_pd/p/j12",
                "~gains_pd/p/j13",
                "~gains_pd/p/j20", "~gains_pd/p/j21", "~gains_pd/p/j22",
                "~gains_pd/p/j23",
                "~gains_pd/p/j30", "~gains_pd/p/j31", "~gains_pd/p/j32",
                "~gains_pd/p/j33"
        };

std::string dGainParams[DOF_JOINTS] =
        {
                "~gains_pd/d/j00", "~gains_pd/d/j01", "~gains_pd/d/j02",
                "~gains_pd/d/j03",
                "~gains_pd/d/j10", "~gains_pd/d/j11", "~gains_pd/d/j12",
                "~gains_pd/d/j13",
                "~gains_pd/d/j20", "~gains_pd/d/j21", "~gains_pd/d/j22",
                "~gains_pd/d/j23",
                "~gains_pd/d/j30", "~gains_pd/d/j31", "~gains_pd/d/j32",
                "~gains_pd/d/j33"
        };

std::string initialPosition[DOF_JOINTS] =
        {
                "~initial_position/j00", "~initial_position/j01",
                "~initial_position/j02",
                "~initial_position/j03",
                "~initial_position/j10", "~initial_position/j11",
                "~initial_position/j12",
                "~initial_position/j13",
                "~initial_position/j20", "~initial_position/j21",
                "~initial_position/j22",
                "~initial_position/j23",
                "~initial_position/j30", "~initial_position/j31",
                "~initial_position/j32",
                "~initial_position/j33"
        };

// Constructor subscribes to topics.	
AllegroNodeImpedance::AllegroNodeImpedance()
        : AllegroNode() {
  control_hand_ = false;
  pBHand = new BHand(eHandType_Right);
  initController(whichHand);

  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &AllegroNodeImpedance::libCmdCallback, this);

  joint_cmd_sub = nh.subscribe(
          DESIRED_STATE_TOPIC, 1, &AllegroNodeImpedance::setJointCallback, this);
          
  impedance_cmd_sub = nh.subscribe(
          DESIRED_IMPEDANCE_TOPIC, 1, &AllegroNodeImpedance::setImpedanceCallback, this);          
         
}

AllegroNodeImpedance::~AllegroNodeImpedance() {
  ROS_INFO("PD controller node is shutting down");
}


allegroKDL::allegroKDL(vector<double> g_vec)
{
  // load urdf file
//   _urdf_file = ros::package::getPath("ll4ma_robots_description");
//   _urdf_file.append("/robots/depreciated/urdf/allegro_kdl.urdf");
  _urdf_file="/home/virginia/allegro_hand_ros_catkin/src/allegro_hand_description/allegro_hand_description_right.urdf";
  _ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  _base_names={"palm_link","palm_link","palm_link","palm_link"};
  _g_vec=g_vec;

  // build kdl tree and chains:
  _allegro_kdl=new robotKDL(_urdf_file,_base_names,_ee_names,_g_vec);
//    _allegro_kdl=new robotKDL(_base_names,_ee_names,_g_vec);
}


// Called when an external (string) message is received
void AllegroNodeImpedance::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());

  const std::string lib_cmd = msg->data.c_str();

  // Compare the message received to an expected input
  if (lib_cmd.compare("pdControl") == 0) {
    control_hand_ = true;
  }

  else if (lib_cmd.compare("home") == 0) {
    // Set the home position as the desired joint states.
    mutex->lock();
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    control_hand_ = true;
    mutex->unlock();
  }
  else if (lib_cmd.compare("off") == 0) {
    control_hand_ = false;
  }

  else if (lib_cmd.compare("save") == 0) {
    // Set the current position as the desired joint states.
    mutex->lock();
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = current_position[i];
    mutex->unlock();
  }
}

void AllegroNodeImpedance::setJointCallback(const sensor_msgs::JointState &msg) {
  ROS_WARN_COND(!control_hand_, "Setting control_hand_ to True because of "
                "received JointState message");
  
  control_hand_ = true;
  imp_received=false;
}

void AllegroNodeImpedance::setImpedanceCallback(const allegro_hrii::StiffControl &msg) {
  ROS_WARN("Impedance received");
  mutex->lock();
  for(int i=0; i<16;i++){
     q_des[i]=msg.DesPos[i];
     k_p[i]=msg.DesKp[i];
     k_d[i]=msg.DesKd[i];
  }
  imp_received=true;
  control_hand_ = false;
  mutex->unlock();
}


void AllegroNodeImpedance::computeDesiredTorque() {
  // NOTE: here we just compute and set the desired_torque class member
  // variable. AllegroNode::updateWriteReadCAN() will send the desired torque to
  // the hand.

  // No control: set torques to zero.
  if (((!control_hand_) && (!imp_received)) || ((control_hand_) && (imp_received))) {
     //ROS_INFO_THROTTLE(1.0, "Hand control is false");
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_torque[i] = 0.0;
    }
    return;
  }

  if ((control_hand_) && (!imp_received)){
  // Sanity/defensive check: if *both* position and torques are set in the
  // message, do nothing.
  if (desired_joint_state.position.size() > 0 &&
      desired_joint_state.effort.size() > 0) {
    ROS_WARN("Error: both positions and torques are specified in the desired "
                     "state. You cannot control both at the same time.");
    return;
  }

  
    mutex->lock();

    if (desired_joint_state.position.size() == DOF_JOINTS) {
      // Control joint positions: compute the desired torques (PD control).
      double error;
      for (int i = 0; i < DOF_JOINTS; i++) {
        error = desired_joint_state.position[i] - current_position_filtered[i];
        desired_torque[i] = 1.0/canDevice->torqueConversion() *
                (k_p[i] * error - k_d[i] * current_velocity_filtered[i]);
      }
    } else if (desired_joint_state.effort.size() > 0) {
      // Control joint torques: set desired torques as the value stored in the
      // desired_joint_state message.
      for (int i = 0; i < DOF_JOINTS; i++) {
        desired_torque[i] = desired_joint_state.effort[i];
      }
    }
    mutex->unlock();
  }
  ////////IMPEDANCE CONTROL
  if ((!control_hand_) && (imp_received)){  
	mutex->lock();  
		
	float my_G[16];
	vector<double> my_G_balak;
	my_G_balak.resize(16,0.0);		
	my_G_balak=myGrav(current_position_filtered);
	
	float new_my_G[16]; // convert from vector double to array of floats 
        for (int i =0;i<4;i++)
	{
	  new_my_G[i]=my_G_balak[i];
	}
	
	int my_index;
	//pBHand->SetJointDesiredPosition(current_position_filtered); 
// 	pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
// 	pBHand->SetJointPosition(current_position_filtered);
//         pBHand->UpdateControl((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);
//         pBHand->GetJointTorque(desired_torque);    
//    // pBHand->CalculateGravity();    
//     for (int i = 0; i < 4; i++) {
// 		for (int j = 0; j < 4; j++) {
// 			my_index=(i+1)*4-4+j;
// 			my_G[my_index]=desired_torque[my_index];
// 			//my_G[my_index]=pBHand->_G[i][j];
// 			gravity_mine.data[my_index]=my_G[my_index];			
// 		}
// 	}	
// 	gravity_mine_pub.publish(gravity_mine);
	
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			my_index=(i+1)*4-4+j;
			my_G[my_index]=new_my_G[my_index];
			//my_G[my_index]=pBHand->_G[i][j];
			gravity_mine.data[my_index]=my_G[my_index];			
		}
	}	
	gravity_mine_pub.publish(gravity_mine);
	
	   
      // Control joint positions: compute the desired torques (PD control).
      double error;
      for (int i = 0; i < DOF_JOINTS; i++) {
        error = q_des[i] - current_position_filtered[i];
        desired_torque[i] = 1.0/canDevice->torqueConversion() *
                (k_p[i] * error - k_d[i] * current_velocity_filtered[i])+my_G[i];
        //desired_torque[i]= my_G[i];      
      }
    
    mutex->unlock();
	  
  }
  
}

void AllegroNodeImpedance::initController(const std::string &whichHand) {
  gravity_mine.data.resize(DOF_JOINTS); //VRG
  gravity_mine_pub =nh.advertise<std_msgs::Float64MultiArray>(GRAVITY_STATE_TOPIC, 100);//VRG 
  
  // set gains_pd via gains_pd.yaml or to default values
  if (ros::param::has("~gains_pd")) {
    ROS_INFO("CTRL: PD gains loaded from param server.");
    for (int i = 0; i < DOF_JOINTS; i++) {
      ros::param::get(pGainParams[i], k_p[i]);
      ros::param::get(dGainParams[i], k_d[i]);
    }
  }
  else {
    // gains will be loaded every control iteration
    ROS_WARN("CTRL: PD gains not loaded");
    ROS_WARN("Check launch file is loading /parameters/gains_pd.yaml");
    ROS_WARN("Loading default PD gains...");
  }

  // set initial position via initial_position.yaml or to default values
  if (ros::param::has("~initial_position")) {
    ROS_INFO("CTRL: Initial Pose loaded from param server.");
    double tmp;
    mutex->lock();
    desired_joint_state.position.resize(DOF_JOINTS);
    for (int i = 0; i < DOF_JOINTS; i++) {
      ros::param::get(initialPosition[i], tmp);
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(tmp);
    }
    mutex->unlock();
  }
  else {
    ROS_WARN("CTRL: Initial position not loaded.");
    ROS_WARN("Check launch file is loading /parameters/initial_position.yaml");
    ROS_WARN("Loading Home position instead...");

    // Home position
    mutex->lock();
    desired_joint_state.position.resize(DOF_JOINTS);
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    mutex->unlock();
  }
  control_hand_ = false;

  printf("*************************************\n");
  printf("      Joint PD Control Method        \n");
  printf("-------------------------------------\n");
  printf("  Only 'H', 'O', 'S', 'Space' works. \n");
  printf("*************************************\n");
}

void AllegroNodeImpedance::doIt(bool polling) {
  // Main spin loop, uses the publisher/subscribers.
  if (polling) {
    ROS_INFO("Polling = true.");
    while (ros::ok()) {
      updateController();
      ros::spinOnce();
    }
  } else {
    ROS_INFO("Polling = false.");

    // Timer callback (not recommended).
    ros::Timer timer = startTimerCallback();
    ros::spin();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_pd");
  AllegroNodeImpedance allegroNode;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }
  allegroNode.doIt(polling);
}



vector<double> allegroKDL::get_G(vector<double> q)
{
  _tau_g.resize(16);
  // send only joints of idx finger:
  for (int j=0;j<4;j++)
    {
      _q_finger.resize(4);
      for (int i =0;i<4;i++)
	{
	  _q_finger[i]=q[j*4+i];
	}
      _tau_g_finger=_allegro_kdl->getGtau(j,_q_finger);
      for (int i =0;i<4;i++)
	{
	  _tau_g[j*4+i]=_tau_g_finger[i];
	}
    }
  return _tau_g;
}


vector<double> myGrav (double* q)
{
//  //vector<double> g_vec={0.0,-18.0,0.0}; // in-grasp box 
//  //vector<double> g_vec={0.0,0.0,-18.0};// upright
//  vector<double> g_vec={18.6,0.0,0.0}; // in-grasp box 
 
 vector<double> g_vec={0.0,0.0,-9.81}; // in-grasp box 
//  float TransfEE[16];
//  TransfEE=franka_states.O_T_EE;
//  g_vec[0]=TransfEE[0]*g_vec[0]+TransfEE[1]*g_vec[1]+TransfEE[2]*g_vec[2];
//  g_vec[1]=TransfEE[4]*g_vec[0]+TransfEE[5]*g_vec[1]+TransfEE[6]*g_vec[2];
//  g_vec[2]=TransfEE[8]*g_vec[0]+TransfEE[9]*g_vec[1]+TransfEE[10]*g_vec[2];
 
 allegroKDL kdl_comp(g_vec);
 vector<double> tau_g;
 tau_g.resize(16,0.0);
 
 std::vector<double> q_vector(q, q + 16);
 tau_g=kdl_comp.get_G(q_vector);
 
return tau_g;
 
// double g= 9.81f;
// double mass1[4] = {0.0119, 0.065, 0.0355, 0.0096+0.0168};  //link 1, 2, 3, 4 of finger 1
// double mass2[4] = {0.0119, 0.065, 0.0355, 0.0096+0.0168};  //link 1, 2, 3, 4 of finger 1
// double mass3[4] = {0.0119, 0.065, 0.0355, 0.0096+0.0168};  //link 1, 2, 3, 4 of finger 1
// double mass4[4] = {0.0176, 0.0119, 0.038, 0.0388+0.0168};  //link 1, 2, 3, 4 of thumb
// 
// double inet_f1[3][3] = {1.01666658333e-06, 0.0, 0.01,
// 	 0.0, 6.47677333333e-07, 0.0,
// 	 0.0, 0.0, 1.01666658333e-06};
// double inet_f2[3][3] = {7.95654166667e-05, 1.7199e-05, 8.75875e-06,
// 	 1.7199e-05, 2.47088833333e-05, 2.413125e-05,
// 	 8.75875e-06, 2.413125e-05, 7.95654166667e-05};
// 	 
// double inet_f3[3][3] = {2.63979183333e-05,6.67968e-06,4.783625e-06,
// 	 6.67968e-06,4.783625e-06,4.783625e-06,
// 	 4.783625e-06,4.783625e-06,2.63979183333e-05};	 
// 	 
// double inet_f4[3][3] = {1.255968e-06+9.68e-07, 1.255968e-06, 1.2936e-06,
// 	 1.255968e-06, 3.649312e-06+9.68e-07, 1.7622e-06,
// 	 1.2936e-06, 1.7622e-06, 4.701248e-06+9.68e-07};
// 	 
// double inet_t1[3][3] = {1.89273333333e-5,7.16716e-06,5.35568e-06,
// 	 7.16716e-06,1.43008213333e-05,6.8068e-06,
// 	 5.35568e-06,6.8068e-06,1.89273333333e-05};	 
// double inet_t2[3][3] = {4.24250866667e-06,1.032087e-06,1.603525e-06,
// 	 1.032087e-06,4.52362633333e-06,1.44808125e-06,
// 	 1.603525e-06,1.44808125e-06,4.24250866667e-06};	
// double inet_t3[3][3] = {4.30439933333e-05,9.57068e-06,5.1205e-06,
// 	 9.57068e-06,1.44451933333e-05,1.342825e-05,
// 	 5.1205e-06,1.342825e-05,4.30439933333e-05};
// double inet_t4[3][3] = {3.29223173333e-05+9.68e-07,8.042076e-06,5.2283e-06,
// 	 8.042076e-06,1.47493026667e-5+9.68e-07,1.1283525e-5,
// 	 5.2283e-06,1.1283525e-5,3.29223173333e-05+9.68e-07};
//  vector<double> g_vec={0.0,0.0,-18.0};// upright

 // create kdl controller instance:
  //vector<double> g_vec={0.0,-18.0,0.0}; // in-grasp box 
  //vector<double> g_vec={0.0,0.0,-18.0};// upright
 
	
 }
