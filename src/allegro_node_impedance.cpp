#include "allegro_node_impedance.h"

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
	int my_index;
	//pBHand->SetJointDesiredPosition(current_position_filtered); 
	pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
	pBHand->SetJointPosition(current_position_filtered);
    pBHand->UpdateControl((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);
    pBHand->GetJointTorque(desired_torque);    
   // pBHand->CalculateGravity();    
    for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			my_index=(i+1)*4-4+j;
			my_G[my_index]=desired_torque[my_index];
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


