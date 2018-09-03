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


/*void myGrav()
{
double g = 9.81f;
double mass1[4] = {_mass[0][0], _mass[1][0], _mass[2][0], _mass[3][0]};
double mass2[4] = {_mass[0][1], _mass[1][1], _mass[2][1], _mass[3][1]};
double mass3[4] = {_mass[0][2], _mass[1][2], _mass[2][2], _mass[3][2]};
double mass4[4] = {_mass[0][3], _mass[1][3], _mass[2][3], _mass[3][3]};

_G[0][0] = 0;
_G[0][1] = -27.0f*g*mass2[0]*S2_C[0]+g*mass3[0]*(-25.886f*S23_C[0]+0.137f*C23_C[0]-54.0f*S2_C[0])+g*mass4[0]*(-20.571f*S234_C[0]+0.193f*C234_C[0]-38.4f*S23_C[0]-54.0f*S2_C[0]);
_G[0][2] = g*mass3[0]*(-25.886f*S23_C[0]+0.137f*C23_C[0])+g*mass4[0]*(-20.571f*S234_C[0]+0.193f*C234_C[0]-38.4f*S23_C[0]);
_G[0][3] = g*mass4[0]*(-20.571f*S234_C[0]+0.193f*C234_C[0]);

_G[1][0] = 0;
_G[1][1] = -27.0f*g*mass2[1]*S2_C[1]+g*mass3[1]*(-25.886f*S23_C[1]+0.137f*C23_C[1]-54.0f*S2_C[1])+g*mass4[1]*(-20.571f*S234_C[1]+0.193f*C234_C[1]-38.4f*S23_C[1]-54.0f*S2_C[1]);
_G[1][2] = g*mass3[1]*(-25.886f*S23_C[1]+0.137f*C23_C[1])+g*mass4[1]*(-20.571f*S234_C[1]+0.193f*C234_C[1]-38.4f*S23_C[1]);
_G[1][3] = g*mass4[1]*(-20.571f*S234_C[1]+0.193f*C234_C[1]);

_G[2][0] = g*mass1[2]*(-0.049473f*C1_C[2]-0.005139f*S1_C[2])+2.3517f*g*mass2[2]*C1_C[2]*S2_C[2]+0.0871f*g*mass3[2]*C1_C[2]*(25.886f*S23_C[2]-0.137f*C23_C[2]+54.0f*S2_C[2])+g*mass4[2]*(0.0871f*C1_C[2]*(20.571f*S234_C[2]-0.193f*C234_C[2]+38.4f*S23_C[2]+54.0f*S2_C[2])-0.001742f*S1_C[2]);
_G[2][1] = g*mass2[2]*(2.3517f*S1_C[2]*C2_C[2]-26.8974f*S2_C[2])+g*mass3[2]*(0.0871f*S1_C[2]*(25.886f*C23_C[2]+0.137f*S23_C[2]+54.0f*C2_C[2])-25.78763f*S23_C[2]+0.136479f*C23_C[2]-53.7948f*S2_C[2])+g*mass4[2]*(0.0871f*S1_C[2]*(20.571f*C234_C[2]+0.193f*S234_C[2]+38.4f*C23_C[2]+54.0f*C2_C[2])-20.49283f*S234_C[2]+0.192267f*C234_C[2]-38.25408f*S23_C[2]-53.7948f*S2_C[2]);
_G[2][2] = g*mass3[2]*(0.0871f*S1_C[2]*(25.886f*C23_C[2]+0.137f*S23_C[2])-25.78763f*S23_C[2]+0.136479f*C23_C[2])+g*mass4[2]*(0.0871f*S1_C[2]*(20.571f*C234_C[2]+0.193f*S234_C[2]+38.4f*C23_C[2])-20.49283f*S234_C[2]+0.192267f*C234_C[2]-38.25408f*S23_C[2]);
_G[2][3] = g*mass4[2]*(0.0871f*S1_C[2]*(20.571f*C234_C[2]+0.193f*S234_C[2])-20.49283f*S234_C[2]+0.192267f*C234_C[2]);

_G[3][0] = (g*mass1[3]*(2110433.0f*C1_C[3] + 2516319.0f*S1_C[3]))/2500000.0f + g*mass4[3]*((871.0f*C1_C[3]*(257.0f*S2_C[3]*S3_C[3] - 685.0f*C34_C[3]*S2_C[3] + 62570.0f*S34_C[3]*S2_C[3] + 25.0f))/50000000.0f + (871*S1_C[3]*(62570.0f*C34_C[3] + 685.0f*S34_C[3] + 257.0f*C3_C[3] + 274.0f))/50000000.0f) + (g*mass2[3]*(43550.0f*C1_C[3] + 51107667.0f*S1_C[3] - 513890.0f*C1_C[3]*C2_C[3] - 4947280.0f*C1_C[3]*S2_C[3]))/100000000.0f + g*mass3[3]*((871.0f*S1_C[3]*(62570.0f*C3_C[3] + 685.0f*S3_C[3] + 274.0f))/50000000.0f + (871.0f*C1_C[3]*(12514.0f*S2_C[3]*S3_C[3] - 137.0f*C3_C[3]*S2_C[3] + 5.0f))/10000000.0f);
_G[3][1] = (g*mass2[3]*(5877580.0f*C2_C[3] + 56584160.0f*S2_C[3] - 4947280.0f*C2_C[3]*S1_C[3] + 513890.0f*S1_C[3]*S2_C[3]))/100000000.0f - g*mass4[3]*((1280117.0f*S2_C[3]*S3_C[3])/25000000.0f - (871.0f*S1_C[3]*(257.0f*C2_C[3]*S3_C[3] - 685.0f*C34_C[3]*C2_C[3] + 62570.0f*S34_C[3]*C2_C[3]))/50000000.0f - (682397.0f*C34_C[3]*S2_C[3])/5000000.0f + (31166117.0f*S34_C[3]*S2_C[3])/2500000.0f) - g*mass3[3]*((31166117.0f*S2_C[3]*S3_C[3])/2500000.0f - (682397.0f*C3_C[3]*S2_C[3])/5000000.0f + (871.0f*S1_C[3]*(137.0f*C2_C[3]*C3_C[3] - 12514.0f*C2_C[3]*S3_C[3]))/10000000.0f);
_G[3][2] = g*mass3[3]*((31166117.0f*C2_C[3]*C3_C[3])/2500000.0f - (871.0f*C1_C[3]*(685.0f*C3_C[3] - 62570.0f*S3_C[3]))/50000000.0f + (682397.0f*C2_C[3]*S3_C[3])/5000000.0f + (871.0f*S1_C[3]*(12514.0f*C3_C[3]*S2_C[3] + 137.0f*S2_C[3]*S3_C[3]))/10000000.0f) + g*mass4[3]*((1280117.0f*C2_C[3]*C3_C[3])/25000000.0f + (871.0f*S1_C[3]*(257.0f*C3_C[3]*S2_C[3] + 62570.0f*C34_C[3]*S2_C[3] + 685.0f*S34_C[3]*S2_C[3]))/50000000.0f + (31166117.0f*C34_C[3]*C2_C[3])/2500000.0f + (682397.0f*S34_C[3]*C2_C[3])/5000000.0f + (871.0f*C1_C[3]*(62570.0f*S34_C[3] - 685.0f*C34_C[3] + 257.0f*S3_C[3]))/50000000.0f);
_G[3][3] = g*mass4[3]*((871.0f*S1_C[3]*(62570.0f*C34_C[3]*S2_C[3] + 685.0f*S34_C[3]*S2_C[3]))/50000000.0f - (871.vicky5632
* 0f*C1_C[3]*(685.0f*C34_C[3] - 62570.0f*S34_C[3]))/50000000.0f + (31166117.0f*C34_C[3]*C2_C[3])/2500000.0f + (682397.0f*S34_C[3]*C2_C[3])/5000000.0f);

for (int i=0; i<NOF; i++)
{
_G[i][0] = _G[i][0] *0.001f;
_G[i][1] = _G[i][1] *0.001f;
_G[i][2] = _G[i][2] *0.001f;
_G[i][3] = _G[i][3] *0.001f;
}
}*/


void myGrav()
{
double g= 9.81f;
double mass1[4] = {0.0119, 0.065, 0.0355, 0.0096+0.0168};  //link 1, 2, 3, 4 of finger 1
double mass2[4] = {0.0119, 0.065, 0.0355, 0.0096+0.0168};  //link 1, 2, 3, 4 of finger 1
double mass3[4] = {0.0119, 0.065, 0.0355, 0.0096+0.0168};  //link 1, 2, 3, 4 of finger 1
double mass4[4] = {0.0176, 0.0119, 0.038, 0.0388+0.0168};  //link 1, 2, 3, 4 of thumb

double inet_f1[3][3] = {1.01666658333e-06, 0.0, 0.01,
	 0.0, 6.47677333333e-07, 0.0,
	 0.0, 0.0, 1.01666658333e-06};
double inet_f2[3][3] = {7.95654166667e-05, 1.7199e-05, 8.75875e-06,
	 1.7199e-05, 2.47088833333e-05, 2.413125e-05,
	 8.75875e-06, 2.413125e-05, 7.95654166667e-05};
	 
double inet_f3[3][3] = {2.63979183333e-05,6.67968e-06,4.783625e-06,
	 6.67968e-06,4.783625e-06,4.783625e-06,
	 4.783625e-06,4.783625e-06,2.63979183333e-05};	 
	 
double inet_f4[3][3] = {1.255968e-06+9.68e-07, 1.255968e-06, 1.2936e-06,
	 1.255968e-06, 3.649312e-06+9.68e-07, 1.7622e-06,
	 1.2936e-06, 1.7622e-06, 4.701248e-06+9.68e-07};
	 
double inet_t1[3][3] = {1.89273333333e-5,7.16716e-06,5.35568e-06,
	 7.16716e-06,1.43008213333e-05,6.8068e-06,
	 5.35568e-06,6.8068e-06,1.89273333333e-05};	 
double inet_t2[3][3] = {4.24250866667e-06,1.032087e-06,1.603525e-06,
	 1.032087e-06,4.52362633333e-06,1.44808125e-06,
	 1.603525e-06,1.44808125e-06,4.24250866667e-06};	
double inet_t3[3][3] = {4.30439933333e-05,9.57068e-06,5.1205e-06,
	 9.57068e-06,1.44451933333e-05,1.342825e-05,
	 5.1205e-06,1.342825e-05,4.30439933333e-05};
double inet_t4[3][3] = {3.29223173333e-05+9.68e-07,8.042076e-06,5.2283e-06,
	 8.042076e-06,1.47493026667e-5+9.68e-07,1.1283525e-5,
	 5.2283e-06,1.1283525e-5,3.29223173333e-05+9.68e-07};
	 

  // initialize kdl class:
  // load robot from robot_description:
  /*KDL::JntArray q(joint_pos.size());
  KDL::JntArray tau_g(joint_pos.size());
  
  q.data = joint_pos;
  dyn_solvers_[chain_idx]->JntToGravity(q, tau_g);
  tau_g_ = tau_g.data;*/
  
    
  //q.data =  current_position_filtered;
  //dyn_solvers_[chain_idx]->JntToGravity(current_position_filtered, tau_g);
  //tau_g_ = tau_g.data;
  
 
 /*vector<double> g_vec={0.0,0.0,-9.8}; 
 manipulator_kdl::robotKDL lbr4_("robot_description",n,base_names,ee_names,g_vec);*/
 /* cerr<<"Created KDL model"<<endl;
  Eigen::VectorXd j_pos;
  j_pos.resize(7);
  j_pos.setZero();
  int ch=0;
  bool rpy=true;
  Eigen::VectorXd ee_pose;
  lbr4_.getFK(ch,j_pos,ee_pose,rpy);
  cerr<<ee_pose<<endl;
  Eigen::VectorXd tau_g;
  lbr4_.getGtau(0,j_pos,tau_g);
  cerr<<tau_g<<endl;*/	 	
  
  
  /*vector<double> g_vec={0.0,-18.0,0.0}; // in-grasp box 
  //vector<double> g_vec={0.0,0.0,-18.0};// upright
  allegroKDL kdl_comp(g_vec);
  vector<double> tau_g;
  tau_g.resize(16,0.0); */

}
