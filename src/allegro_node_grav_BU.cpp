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
          
  base_cmd_sub = nh.subscribe(
          DESIRED_BASE_TOPIC, 1, &AllegroNodeImpedance::setBaseRotateCallback, this);                  
         
}

AllegroNodeImpedance::~AllegroNodeImpedance() {
  ROS_INFO("PD controller node is shutting down");
}


/*allegroKDL::allegroKDL(vector<double> g_vec)
{
  // load urdf file
//   _urdf_file = ros::package::getPath("ll4ma_robots_description");
//   _urdf_file.append("/robots/depreciated/urdf/allegro_kdl.urdf");
  _urdf_file="/home/virginia/allegro_hand_ros_catkin/src/allegro_hand_description/allegro_hand_description_right.urdf";
  _ee_names={"link_3.0_tip","link_7.0_tip","link_11.0_tip","link_15.0_tip"};
  _base_names={"palm_link","palm_link","palm_link","palm_link"};
  _g_vec=g_vec;

  // build kdl tree and chains:
  _allegro_kdl=new robotKDL(_urdf_file,_base_names,_ee_names,_g_vec);
//    _allegro_kdl=new robotKDL(_base_names,_ee_names,_g_vec);
}*/


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

double RotBaseMine [3][3]; //VRG    
double JacobBase[24][16]={0};
  /*    JacobBase = new double*[24];      
      for (int i=0; i < 24; i++) {
		  for (int j = 0; j < 16; j++)
            {
				JacobBase[i][j]=0;
            }
     }        */


void AllegroNodeImpedance::setBaseRotateCallback(const allegro_hrii::BaseRotate &msg) {
  ROS_WARN("Rotation received");
  mutex->lock();  
  //vector<double> JacobMine;
  //JacobMine.resize(384); //24*16, 4 fingers, 3 pos, 3 rot, 16 joints 
    for(int i=0; i<3;i++){
	  for(int j=0; j<3;j++){
		  RotBaseMine[i][j]=msg.RotBase[i*3+j];  		  
	  }
  } 
  rot_received=true;     
  mutex->unlock();
}

void AllegroNodeImpedance::ComputeBaseJacobian(){
	 double q[DOF_JOINTS];
     for (int i = 0; i < DOF_JOINTS; i++) {
        q[i]= current_position_filtered[i];       
      }   
	double t2 = M_PI/2; 	//pi.*(1.0./2.0);
	double t3 = q[1]+t2;	//q12+t2;
	double t4 = cos(t3);
	double t5 = cos(q[0]);	//cos(q11);
	double t6 = sin(t3);
	double t7 = cos(q[2]); 	//cos(q13);
	double t8 = t4*6.123233995736766*pow(10,-17); //4t.*6.123233995736766e-17
	double t14 = t5*t6;
	double t9 = t8-t14;
	double t10 = sin(q[2]);	//sin(q13)
	double t11 = t6*6.123233995736766*pow(10,-17);//t6.*6.123233995736766e-17
	double t12 = t4*t5;
	double t13 = t11+t12;
	double t15 = cos(q[3]);	//cos(q14);
	double t16 = t6*9.961946980917455*pow(10,-1); //t6.*9.961946980917455e-1
	double t17 = t5*6.099933241728101*pow(10,-17); //t5.*6.099933241728101e-17;
	double t18 = sin(q[0]);	//sin(q11);
	double t21 = t18*8.715574274765822*pow(10,-2);	//t18.*8.715574274765822e-2;
	double t19 = t17-t21;
	double t26 = t4*t19;
	double t20 = t16-t26;
	double t22 = t4*9.961946980917455*pow(10,-1);	//t4.*9.961946980917455e-1;
	double t23 = t6*t19;
	double t24 = t22+t23;
	double t25 = sin(q[3]);
	double t27 = t7*t13;
	double t28 = t9*t10;
	double t29 = t27+t28;
	double t30 = t7*t9;
	double t45 = t10*t13;
	double t31 = t30-t45;
	double t32 = t7*t20;
	double t33 = t10*t24;
	double t34 = t32+t33;
	double t35 = t10*t20;
	double t40 = t7*t24;
	double t36 = t35-t40;
	double t37 = t7*t20*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t7.*t20.*(2.4e1./6.25e2);
	double t38 = t15*t34*4.37*pow(10,-2); 	//t15.*t34.*4.37e-2;
	double t39 = t10*t24*((2.4*pow(10,1))/(6.25*pow(10,2))); //t10.*t24.*(2.4e1./6.25e2)
	double t41 = t5*8.715574274765822*pow(10,-2);	//t5.*8.715574274765822e-2;
	double t42 = t18*6.099933241728101*pow(10,-17);	//t18.*6.099933241728101e-17;
	double t43 = t41+t42;
	double t44 = t15*t29*4.37*pow(10,-2);	//t15.*t29.*4.37e-2;
	double t46 = t25*t31*4.37*pow(10,-2);	// t25.*t31.*4.37e-2;
	double t47 = t7*t13*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t7.*t13.*(2.4e1./6.25e2);
	double t48 = t9*t10*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t9.*t10.*(2.4e1./6.25e2);
	double t49 = t4*8.715574274765822*pow(10,-2);	//t4.*8.715574274765822e-2;
	double t50 = t5*5.336750069161489*pow(10,-18);	//t5.*5.336750069161489e-18;
	double t51 = t18*9.961946980917455*pow(10,-1);	//t18.*9.961946980917455e-1;
	double t52 = t50+t51;
	double t53 = t6*t52;
	double t54 = t49+t53;
	double t55 = t6*8.715574274765822*pow(10,-2);	//t6.*8.715574274765822e-2;
	double t57 = t4*t52;
	double t56 = t55-t57;
	double t58 = t10*t54;
	double t59 = t7*t56;
	double t60 = t58+t59;
	double t61 = t7*t54;
	double t72 = t10*t56;
	double t62 = t61-t72;
	double t63 = t6*3.306546357697854*pow(10,-18);	//t6.*3.306546357697854e-18;
	double t64 = t4*t5*((2.7*pow(10,1))/(5.0*pow(10,2))); //t4.*t5.*(2.7e1./5.0e2);
	double t65 = t44+t46+t47+t48+t63+t64;
	double t66 = t5*9.961946980917455*pow(10,-1);	//t5.*9.961946980917455e-1;
	double t67 = t18*5.336750069161489*pow(10,-18);	//t18.*5.336750069161489e-18;
	double t68 = t66-t67;
	double t69 = t44+t46+t47+t48;
	double t70 = t10*t54*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t10.*t54.*(2.4e1./6.25e2);
	double	t71 = t15*t60*4.37*pow(10,-2);	//t15.*t60.*4.37e-2;
	double	t73 = t7*t56*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t7.*t56.*(2.4e1./6.25e2);
	double	t74 = t25*t62*4.37*pow(10,-2); //t25.*t62.*4.37e-2;
	double	t75 = t44+t46;
	double	t76 = t6*4.706410108373544*pow(10,-3);	//t6.*4.706410108373544e-3;
	double	t77 = t70+t71+t73+t74+t76-t4*t52*((2.7*pow(10,1))/(5.0*pow(10,2)));	//t70+t71+t73+t74+t76-t4.*t52.*(2.7e1./5.0e2);
	double	t78 = t6*5.379451369695426*pow(10,-2);	//t6.*5.379451369695426e-2;
	double	t80 = t25*t36*4.37*pow(10,-2);	//t25.*t36.*4.37e-2;
	double	t79 = t37+t38+t39+t78-t80-t4*t19*((2.7*pow(10,1))/(5.0*pow(10,2)));	//t37+t38+t39+t78-t80-t4.*t19.*(2.7e1./5.0e2);
	double	t81 = t70+t71+t73+t74;
	double	t82 = t71+t74;
	double	t83 = q[5]+t2;	//q22+t2;
	double	t84 = cos(t83);
	double	t85 = cos(q[4]);	//cos(q21);
	double	t86 = sin(t83);
	double	t87 = cos(q[6]);	//cos(q23);
	double	t88 = t84*6.123233995736766*pow(10,-17);	//t84.*6.123233995736766e-17;
	double	t97 = t85*t86;
	double	t89 = t88-t97;
	double	t90 = sin(q[6]);	//sin(q23);
	double	t91 = t86*6.123233995736766*pow(10,-17);	//t86.*6.123233995736766e-17;
	double	t92 = t84*t85;
	double	t93 = t91+t92;
	double	t94 = t85*6.123233995736766*pow(10,-17);	//t85.*6.123233995736766e-17;
	double	t95 = sin(q[4]);		//sin(q21);	
	double	t98 = t95*6.123233995736766*pow(10,-17);	//t95.*6.123233995736766e-17;
	double	t96 = t94-t98;
	double	t99 = cos(q[7]);	//cos(q24);
	double	t104 = t84*t96;
	double	t100 = t86-t104;
	double	t101 = t86*t96;
	double	t102 = t84+t101;
	double	t103 = sin(q[7]);	//sin(q24);
	double	t105 = t87*t100;
	double	t106 = t90*t102;
	double	t107 = t105+t106;
	double	t108 = t87*t102;
	double	t123 = t90*t100;
	double	t109 = t108-t123;
	double	t110 = t87*t93;
	double	t111 = t89*t90;
	double	t112 = t110+t111;
	double	t113 = t99*t112*4.37*pow(10,-2);	//t99.*t112.*4.37e-2;
	double	t114 = t87*t89;
	double	t126 = t90*t93;
	double	t115 = t114-t126;
	double	t116 = t103*t115*4.37*pow(10,-2);	//t103.*t115.*4.37e-2;
	double	t117 = t84*t85*((2.7*pow(10,1))/(5.0*pow(10,2)));	//t84.*t85.*(2.7e1./5.0e2);
	double	t118 = t87*t93*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t87.*t93.*(2.4e1./6.25e2);
	double	t119 = t89*t90*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t89.*t90.*(2.4e1./6.25e2);
	double	t120 = t87*t100*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t87.*t100.*(2.4e1./6.25e2);
	double	t121 = t90*t102*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t90.*t102.*(2.4e1./6.25e2);
	double	t122 = t99*t107*4.37*pow(10,-2);	//t99.*t107.*4.37e-2;
	double	t124 = t103*t109*4.37*pow(10,-2);	//t103.*t109.*4.37e-2;
	double	t125 = t94+t98;
	double	t127 = t85*3.749399456654644*pow(10,-33);	//t85.*3.749399456654644e-33;
	double	t128 = t95+t127;
	double	t132 = t84*t128;
	double	t129 = t91-t132;
	double	t130 = t86*t128;
	double	t131 = t88+t130;
	double	t133 = t86*3.306546357697854*pow(10,-18);	//t86.*3.306546357697854e-18;
	double	t134 = t87*t129;
	double	t135 = t90*t131;
	double	t136 = t134+t135;
	double	t137 = t87*t131;
	double	t145 = t90*t129;
	double	t138 = t137-t145;
	double	t139 = t113+t116+t117+t118+t119+t133;
	double	t147 = t95*3.749399456654644*pow(10,-33);	//t95.*3.749399456654644e-33;
	double	t140 = t85-t147;
	double	t141 = t113+t116+t118+t119;
	double	t142 = t87*t129*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t87.*t129.*(2.4e1./6.25e2);
	double	t143 = t90*t131*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t90.*t131.*(2.4e1./6.25e2);
	double	t144 = t99*t136*4.37*pow(10,-2);	//t99.*t136.*4.37e-2;
	double	t146 = t103*t138*4.37*pow(10,-2);	//t103.*t138.*4.37e-2;
	double	t148 = t113+t116;
	double	t149 = t84*t96*3.306546357697854*pow(10,-18);	//t84.*t96.*3.306546357697854e-18;
	double	t150 = t86*((2.7*pow(10,1))/(5.0*pow(10,2)));	//t86.*(2.7e1./5.0e2);
	double	t151 = t120+t121+t122+t124+t150-t84*t96*((2.7*pow(10,1))/(5.0*pow(10,2)));	//t120+t121+t122+t124+t150-t84.*t96.*(2.7e1./5.0e2);
	double	t152 = t133+t142+t143+t144+t146-t84*t128*((2.7*pow(10,1))/(5.0*pow(10,2)));	//t133+t142+t143+t144+t146-t84.*t128.*(2.7e1./5.0e2);
	double	t153 = t142+t143+t144+t146;
	double	t154 = t120+t121+t122+t124;
	double	t155 = t122+t124;
	double	t156 = t144+t146;
	double	t157 = q[9]+t2;	//q32+t2;
	double	t158 = cos(t157);
	double	t159 = cos(q[8]);	//cos(q31);
	double	t160 = sin(t157);
	double	t161 = cos(q[10]);	//cos(q33);
	double	t162 = t158*6.123233995736766*pow(10,-17);	//t158.*6.123233995736766e-17;
	double	t168 = t159*t160;
	double	t163 = t162-t168;
	double	t164 = sin(q[10]);	//sin(q33);
	double	t165 = t160*6.123233995736766*pow(10,-17);	//t160.*6.123233995736766e-17;
	double	t166 = t158*t159;
	double	t167 = t165+t166;
	double	t169 = cos(q[11]);	//cos(q34);
	double	t170 = t160*9.961946980917455*pow(10,-1);	//t160.*9.961946980917455e-1;
	double	t171 = t159*6.099933241728101*pow(10,-17);	//t159.*6.099933241728101e-17;
	double	t172 = sin(q[8]);	//sin(q31);
	double	t173 = t172*8.715574274765811*pow(10,-2);	//t172.*8.715574274765811e-2;
	double	t174 = t171+t173;
	double	t180 = t158*t174;
	double	t175 = t170-t180;
	double	t176 = t158*9.961946980917455*pow(10,-1);	//t158.*9.961946980917455e-1;
	double	t177 = t160*t174;
	double	t178 = t176+t177;
	double	t179 = sin(q[11]);	//sin(q34);
	double	t181 = t161*t167;
	double	t182 = t163*t164;
	double	t183 = t181+t182;
	double	t184 = t161*t163;
	double	t200 = t164*t167;
	double	t185 = t184-t200;
	double	t186 = t161*t175;
	double	t187 = t164*t178;
	double	t188 = t186+t187;
	double	t189 = t161*t178;
	double	t194 = t164*t175;
	double	t190 = t189-t194;
	double	t191 = t161*t175*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t161.*t175.*(2.4e1./6.25e2);
	double	t192 = t169*t188*4.37*pow(10,-2);	//t169.*t188.*4.37e-2;
	double	t193 = t164*t178*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t164.*t178.*(2.4e1./6.25e2);
	double	t195 = t179*t190*4.37*pow(10,-2);	//t179.*t190.*4.37e-2;
	double	t196 = t159*8.715574274765811*pow(10,-2);	//t159.*8.715574274765811e-2;
	double	t197 = t172*6.099933241728101*pow(10,-17);	//t172.*6.099933241728101e-17
	double	t198 = t196-t197;
	double	t199 = t169*t183*4.37*pow(10,-2);	//t169.*t183.*4.37e-2;
	double	t201 = t179*t185*4.37*pow(10,-2);	//t179.*t185.*4.37e-2;
	double	t202 = t161*t167*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t161.*t167.*(2.4e1./6.25e2);
	double	t203 = t163*t164*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t163.*t164.*(2.4e1./6.25e2);
	double	t204 = t159*5.336750069161482*pow(10,-18);	//t159.*5.336750069161482e-18;
	double	t205 = t172*9.961946980917455*pow(10,-1);	//t172.*9.961946980917455e-1;
	double	t206 = t204-t205;
	double	t207 = t160*8.715574274765811*pow(10,-2);	//t160.*8.715574274765811e-2;
	double	t212 = t158*t206;
	double	t208 = t207-t212;
	double	t209 = t158*8.715574274765811*pow(10,-2);	//t158.*8.715574274765811e-2;
	double	t210 = t160*t206;
	double	t211 = t209+t210;
	double	t213 = t160*3.306546357697854*pow(10,-18);	//t160.*3.306546357697854e-18;
	double	t214 = t158*t159*((2.7*pow(10,1))/(5.0*pow(10,2)));	//t158.*t159.*(2.7e1./5.0e2);
	double	t215 = t199+t201+t202+t203+t213+t214;
	double	t216 = t161*t208;
	double	t217 = t164*t211;
	double	t218 = t216+t217;
	double	t219 = t161*t211;
	double	t228 = t164*t208;
	double	t220 = t219-t228;
	double	t221 = t159*9.961946980917455*pow(10,-1);	//t159.*9.961946980917455e-1;
	double	t222 = t172*5.336750069161482*pow(10,-18);	//t172.*5.336750069161482e-18;
	double	t223 = t221+t222;
	double	t224 = t199+t201+t202+t203;
	double	t225 = t161*t208*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t161.*t208.*(2.4e1./6.25e2);
	double	t226 = t164*t211*((2.4*pow(10,1))/(6.25*pow(10,2)));	//t164.*t211.*(2.4e1./6.25e2);
	double	t227 = t169*t218*4.37*pow(10,-2);	//t169.*t218.*4.37e-2;
	double	t229 = t179*t220*4.37*pow(10,-2);	//t179.*t220.*4.37e-2;
	double	t230 = t199+t201;
	double	t231 = t160*4.706410108373538*pow(10,-3);	//t160.*4.706410108373538e-3;
	double	t232 = t225+t226+t227+t229+t231-t158*t206*((2.7*pow(10,1))/(5.0*pow(10,2)));	//t225+t226+t227+t229+t231-t158.*t206.*(2.7e1./5.0e2);
	double	t233 = t160*5.379451369695426*pow(10,-2);	//t160.*5.379451369695426e-2;
	double	t234 = t191+t192+t193+t195+t233-t158*t174*((2.7*pow(10,1))/(5.0*pow(10,2)));	//t191+t192+t193+t195+t233-t158.*t174.*(2.7e1./5.0e2);
	double	t235 = t225+t226+t227+t229;
	double	t236 = t191+t192+t193+t195;
	double	t237 = t227+t229;
	double	t238 = t192+t195;
	double	t239 = sin(q[12]);	//sin(q41);
	double	t240 = q[14]+t2;	//q43+t2;
	double	t241 = cos(q[12]);	//cos(q41);
	double	t242 = q[13]+t2;	//q42+t2;
	double	t243 = cos(t240);
	double	t244 = sin(t240);
	double	t245 = sin(t242);
	double	t246 = t245*6.123233995736766*pow(10,-17);	//t245.*6.123233995736766e-17;
	double	t247 = cos(t242);
	double	t249 = t241*t247;
	double	t248 = t246-t249;
	double	t250 = cos(q[15]);	//cos(q44);
	double	t251 = t245*9.961946980917455*pow(10,-1);	//t245.*9.961946980917455e-1;
	double	t252 = t239*8.715574274765811*pow(10,-2);	//t239.*8.715574274765811e-2;
	double	t260 = t241*6.099933241728101*pow(10,-17);	//t241.*6.099933241728101e-17;
	double	t253 = -t252+t260;
	double	t254 = t247*t253;
	double	t255 = t251+t254;
	double	t256 = t241*8.715574274765811*pow(10,-2);	//t241.*8.715574274765811e-2;
	double	t257 = t239*6.099933241728101*pow(10,-17);	//t239.*6.099933241728101e-17;
	double	t258 = t256+t257;
	double	t259 = sin(q[15]);	//sin(q44);
	double	t261 = t239*t244;
	double	t275 = t243*t248;
	double	t262 = t261-t275;
	double	t263 = t239*t243;
	double	t264 = t244*t248;
	double	t265 = t263+t264;
	double	t266 = t243*t255;
	double	t267 = t244*t258;
	double	t268 = t266+t267;
	double	t269 = t244*t255;
	double	t274 = t243*t258;
	double	t270 = t269-t274;
	double	t271 = t243*t255*5.139961104911203*pow(10,-2);	//t243.*t255.*5.139961104911203e-2;
	double	t272 = t244*t258*5.139961104911203*pow(10,-2);	//t244.*t258.*5.139961104911203e-2;
	double	t273 = t250*t268*5.93*pow(10,-2);	//t250.*t268.*5.93e-2;
	double	t276 = t250*t262*5.93*pow(10,-2);	//t250.*t262.*5.93e-2;
	double	t277 = t259*t265*5.93*pow(10,-2);	//t259.*t265.*5.93e-2;
	double	t278 = t239*t244*5.139961104911203*pow(10,-2);	//t239.*t244.*5.139961104911203e-2;
	double	t279 = t247*9.961946980917455*pow(10,-1);	//t247.*9.961946980917455e-1;
	double	t313 = t245*t253;
	double	t280 = t279-t313;
	double	t281 = t247*6.123233995736766*pow(10,-17);	//t247.*6.123233995736766e-17;
	double	t282 = t241*t245;
	double	t283 = t281+t282;
	double	t284 = t239*5.336750069161482*pow(10,-18);	// t239.*5.336750069161482e-18;
	double	t292 = t241*9.961946980917455*pow(10,-1);	//t241.*9.961946980917455e-1;
	double	t285 = -t284+t292;
	double	t286 = t245*8.715574274765811*pow(10,-2);	//t245.*8.715574274765811e-2;
	double	t287 = t241*5.336750069161482*pow(10,-18);	//t241.*5.336750069161482e-18;
	double	t288 = t239*9.961946980917455*pow(10,-1);	//t239.*9.961946980917455e-1;
	double	t289 = t287+t288;
	double	t290 = t247*t289;
	double	t291 = t286+t290;
	double	t293 = t239*5.541503154379685*pow(10,-2);	//t239.*5.541503154379685e-2;
	double	t300 = t243*t248*5.139961104911203*pow(10,-2);	//t243.*t248.*5.139961104911203e-2;
	double	t294 = t276+t277+t278+t293-t300;
	double	t295 = t244*t285;
	double	t302 = t243*t291;
	double	t296 = t295-t302;
	double	t297 = t243*t285;
	double	t298 = t244*t291;
	double	t299 = t297+t298;
	double	t301 = t244*t285*5.139961104911203*pow(10,-2);	//t244.*t285.*5.139961104911203e-2;
	double	t303 = t250*t296*5.93*pow(10,-2);	//t250.*t296.*5.93e-2;
	double	t304 = t259*t299*5.93*pow(10,-2);	//t259.*t299.*5.93e-2;
	double	t305 = t247*8.715574274765811*pow(10,-2);	//t247.*8.715574274765811e-2;
	double	t315 = t245*t289;
	double	t306 = t305-t315;
	double	t307 = t276+t277;
	double	t308 = t241*5.520416061851726*pow(10,-2);	//t241.*5.520416061851726e-2;
	double	t314 = t243*t291*5.139961104911203*pow(10,-2);	//t243.*t291.*5.139961104911203e-2;
	double	t309 = t239*(-2.957361734239436*pow(10,-19))+t301+t303+t304+t308-t314;	//t239.*(-2.957361734239436e-19)+t301+t303+t304+t308-t314;
	double	t310 = t241*4.829738233584518*pow(10,-3);	//t241.*4.829738233584518e-3;
	double	t311 = t239*3.380279930054177*pow(10,-18);	//t239.*3.380279930054177e-18;
	double	t316 = t259*t270*5.93*pow(10,-2);	//t259.*t270.*5.93e-2;
	double	t312 = t271+t272+t273+t310+t311-t316;
	double	t317 = t303+t304;
	double	t318 = -t66+t67;
	double	t319 = -t41-t42;
	double	t320 = -t85+t147;
	double	t321 = -t94-t98;
	double	t322 = -t221-t222;
	double	t323 = -t305+t315;
	double	t324 = -t281-t282;
   	  
    JacobBase[0][0]=t4*t5*5.379451369695426*pow(10,-2)+t9*t10*3.825387640672303*pow(10,-2)+t7*t13*3.825387640672303*pow(10,-2)+t4*t19*3.306546357697854*pow(10,-18)-t7*t20*2.351321854362918*pow(10,-18)-t10*t24*2.351321854362918*pow(10,-18)+t15*t29*4.353370830660928*pow(10,-2)-t15*t34*2.675853256136967*pow(10,-18)+t25*t31*4.353370830660928*pow(10,-2)+t25*t36*2.675853256136967*pow(10,-18);
	JacobBase[1][0]=t4*t5*4.706410108373544*pow(10,-3)+t9*t10*3.346780521510076*pow(10,-3)+t7*t13*3.346780521510076*pow(10,-3)+t15*t29*3.808705958072664*pow(10,-3)+t4*t52*3.306546357697854*pow(10,-18)+t25*t31*3.808705958072664*pow(10,-3)-t7*t56*2.351321854362918*pow(10,-18)-t10*t54*2.351321854362918*pow(10,-18)-t15*t60*2.675853256136967*pow(10,-18)-t25*t62*2.675853256136967*pow(10,-18);
	JacobBase[2][0]=t4*t19*4.706410108373544*pow(10,-3)-t7*t20*3.346780521510076e-3-t10*t24*3.346780521510076e-3-t15*t34*3.808705958072664*pow(10,-3)-t4*t52*5.379451369695426*pow(10,-2)+t7*t56*3.825387640672303*pow(10,-2)+t10*t54*3.825387640672303*pow(10,-2)+t15*t60*4.353370830660928*pow(10,-2)+t25*(t35-t40)*3.808705958072664*pow(10,-3)+t25*(t61-t72)*4.353370830660928*pow(10,-2);
	JacobBase[12][0]=-8.715574274765822*pow(10,-2);
	JacobBase[13][0]=9.961946980917455*pow(10,-1);
	JacobBase[14][0]=6.123233995736766*pow(10,-17);
	JacobBase[0][1]=-t18*t79-t43*t65;
	JacobBase[1][1]=-t18*t77+t65*t68;
	JacobBase[2][1]=-t43*t77-t68*t79;
	JacobBase[12][1]=t318;
	JacobBase[13][1]=t319;
	JacobBase[14][1]=t18;
	JacobBase[0][2]=-t43*t69-t18*(t37+t38+t39-t25*t36*4.37*pow(10,-2));
	JacobBase[1][2]=-t18*t81+t68*t69;
	JacobBase[2][2]=-t43*t81-t68*(t37+t38+t39-t80);
	JacobBase[12][2]=t318;
	JacobBase[13][2]=t319;
	JacobBase[14][2]=t18;
	JacobBase[0][3]=-t43*t75-t18*(t38-t25*t36*4.37*pow(10,-2));
	JacobBase[1][3]=-t18*t82+t68*t75;
	JacobBase[2][3]=-t43*t82-t68*(t38-t80);
	JacobBase[12][3]=t318;
	JacobBase[13][3]=t319;
	JacobBase[14][3]=t18;
	
	JacobBase[3][4]=t113+t116+t117+t118+t119+t149-t87*t100*2.351321854362918*pow(10,-18)-t90*t102*2.351321854362918*pow(10,-18)-t99*t107*2.675853256136967*pow(10,-18)-t103*t109*2.675853256136967*pow(10,-18);
	JacobBase[4][4]=t84*t85*3.306546357697854*pow(10,-18)+t89*t90*2.351321854362918*pow(10,-18)+t87*t93*2.351321854362918*pow(10,-18)+t99*t112*2.675853256136967*pow(10,-18)+t84*t128*3.306546357697854*pow(10,-18)-t87*t129*2.351321854362918*pow(10,-18)+t103*t115*2.675853256136967*pow(10,-18)-t90*t131*2.351321854362918*pow(10,-18)-t99*t136*2.675853256136967*pow(10,-18)-t103*t138*2.675853256136967*pow(10,-18);
	JacobBase[5][4]=t142+t143+t144+t146+t149-t87*t100*2.351321854362918*pow(10,-18)-t90*t102*2.351321854362918*pow(10,-18)-t99*t107*2.675853256136967*pow(10,-18)-t84*t128*((2.7*pow(10,1))/(5.0*pow(10,2)))-t103*t109*2.675853256136967*pow(10,-18);
	JacobBase[15][4]=-6.123233995736766*pow(10,-17);
	JacobBase[16][4]=1.0;
	JacobBase[17][4]=6.123233995736766*pow(10,-17);
	JacobBase[3][5]=-t95*t151-t125*t139;
	JacobBase[4][5]=-t95*t152+t139*t140;
	JacobBase[5][5]=-t125*t152-t140*t151;
	JacobBase[15][5]=t320;
	JacobBase[16][5]=t321;
	JacobBase[17][5]=t95;
	JacobBase[3][6]=-t95*t154-t125*t141;
	JacobBase[4][6]=-t95*t153+t140*t141;
	JacobBase[5][6]=-t125*t153-t140*t154;
	JacobBase[15][6]=t320;
	JacobBase[16][6]=t321;
	JacobBase[17][6]=t95;
	JacobBase[3][7]=-t95*t155-t125*t148;
	JacobBase[4][7]=-t95*t156+t140*t148;
	JacobBase[5][7]=-t125*t156-t140*t155;
	JacobBase[15][7]=t320;
	JacobBase[16][7]=t321;
	JacobBase[17][7]=t95;

	JacobBase[6][8]=t158*t159*5.379451369695426*pow(10,-2)+t163*t164*3.825387640672303*pow(10,-2)+t161*t167*3.825387640672303*pow(10,-2)+t158*t174*3.306546357697854*pow(10,-18)-t161*t175*2.351321854362918*pow(10,-18)-t164*t178*2.351321854362918*pow(10,-18)+t169*t183*4.353370830660928*pow(10,-2)-t169*t188*2.675853256136967*pow(10,-18)+t179*t185*4.353370830660928*pow(10,-2)-t179*t190*2.675853256136967*pow(10,-18);
	JacobBase[7][8]=t158*t159*(-4.706410108373538*pow(10,-3))-t163*t164*3.346780521510071*pow(10,-3)-t161*t167*3.346780521510071*pow(10,-3)-t169*t183*3.808705958072659*pow(10,-3)-t158*t206*3.306546357697854*pow(10,-18)-t179*t185*3.808705958072659*pow(10,-3)+t161*t208*2.351321854362918*pow(10,-18)+t164*t211*2.351321854362918*pow(10,-18)+t169*t218*2.675853256136967*pow(10,-18)+t179*t220*2.675853256136967*pow(10,-18);
	JacobBase[8][8]=t158*t174*(-4.706410108373538*pow(10,-3))+t161*t175*3.346780521510071*pow(10,-3)+t164*t178*3.346780521510071*pow(10,-3)+t169*t188*3.808705958072659*pow(10,-3)+t158*t206*5.379451369695426*pow(10,-2)-t161*t208*3.825387640672303*pow(10,-2)+t179*t190*3.808705958072659*pow(10,-3)-t164*t211*3.825387640672303*pow(10,-2)-t169*t218*4.353370830660928*pow(10,-2)-t179*t220*4.353370830660928*pow(10,-2);
	JacobBase[18][8]=8.715574274765811*pow(10,-2);
	JacobBase[19][8]=9.961946980917455*pow(10,-1);
	JacobBase[20][8]=6.123233995736766*pow(10,-17);
	JacobBase[6][9]=-t172*t234+t198*t215;
	JacobBase[7][9]=t172*t232+t215*t223;
	JacobBase[8][9]=-t198*t232-t223*t234;
	JacobBase[18][9]=t322;
	JacobBase[19][9]=t198;
	JacobBase[20][9]=t172;
	JacobBase[6][10]=-t172*t236+t198*t224;
	JacobBase[7][10]=t172*t235+t223*t224;
	JacobBase[8][10]=-t198*t235-t223*t236;
	JacobBase[18][10]=t322;
	JacobBase[19][10]=t198;
	JacobBase[20][10]=t172;
	JacobBase[6][11]=-t172*t238+t198*t230;
	JacobBase[7][11]=t172*t237+t223*t230;
	JacobBase[8][11]=-t198*t237-t223*t238;
	JacobBase[18][11]=t322;
	JacobBase[19][11]=t198;
	JacobBase[20][11]=t172;


	JacobBase[9][12]=t239*5.520416061851726*pow(10,-2)+t241*4.980973490458728*pow(10,-3)+t239*t244*5.120402001110331*pow(10,-2)-t243*t248*5.120402001110331*pow(10,-2)+t243*t255*3.147318457435699*pow(10,-18)+t244*t258*3.147318457435699*pow(10,-18)+t250*t262*5.907434559684051*pow(10,-2)+t250*t268*3.631077759471902*pow(10,-18)+t259*t265*5.907434559684051*pow(10,-2)-t259*t270*3.631077759471902*pow(10,-18);
	JacobBase[10][12]=t239*4.829738233584518*pow(10,-3)+t241*4.357787137382872*pow(10,-4)+t239*t244*4.479771277926094*pow(10,-3)-t243*t248*4.479771277926094*pow(10,-3)+t250*t262*5.168335544936126*pow(10,-3)+t259*t265*5.168335544936126*pow(10,-3)-t244*t285*3.147318457435699*pow(10,-18)+t243*t291*3.147318457435699*pow(10,-18)-t250*t296*3.631077759471902*pow(10,-18)-t259*t299*3.631077759471902*pow(10,-18);
	JacobBase[11][12]=t239*5.0*pow(10,-3)-t241*5.541503154379685*pow(10,-2)-t243*t255*4.479771277926094*pow(10,-3)-t244*t258*4.479771277926094*pow(10,-3)-t250*t268*5.168335544936126*pow(10,-3)-t244*t285*5.120402001110331*pow(10,-2)+t259*t270*5.168335544936126*pow(10,-3)+t243*t291*5.120402001110331*pow(10,-2)-t250*t296*5.907434559684051*pow(10,-2)-t259*t299*5.907434559684051*pow(10,-2);
	JacobBase[21][12]=8.715574274765811*pow(10,-2);
	JacobBase[22][12]=-9.961946980917455*pow(10,-1);
	JacobBase[23][12]=6.123233995736766*pow(10,-17);
	JacobBase[9][13]=-t239*t312+t258*t294;
	JacobBase[10][13]=t239*t309-t285*t294;
	JacobBase[11][13]=-t258*t309+t285*t312;
	JacobBase[21][13]=t284-t292;
	JacobBase[22][13]=-t256-t257;
	JacobBase[23][13]=-t239;
	JacobBase[9][14]=-t280*(t276+t277+t278-t243*t248*5.139961104911203*pow(10,-2))-t283*(t271+t272+t273-t259*t270*5.93*pow(10,-2));
	JacobBase[10][14]=t283*(t301+t303+t304-t243*t291*5.139961104911203*pow(10,-2))-t306*(t276+t277+t278-t300);
	JacobBase[11][14]=t306*(t271+t272+t273-t316)+t280*(t301+t303+t304-t314);
	JacobBase[21][14]=t323;
	JacobBase[22][14]=t280;
	JacobBase[23][14]=t324;
	JacobBase[9][15]=-t280*t307-t283*(t273-t259*t270*5.93*pow(10,-2));
	JacobBase[10][15]=t283*t317-t306*t307;
	JacobBase[11][15]=t280*t317+t306*(t273-t316);
	JacobBase[21][15]=t323;
	JacobBase[22][15]=t280;
	JacobBase[23][15]=t324;


}
    
void AllegroNodeImpedance::computeDesiredTorque() {
  // NOTE: here we just compute and set the desired_torque class member
  // variable. AllegroNode::updateWriteReadCAN() will send the desired torque to
  // the hand.
  
  //COMPUTE GRAVITY COMP TORQUE ALWAYS    
	/*float my_G[16];
	int my_index;
			pBHand->SetJointDesiredPosition(current_position_filtered); 
		 	pBHand->SetMotionType(eMotionType_GRAVITY_COMP);
		 	pBHand->SetJointPosition(current_position_filtered);
		         pBHand->UpdateControl((double) frame * ALLEGRO_CONTROL_TIME_INTERVAL);
		         pBHand->GetJointTorque(desired_torque);    
		     pBHand->CalculateGravity();    
		     for (int i = 0; i < 4; i++) {
		 		for (int j = 0; j < 4; j++) {
		 			my_index=(i+1)*4-4+j;
		 			my_G[my_index]=desired_torque[my_index];
		 			//my_G[my_index]=pBHand->_G[i][j];
		 			gravity_mine.data[my_index]=my_G[my_index];			
		 		}
		 	}	
		 	gravity_mine_pub.publish(gravity_mine);
	*/
	float my_G[16];
	int my_index;
	for (int i = 0; i < 16; i++) {
		my_G[i]=0;
	}	
	
	if (rot_received){ 		 		
		ComputeBaseJacobian();
		double Jtotal_Rot_Transp[16][24]={0};
		vector<double> ext_force={0.0,0.0,-9.81, 0.0 , 0.0, 0.0};
		double Jindex_base[6][4]; //=[Jt_base(1:3,1:4);Jt_base(13:15,1:4)];	
		double Jmiddle_base[6][4];
		double Jring_base[6][4];
		double Jthumb_base[6][4];		
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
			Jindex_base[i][j]=JacobBase[i][j];
			Jindex_base[i+3][j]=JacobBase[i+12][j];			
			Jmiddle_base[i][j]=JacobBase[i+3][j+4];
			Jmiddle_base[i+3][j]=JacobBase[i+15][j+4];			
			Jring_base[i][j]=JacobBase[i+6][j+8];
			Jring_base[i+3][j]=JacobBase[i+18][j+8];			
			Jthumb_base[i][j]=JacobBase[i+9][j+12];
			Jthumb_base[i+3][j]=JacobBase[i+21][j+12];
			}
		}
		// Rotate finger Jacobians individually
		double Jtotal_Rot[24][16]={0};
		double Jindex_Rot[6][4]; //=[Jt_base(1:3,1:4);Jt_base(13:15,1:4)];	
		double Jmiddle_Rot[6][4];
		double Jring_Rot[6][4];
		double Jthumb_Rot[6][4];			
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
			Jindex_Rot[i][j]=RotBaseMine[i][0]*Jindex_base[0][j]+RotBaseMine[i][1]*Jindex_base[1][j]+RotBaseMine[i][2]*Jindex_base[2][j];
			Jindex_Rot[i+3][j]=RotBaseMine[i][0]*Jindex_base[3][j]+RotBaseMine[i][1]*Jindex_base[4][j]+RotBaseMine[i][2]*Jindex_base[5][j];	
			
			Jmiddle_Rot[i][j]=RotBaseMine[i][0]*Jmiddle_base[0][j]+RotBaseMine[i][1]*Jmiddle_base[1][j]+RotBaseMine[i][2]*Jmiddle_base[2][j];
			Jmiddle_Rot[i+3][j]=RotBaseMine[i][0]*Jmiddle_base[3][j]+RotBaseMine[i][1]*Jmiddle_base[4][j]+RotBaseMine[i][2]*Jmiddle_base[5][j];		
			
			Jring_Rot[i][j]=RotBaseMine[i][0]*Jring_base[0][j]+RotBaseMine[i][1]*Jring_base[1][j]+RotBaseMine[i][2]*Jring_base[2][j];
			Jring_Rot[i+3][j]=RotBaseMine[i][0]*Jring_base[3][j]+RotBaseMine[i][1]*Jring_base[4][j]+RotBaseMine[i][2]*Jring_base[5][j];		
			
			Jthumb_Rot[i][j]=RotBaseMine[i][0]*Jthumb_base[0][j]+RotBaseMine[i][1]*Jthumb_base[1][j]+RotBaseMine[i][2]*Jthumb_base[2][j];
			Jthumb_Rot[i+3][j]=RotBaseMine[i][0]*Jthumb_base[3][j]+RotBaseMine[i][1]*Jthumb_base[4][j]+RotBaseMine[i][2]*Jthumb_base[5][j];			
			}
		}		
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				Jtotal_Rot[i][j]=Jindex_Rot[i][j];
				Jtotal_Rot[i+12][j]=Jindex_Rot[i+3][j];			
				Jtotal_Rot[i+3][j+4]=Jmiddle_Rot[i][j];
				Jtotal_Rot[i+15][j+4]=Jmiddle_Rot[i+3][j];			
				Jtotal_Rot[i+6][j+8]=Jring_Rot[i][j];
				Jtotal_Rot[i+18][j+8]=Jring_Rot[i+3][j];			
				Jtotal_Rot[i+9][j+12]=Jthumb_Rot[i][j];
				Jtotal_Rot[i+21][j+12]=Jthumb_Rot[i+3][j];
			}
		}
		for (int i = 0; i < 16; i++) {
			for (int j = 0; j < 24; j++) {
				Jtotal_Rot_Transp[i][j]=Jtotal_Rot[j][i];
			}
		}
		
		for (int i = 0; i <4; i++) {
			//Index
			my_G[i]=Jtotal_Rot_Transp[i][0]*ext_force[0]+Jtotal_Rot_Transp[i][1]*ext_force[1]+Jtotal_Rot_Transp[i][2]*ext_force[2]+Jtotal_Rot_Transp[i][12]*ext_force[3]+Jtotal_Rot_Transp[i][13]*ext_force[4]+Jtotal_Rot_Transp[i][14]*ext_force[5];
			//Middle
			my_G[i+4]=Jtotal_Rot_Transp[i+4][3]*ext_force[0]+Jtotal_Rot_Transp[i+4][4]*ext_force[1]+Jtotal_Rot_Transp[i+4][5]*ext_force[2]+Jtotal_Rot_Transp[i+4][15]*ext_force[3]+Jtotal_Rot_Transp[i+4][16]*ext_force[4]+Jtotal_Rot_Transp[i+4][17]*ext_force[5];
			//Ring
			my_G[i+8]=Jtotal_Rot_Transp[i+8][6]*ext_force[0]+Jtotal_Rot_Transp[i+8][7]*ext_force[1]+Jtotal_Rot_Transp[i+8][8]*ext_force[2]+Jtotal_Rot_Transp[i+8][18]*ext_force[3]+Jtotal_Rot_Transp[i+8][19]*ext_force[4]+Jtotal_Rot_Transp[i+8][20]*ext_force[5];
			//Thumb
			my_G[i+12]=Jtotal_Rot_Transp[i+12][9]*ext_force[0]+Jtotal_Rot_Transp[i+12][10]*ext_force[1]+Jtotal_Rot_Transp[i+12][11]*ext_force[2]+Jtotal_Rot_Transp[i+12][21]*ext_force[3]+Jtotal_Rot_Transp[i+12][22]*ext_force[4]+Jtotal_Rot_Transp[i+12][23]*ext_force[5];
			
		}				
	}
	/*for (int i = 0; i <16; i++) {
	cout << my_G[i];
	  cout << " ";
   }*/
   //cout <<  "\n";
	/*else{
		for (int i = 0; i < 16; i++) {
			for (int j = 0; j < 24; j++) {
				Jtotal_Rot_Transp[i][j]=JacobBase[j][i];
			}
		}
	}*/
		// Compute Gravity cmp torque
			
		
/*	if (rot_received){  
		vector<double> my_G_balak;	
		my_G_balak.resize(16,0.0);		
		my_G_balak=myGrav(current_position_filtered);
		float new_my_G[16]; // convert from vector double to array of floats 
			for (int i =0;i<4;i++)
		{
		  new_my_G[i]=my_G_balak[i];
		}
		int my_index;
		float my_G[16];
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				my_index=(i+1)*4-4+j;
				my_G[my_index]=new_my_G[my_index];
				//my_G[my_index]=pBHand->_G[i][j];
				gravity_mine.data[my_index]=my_G[my_index];	
						
			}
		}	
		gravity_mine_pub.publish(gravity_mine);
	}*/
	
  // No control: set torques to zero./GRAVITY COMP
  if (((!control_hand_) && (!imp_received)) || ((control_hand_) && (imp_received))) {
     //ROS_INFO_THROTTLE(1.0, "Hand control is false");
    for (int i = 0; i < DOF_JOINTS; i++) {
      //desired_torque[i] = 0.0;
      desired_torque[i]= my_G[i];    
	  //cout << my_G[i];
	  //cout << " ";
    }
    //cout <<  "\n";     
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
	  // Control joint positions: compute the desired torques (PD control).
      double error;
      for (int i = 0; i < DOF_JOINTS; i++) {
        error = q_des[i] - current_position_filtered[i];
        /*desired_torque[i] = 1.0/canDevice->torqueConversion() *
                (k_p[i] * error - k_d[i] * current_velocity_filtered[i])+my_G[i];*/
                desired_torque[i]= my_G[i]; 
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



/*vector<double> allegroKDL::get_G(vector<double> q)
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
}*/


vector<double> myGrav (double* q)
{
//  //vector<double> g_vec={0.0,-18.0,0.0}; // in-grasp box 
//  //vector<double> g_vec={0.0,0.0,-18.0};// upright
//  vector<double> g_vec={18.6,0.0,0.0}; // in-grasp box 
 
// vector<double> g_vec={0.0,0.0,-9.81}; // in-grasp box 
 vector<double> g_vec={0.0,0.0,-9.81}; // in-grasp box 
 
//  float TransfEE[16];
//  TransfEE=franka_states.O_T_EE;
//  g_vec[0]=TransfEE[0]*g_vec[0]+TransfEE[1]*g_vec[1]+TransfEE[2]*g_vec[2];
//  g_vec[1]=TransfEE[4]*g_vec[0]+TransfEE[5]*g_vec[1]+TransfEE[6]*g_vec[2];
//  g_vec[2]=TransfEE[8]*g_vec[0]+TransfEE[9]*g_vec[1]+TransfEE[10]*g_vec[2];
 
/* allegroKDL kdl_comp(g_vec);
 vector<double> tau_g;
 tau_g.resize(16,0.0);
 
 std::vector<double> q_vector(q, q + 16);
 tau_g=kdl_comp.get_G(q_vector);*/
 
 
//return tau_g;
 
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
