Apart from adding this package you have to:

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
