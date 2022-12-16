#include "robot_control.hpp"

float mapping = 0.2;
double geo_pos[3];

/**
 * @brief 
 * 
 * @param cur_joints 
 * @param next_joint 
 * @param state 
 * @param next_kinematics 
 * @param rot_err 
 */

void RobotControl::Touch2Elfin(Eigen::Matrix<double,1,6>& cur_joints, Eigen::Matrix<double,1,6>& next_joint, 
                              OmniState *state, Eigen::Matrix4d& next_kinematics, Eigen::Matrix<double,3,1>& rot_err) {
  
  geo_pos[0] = state->position[0];
  geo_pos[1] = state->position[1];
  geo_pos[2] = state->position[2];
  
  // std::cout << geo_pos << std::endl;



}

/**
 * @brief 
 * 
 * @param state {OmniState}
 */
void RobotControl::GeomagicControl(OmniState *state)
{
    
    DcsCommand roboconnect;
    // ElfinModel elfin;
    std::cout<<"conect ro"<<std::endl;
    roboconnect.ConnectTCPSocket();
    roboconnect.SetSpeedRadio(0.05);

    Eigen::Matrix<double,1,6> target_joints;    // initial position
    target_joints << 13.720*M_PI/180, 31.219*M_PI/180, -45.2*M_PI/180, -130.435*M_PI/180, 51.593*M_PI/180, 56.404*M_PI/180;
    roboconnect.MoveJ(target_joints);
    usleep(10000);
    while(roboconnect.isMoving()) usleep(1e6);

    // start servo
    double servo_time = 0.012;
    double lookahead_time = 0.2;
    roboconnect.StartServo(servo_time,lookahead_time);

    Eigen::Matrix<double,1,6> cur_joints, next_joints;
    Eigen::Matrix<double,6,1> targetdiff;
    Eigen::Matrix<double,3,1> rot_err;
    Eigen::Matrix4d next_kinematics;

    targetdiff.fill(0);
    next_kinematics.fill(0);
    rot_err.fill(0);

    while (true)
    {

        roboconnect.ReadCurrentJoint(cur_joints);
        // std::cout << cur_joints(0) << ", " << cur_joints(1) << ", " << cur_joints(2) << ", " << cur_joints(3) << ", " << cur_joints(4) << ", " << cur_joints(5) << std::endl;
        Touch2Elfin(cur_joints, next_joints, state, next_kinematics, rot_err);
        // roboconnect.PushServoJ(next_joints);
        usleep(20000);

    }

}


