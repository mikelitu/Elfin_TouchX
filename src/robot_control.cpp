#include "robot_control.hpp"
#include "fstream"

double mapping = 0.25;
double pre_pos[3], cur_pos[3];
ElfinModel elfin;

/**
 * @brief 
 * 
 * @param cur_joints 
 * @param next_joint 
 * @param state 
 * @param next_kinematics 
 * @param rot_err 
 */

void RobotControl::Touch2Elfin(Eigen::Matrix<double,1,6>& cur_joints, Eigen::Matrix<double,1,6>& next_joints, 
                              OmniState *state, Eigen::Matrix<double,3,1> pos_error, Eigen::Matrix<double,3,1>& rot_err) {
  
  // cur_pos[0] = state->position[0];
  // cur_pos[1] = state->position[1];
  // cur_pos[2] = state->position[2];
  
  // std::cout << geo_pos << std::endl;
  
  // pre_pos[0] = state->pre_position[0];
  // pre_pos[1] = state->pre_position[1];
  // pre_pos[2] = state->pre_position[2];

  pos_error << mapping * (state->position[0] - state->pre_position[0]), mapping * (state->position[2] - state->pre_position[2]), -mapping * (state->position[1] - state->pre_position[1]);
  elfin.GetNextJoints(next_joints, cur_joints, pos_error, rot_err);

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
    Eigen::Matrix<double,3,1> rot_err, pos_error;

    targetdiff.fill(0);
    pos_error.fill(0);
    rot_err.fill(0);

    while (true)
    {

        roboconnect.ReadCurrentJoint(cur_joints);
        // std::cout << cur_joints(0) << ", " << cur_joints(1) << ", " << cur_joints(2) << ", " << cur_joints(3) << ", " << cur_joints(4) << ", " << cur_joints(5) << std::endl;
        Touch2Elfin(cur_joints, next_joints, state, pos_error, rot_err);
        // std::cout << next_joints(0) * 180/M_PI << ", " << next_joints(1) * 180/M_PI << ", " << next_joints(2) * 180/M_PI << ", " << next_joints(3) * 180/M_PI << ", " << next_joints(4) * 180/M_PI << ", " << next_joints(5) * 180/M_PI << ", " << std::endl;
        roboconnect.PushServoJ(next_joints);
        usleep(17000);

    }

}


