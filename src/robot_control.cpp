#include "robot_control.hpp"
#include "fstream"

double pre_pos[3], cur_pos[3], euler_angles[3];
ElfinModel elfin;
Eigen::Matrix3d lastrot;
Eigen::Vector3d lasttrans;

Eigen::Matrix3d RotMatFromEuler(double pitch, double yaw, double roll) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    return q.matrix();
}

/**
 * @brief Function to map the twists from the TouchX to the Elfin arm
 * 
 * @param cur_joints Current joints position of the Elfin robot
 * @param next_joint Next joint position of the Elfin
 * @param state State of the TouchX device
 * @param pos_error Position difference between the previous and current TouchX state
 * @param rot_err Orientation difference between the previous and current TouchX state
 */

void RobotControl::Touch2Elfin(Eigen::Matrix<double,1,6>& cur_joints, Eigen::Matrix<double,1,6>& next_joints, 
                              OmniState *state, Eigen::Matrix<double,3,1> pos_error, Eigen::Matrix<double,3,1>& rot_err) {
  Eigen::Vector3d hap_pos_twist, hap_or_twist;
  hap_pos_twist << state->position[0] - state->pre_position[0], state->position[1]- state->pre_position[1], state->position[2] - state->pre_position[2];
  // std::cout << "Position of haptic device: " << state->position[0] << ", " << state->position[1] << ", " << state->position[2] << std::endl;

  pos_error = 85.0 * (lastrot.inverse() * hap_pos_twist);

  // std::cout << "Position of the robot: " << pos_error(0) << ", " << pos_error(1) << ", " << pos_error(2) << std::endl;   
  
  hap_or_twist << state-> cur_gimbal_angles[0] - state->pre_gimbal_angles[0], state-> cur_gimbal_angles[1] - state->pre_gimbal_angles[1], state-> cur_gimbal_angles[2] - state->pre_gimbal_angles[2];
  rot_err = (lastrot.inverse() * hap_or_twist);
  elfin.GetNextJoints(next_joints, cur_joints, pos_error, rot_err);

}

/**
 * @brief Function to teleoperate the robot with the TouchX haptic device
 * 
 * @param state {OmniState} State of the Touch X device
 * @param cur_joints Pointer to the current joint position of the robot
 */
void RobotControl::GeomagicControl(OmniState *state, Eigen::Matrix<double,1,6>& cur_joints, bool& init_exp)
{
    // Initialize the robot and set initial velocity
    DcsCommand roboconnect;

    std::cout<<"conect ro"<<std::endl;
    roboconnect.ConnectTCPSocket();
    roboconnect.SetSpeedRadio(0.075);

    // Move to predefined initial position
    Eigen::Matrix<double,1,6> target_joints;    // initial position
    target_joints << 25.451*M_PI/180, 43.550*M_PI/180, -36.059*M_PI/180, -147.770*M_PI/180, 43.239*M_PI/180, 94.790*M_PI/180;
    roboconnect.MoveJ(target_joints);
    usleep(20000);
    while(roboconnect.isMoving()) usleep(1e6);
    

    // start servo
    double servo_time = 0.012;
    double lookahead_time = 0.2;
    roboconnect.StartServo(servo_time,lookahead_time);

    // Define variables and set initial values
    Eigen::Matrix<double,1,6> next_joints;
    Eigen::Matrix<double,3,1> rot_err, pos_error;
    pos_error.fill(0);
    rot_err.fill(0);
    lastrot.fill(0);
    lastrot(0,1) = -1;
    lastrot(1,0) = 1;
    lastrot(2,2) = 1;
    init_exp = true;

    /**
     * @brief While loop to teleoperate the robot using position mapping with the Touch X haptic device.
     * Steps:
     *  1. Get the current joint position of the robot
     *  2. Compute the twist of the TouchX device and assign it to the pos_error variable
     *  3. Use the method GetNextJoints from robot_kinematics.hpp to get the next_joints position
     *  4. Push the robot arm to the new position
     *  5. Sleep for a smoother movement
     */
    while (true)
    {

        roboconnect.ReadCurrentJoint(cur_joints);
        Touch2Elfin(cur_joints, next_joints, state, pos_error, rot_err);
        roboconnect.PushServoJ(next_joints);

        // std::cout << next_joints[0] << next_joints[1] << next_joints[2] << next_joints[3] << next_joints[4] << next_joints[5] << std::endl;
        usleep(17700);

    }

}


