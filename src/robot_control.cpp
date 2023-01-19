#include "robot_control.hpp"
#include "fstream"

double mapping = 0.15;
double pre_pos[3], cur_pos[3], euler_angles[3];
ElfinModel elfin;

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

  Eigen::Matrix3d RotMat = RotMatFromEuler(state->cur_gimbal_angles[0], state->cur_gimbal_angles[2], state->cur_gimbal_angles[1]);                         
  pos_error << mapping * (state->position[0] - state->pre_position[0]), mapping * (state->position[2] - state->pre_position[2]), mapping * (state->position[1] - state->pre_position[1]);
  pos_error = RotMat * pos_error;
  rot_err << mapping * (state-> cur_gimbal_angles[0] - state->pre_gimbal_angles[0]), mapping * (state-> cur_gimbal_angles[1] - state->pre_gimbal_angles[1]), mapping * (state-> cur_gimbal_angles[2] - state->pre_gimbal_angles[2]);
  rot_err = RotMat * rot_err;
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
    roboconnect.SetSpeedRadio(0.05);

    // Move to predefined initial position
    Eigen::Matrix<double,1,6> target_joints;    // initial position
    target_joints << 19.284*M_PI/180, 28.371*M_PI/180, -53.553*M_PI/180, -143.202*M_PI/180, 52.791*M_PI/180, 86.136*M_PI/180;
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
        usleep(17000);

    }

}


