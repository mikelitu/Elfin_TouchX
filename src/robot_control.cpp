#include "robot_control.hpp"
#include "fstream"

double mapping = 0.25;
double pre_pos[3], cur_pos[3], euler_angles[3];
ElfinModel elfin;

void EulerfromQuaternion(double x, double y, double z, double w) {
    double t0 = 2.0 * (w * x + y * z);
    double t1 = 1.0 -2.0 * (x * x + y * y);
    double roll_x = atan2(t0, t1);

    double t2 = 2.0 * (w * y - z * w);
    if (t2 > 1.0) {
        t2 = 1.0;
    } else if (t2 < -1.0)
    {
        t2 = -1.0;
    }
    double pitch_y = asin(t2);

    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (y * y + z * z);
    double yaw_z = atan2(t3, t4);

    euler_angles[0] = roll_x;
    euler_angles[1] = pitch_y;
    euler_angles[2] = yaw_z;
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

  pos_error << mapping * (state->position[0] - state->pre_position[0]), mapping * (state->position[2] - state->pre_position[2]), -mapping * (state->position[1] - state->pre_position[1]);
  
  elfin.GetNextJoints(next_joints, cur_joints, pos_error, rot_err);

}

/**
 * @brief Function to teleoperate the robot with the TouchX haptic device
 * 
 * @param state {OmniState} State of the Touch X device
 * @param cur_joints Pointer to the current joint position of the robot
 */
void RobotControl::GeomagicControl(OmniState *state, Eigen::Matrix<double,1,6>& cur_joints)
{
    // Initialize the robot and set initial velocity
    DcsCommand roboconnect;

    std::cout<<"conect ro"<<std::endl;
    roboconnect.ConnectTCPSocket();
    roboconnect.SetSpeedRadio(0.05);

    // Move to predefined initial position
    Eigen::Matrix<double,1,6> target_joints;    // initial position
    target_joints << 13.720*M_PI/180, 31.219*M_PI/180, -45.2*M_PI/180, -130.435*M_PI/180, 51.593*M_PI/180, 56.404*M_PI/180;
    roboconnect.MoveJ(target_joints);
    usleep(10000);
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
        usleep(17000);

    }

}


