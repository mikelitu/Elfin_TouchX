#include <thread> 
#include <iostream>
#include <algorithm>
#include "image_consumer.hpp"
#include "force_sensor.hpp"
#include "robot_control.hpp"
#include "Eigen/Dense"
#include <filesystem>
#include <signal.h>

float sensor[6]; // sensor values
float save_sensor[6]; //transformed sensor values
Eigen::Matrix<double,1,6> cur_joints; // current joints value
Eigen::Matrix<double,1,6> GravityAndError; // gravity compesentation vector
Eigen::Matrix<double,1,6> CenterOfTool; // center of mass of the tool
Eigen::Matrix4d cur_kinematics; // the value of the current kinematics
Eigen::Matrix3d rot2tool;
Eigen::Matrix<double,3,1> trans2tool, postool, trans_force, trans_torque;
int calibrationStyle; // calibration style for the omni device
int width = 640; // camera image width
int height = 480; // camera image height
int fps = 30; // fps of the video
bool init_exp = false; // flag to intialize the experiment
std::stringstream state_stream; // stream to save the robot state
std::string save_dir = "/home/mikel/experiments"; // home folder for the experiments data
std::fstream state_file; // file stream to save the state data
namespace fs = std::filesystem;
typedef std::array<double, 3> double3;
double3 euler;


/**
 * @brief Callback function for the TouchX device
 * 
 * @param pUserData Data structure to cast into the state structure
 * @return HDCallbackCode 
 */

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) {
    OmniState *omni_state = static_cast<OmniState *>(pUserData);
    // if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    //    hdUpdateCalibration(calibrationStyle);
    // }
    hdBeginFrame(hdGetCurrentDevice());
    // Get transform angles
    hduMatrix cur_transform, pre_transform;
    hdGetDoublev(HD_CURRENT_TRANSFORM, cur_transform);
    hdGetDoublev(HD_LAST_TRANSFORM, pre_transform);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);

    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->cur_gimbal_angles);
    hdGetDoublev(HD_LAST_GIMBAL_ANGLES, omni_state->pre_gimbal_angles);
    // Notice that we are inverting the Z-position value and changing Y <----> Z
    // Position
    omni_state->pre_position = hduVector3Dd(pre_transform[3][0], -pre_transform[3][2], pre_transform[3][1]);
    omni_state->position = hduVector3Dd(cur_transform[3][0], -cur_transform[3][2], cur_transform[3][1]);
    // omni_state->position /= omni_state->units_ratio;
    // Orientation (quaternion)
    hduMatrix cur_rotation(cur_transform);
    hduMatrix pre_rotation(pre_transform);
    cur_rotation.getRotationMatrix(cur_rotation);
    pre_rotation.getRotationMatrix(pre_rotation);
    hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                               1.0,  0.0, 0.0, 0.0,
                               0.0,  0.0, 1.0, 0.0,
                               0.0,  0.0, 0.0, 1.0);
    rotation_offset.getRotationMatrix(rotation_offset);
    omni_state->rot = hduQuaternion(rotation_offset * cur_rotation);
    omni_state->pre_rot = hduQuaternion(rotation_offset * pre_rotation);
    // Velocity estimation
    hduVector3Dd vel_buff(0, 0, 0);
    vel_buff = (omni_state->position * 3 -4 * omni_state->pos_hist1
            + omni_state->pos_hist2) / 0.002; //(units)/s, 2nd order backward dif
    omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
            + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
            - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
            -0.7776 * omni_state->out_vel3);
    omni_state->pos_hist2 = omni_state->pos_hist1;
    omni_state->pos_hist1 = omni_state->position;
    omni_state->inp_vel3 = omni_state->inp_vel2;
    omni_state->inp_vel2 = omni_state->inp_vel1;
    omni_state->inp_vel1 = vel_buff;
    omni_state->out_vel3 = omni_state->out_vel2;
    omni_state->out_vel2 = omni_state->out_vel1;
    omni_state->out_vel1 = omni_state->velocity;

    hduVector3Dd feedback;
    // Notice we are changing Y <----> Z and inverting the Z-force_feedback
    feedback[0] = omni_state->force[0];
    feedback[1] = omni_state->force[2];
    feedback[2] = omni_state->force[1];
    hdSetDoublev(HD_CURRENT_FORCE, feedback);

    int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

    hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Error during main scheduler callback");
        if (hduIsSchedulerError(&error))
            return HD_CALLBACK_DONE;
    }

    return HD_CALLBACK_CONTINUE;
}

/**
 * @brief Calibration function for the TouchX
 * 
 */
void HHD_Auto_Calibration() {
    int supportedCalibrationStyles;
    HDErrorInfo error;

    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    std::cout << "HD_CALIBRATION_ENCODER_RESET..." << std::endl;
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    std::cout << "HD_CALIBRATION_INKWELL..." << std::endl;
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    std::cout << "HD_CALIBRATION_AUTO..." << std::endl;
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      std::cout << "Calibrating... (put stylus in well)" << std::endl;
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    std::cout << "Calibration complete." << std::endl;
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      std::cout << "Please place the device into the inkwell for calibration" << std::endl;
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      std::cout << "Calibration updated successfully" << std::endl;
      hdUpdateCalibration(calibrationStyle);
    }
    else
        std::cout << "Unkown calibration status" << std::endl;
  }

}

/**
 * @brief Gravity compensation of the sensor
 * 
 * @param R Current rotation matrix of the robot end effector
 * @param sensor Raw values of the sensor
 */

void ForceTorqueError(Eigen::Matrix<double,3,3>& R, float sensor[6])
{
    Eigen::Matrix<double,3,1> GravityinSensor;
    Eigen::Matrix<double,3,1> lastGravity;
    Eigen::Matrix<double,3,1> initGravity;
    Eigen::Matrix<double,3,1> MomentOfGravity;
    Eigen::Matrix<double,3,1> MomentOfTool;
    Eigen::Matrix<double,3,1> momentError;
    Eigen::Matrix<double,3,1> forceError;
    Eigen::Matrix<double,3,1> forceNormed;
    Eigen::Matrix<double,3,1> torqueNormed;

    lastGravity << GravityAndError(3), GravityAndError(4), GravityAndError(5);
    initGravity << GravityAndError(0), GravityAndError(1), GravityAndError(2);
    GravityinSensor = R.transpose().inverse() * initGravity;
    forceError = GravityinSensor + lastGravity;
    
    MomentOfGravity << GravityinSensor(2)*CenterOfTool(1) - GravityinSensor(1)*CenterOfTool(2), 
                    GravityinSensor(0)*CenterOfTool(2) - GravityinSensor(2)*CenterOfTool(0),
                    GravityinSensor(1)*CenterOfTool(0) - GravityinSensor(0)*CenterOfTool(1);
    MomentOfTool << CenterOfTool(3) - GravityAndError(4)*CenterOfTool(2) + GravityAndError(5)*CenterOfTool(1), 
                    CenterOfTool(4) - GravityAndError(5)*CenterOfTool(0) + GravityAndError(3)*CenterOfTool(2), 
                    CenterOfTool(5) - GravityAndError(3)*CenterOfTool(1) + GravityAndError(4)*CenterOfTool(0);
    momentError = MomentOfGravity + MomentOfTool;
    
    for (int i=0; i<6; i++)
    {
        float cur_read = sensor[i];
        if (i < 3) {
            sensor[i] = cur_read - (float) forceError(i);
        } else {
            sensor[i]=cur_read - (float) momentError(i);
        }
    }

}

double3 Rot2Euler(Eigen::Matrix3d& R) {
  return {
    -atan2(R(1,0), R(0,0)),
    asin(R(2,0)),
    atan2(R(2,1), R(2,2))
  };
}

void TransformState(Eigen::Matrix3d& R) {
  
  euler = Rot2Euler(R);
  postool << cur_kinematics(0,3) + trans2tool(0),
              cur_kinematics(1,3) + trans2tool(1),
              cur_kinematics(2,3) + trans2tool(2);

  postool = R * postool;   
}

/**
 * @brief Update the state stream
 * 
 * @param state TouchX current state information
 * @param init_exp Flag that states if the robot has reached the initial position
 * @param filename Name of the labels file
 */
void savestate(OmniState *state, bool& init_exp, std::string& filename)
{
  state_file.open(filename, std::ios::app);

  rot2tool.fill(0);
  rot2tool(0,2) = 1;
  rot2tool(1,0) = 1;
  rot2tool(2,1) = 1;
  trans2tool << 0, 0.34, -0.1;
  trans_force.fill(0);
  trans_torque.fill(0);

  while (true) {
    std::cout << init_exp << std::endl;
    if (init_exp) {

      Eigen::Matrix3d R = cur_kinematics.block(0,0,3,3);
      
      TransformState(R);
      trans_force << sensor[0], sensor[1], sensor[2];
      trans_torque << sensor[3], sensor[4], sensor[5];

      trans_force = R * trans_force;
      trans_torque = R * trans_torque;

      state_stream << postool(0) << "," << postool(1) << "," << postool(2) << "," << euler[0] << "," << euler[1] << "," << euler[2]
                  << "," << cur_joints(0) << "," << cur_joints(1) << "," << cur_joints(2) << "," << cur_joints(3) << "," << cur_joints(4) << ","
                  << cur_joints(5) << "," << state->position[0] << "," << state->position[1] << "," << state->position[2] << "," << state->cur_gimbal_angles[0] << ","
                  << state->cur_gimbal_angles[1] << "," << state->cur_gimbal_angles[2] << ","<< "," << state->joints[0] << "," << state->joints[1] << "," << state->joints[2] << ","
                  << state->joints[3] << "," << state->joints[4] << "," << state->joints[5] << "," << trans_force(0) << "," << trans_force(1) << "," << trans_force(2) << "," 
                  << trans_torque(0) << "," << trans_torque(1) << "," << trans_torque(2) << "\n";

      // Force feedback
      // state->force[0] = 0.05 * trans_force(0);
      // state->force[1] = 0.05 * trans_force(1);
      // state->force[2] = 0.05 * trans_force(2);

      usleep(5000);
    }
  }        
}

/**
 * @brief Update the state and sensor vectors
 */
void readstate()
{
    CLinuxSerial forcesensor(0,115200);
    Eigen::Matrix<double,3,3> R;
    ElfinModel elfin;
    GravityAndError << 1.22875977, 0.12287842, 2.23705746, 4.60417579, 4.5727096,  7.96071791;
    CenterOfTool << 0.02501844, -0.00661609,  0.05771841,  0.18731095,  0.02821454, -0.1544118;
    
    while (true)
    {
        forcesensor.Sensor();
        for (int i = 0; i < 6; i++)
        {
            sensor[i]=forcesensor.sensor[i];
        }
        
        elfin.GetKinematics(cur_kinematics, cur_joints);
        R = cur_kinematics.block(0,0,3,3);
        ForceTorqueError(R, sensor);
        
        usleep(5000);

    }
}

/**
 * @brief Function to save the state stream and kill the program
 * 
 * @param sig Signal type
 */
void end_experiment(int sig) {
  std::cout << "Caugth signal" <<std::endl;
  state_file << state_stream.str();
  state_file.close();
  exit(0);
}


int main(int argc, char *argv[])
{   
    /////////////////////
    /// Init Touch X ///
    ///////////////////
    HDErrorInfo error;
    HHD hHD;
    hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
      hduPrintError(stderr, &error, "Failed to initialized haptic device");
    }

    // Enable force feedback
    hdEnable(HD_FORCE_OUTPUT);

    //Start the scheduler
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
    }

    // HHD_Auto_Calibration();
    // Define the callback loop
    OmniState state;
    hdScheduleAsynchronous(omni_state_callback, &state,
        HD_MAX_SCHEDULER_PRIORITY);

    // Call dependencies from other header files
    ImageConsumer image_consumer;
    RobotControl roboctr;

    // Create a file to write the labels for the experiment
    
    if (argc < 2) {
      std::string exp_name = "last_data";
      save_dir += "/" + exp_name;
    } else {
      std::string exp_name = argv[1];
      save_dir += "/" + exp_name;
    }
    
    if (!fs::exists(save_dir)) {
        fs::create_directories(save_dir);
    }
    
    // Initialize the file to save the state afterwards
    std::string filename = save_dir + "/labels.csv";
    state_file.open(filename, std::ios::out);
    state_file << "Pos_Rob_X,Pos_Rob_Y,Pos_Rob_Z,Or_Rob_X,Or_Rob_Y,Or_Rob_Z,J_Rob_1,J_Rob_2,J_Rob_3,J_Rob_4,J_Rob_5,J_Rob_6,Pos_Hap_X,Pos_Hap_Y,Pos_Hap_Z,Or_Hap_X,Or_Hap_Y,Or_Hap_Z,Or_Hap_W,J_Hap_1,J_Hap_2,J_Hap_3,J_Hap_4,J_Hap_5,J_Hap_6,F_X,F_Y,F_Z,T_X,T_Y,T_Z\n";
    state_file.close();

    signal(SIGINT, end_experiment); // Function to catch the Ctr+C command and save the data stream into the csv file

    // Start all the threads corresponding to the different parts of the control loop
    std::thread task0(&ImageConsumer::ImagePipeline, image_consumer, width, height, fps, std::ref(init_exp), std::ref(save_dir)); // Image saving pipeline
    std::thread task1(&ImageConsumer::ImageWindow, image_consumer); // CV imshow  
    std::thread task2(readstate); // Force sensor and state update
    std::thread task4(savestate, &state, std::ref(init_exp), std::ref(filename)); // State string stream update
    std::thread task3(&RobotControl::GeomagicControl,roboctr, &state, std::ref(cur_joints), std::ref(init_exp)); // Robot teleoperation

    // Join to the different threads
    task0.join();
    task1.join();
    task2.join();
    task3.join();
    task4.join();

    hdStopScheduler();
    hdDisableDevice(hHD);

    return 0;
    
}

