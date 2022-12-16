#include <thread> 
#include "opencv2/highgui.hpp"
#include <iostream>
#include "image_consumer.hpp"
#include "force_sensor.hpp"
#include "robot_control.hpp"
#include "Eigen/Dense"

float sensor[6];
Eigen::Matrix<double,1,6> cur_joints;
Eigen::Matrix<double,1,6> GravityAndError;
Eigen::Matrix<double,1,6> CenterOfTool;
int calibrationStyle;
float prev_time;
float ft_normalized[6];

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) {
    OmniState *omni_state = static_cast<OmniState *>(pUserData);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
        hdUpdateCalibration(calibrationStyle);
    }
    hdBeginFrame(hdGetCurrentDevice());
    // Get transform angles
    hduMatrix transform;
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
    hduVector3Dd gimbal_angles;
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
    // Notice that we are inverting the Z-position value and changing Y <----> Z
    // Position
    omni_state->position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
    // omni_state->position /= omni_state->units_ratio;
    // Orientation (quaternion)
    hduMatrix rotation(transform);
    rotation.getRotationMatrix(rotation);
    hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                               1.0,  0.0, 0.0, 0.0,
                               0.0,  0.0, 1.0, 0.0,
                               0.0,  0.0, 0.0, 1.0);
    rotation_offset.getRotationMatrix(rotation_offset);
    omni_state->rot = hduQuaternion(rotation_offset * rotation);
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
    // Notice that we are changing Y <---> Z and inverting the Z-force_feeback
    feedback[0] = omni_state->force[0];
    feedback[1] = omni_state->force[1];
    feedback[2] = omni_state->force[2];
    hdSetDoublev(HD_CURRENT_FORCE, feedback);

    // Get buttons
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

    float t[7] = {0., omni_state->joints[0], omni_state->joints[1],
                omni_state->joints[2] - omni_state->joints[1], gimbal_angles[0],
                gimbal_angles[1], gimbal_angles[2]};
    for (int i = 0; i < 7; i++)
        omni_state->thetas[i] = t[i];
    return HD_CALLBACK_CONTINUE;
}

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
            sensor[i] = cur_read + (float) forceError(i);
        } else {
            sensor[i]=cur_read + (float) momentError(i);
        }
    }

    std::cout << sensor[0] <<" "<< std::endl;

}

void readsensor()
{
    CLinuxSerial forcesensor(0,115200);
    Eigen::Matrix<double,3,3> R;
    ElfinModel elfin;
    while (true)
    {
        forcesensor.Sensor();
        for (int i = 0; i < 6; i++)
        {
            sensor[i]=forcesensor.sensor[i];
        }
        elfin.GetOri(cur_joints, R);
        ForceTorqueError(R, sensor);
    }
}

void testread(double *testrr,double *testtt)
{
    while (true)
    {
        //std::cout<<pointer->xroterr<<","<<pointer->xtraerr<<std::endl;
        
        std::cout<<"!!!!!!!!!! "<<*testrr<<","<<*testtt<<std::endl;
        usleep(100000);
    }
    
}

int main()
{   
    /////////////////////
    /// Init Touch X ///
    ///////////////////
    HDErrorInfo error;
    HHD hHD;
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialized haptic device");
    }

    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
    }

    HHD_Auto_Calibration();

    OmniState state;
    hdScheduleAsynchronous(omni_state_callback, &state,
        HD_MAX_SCHEDULER_PRIORITY);

    // pointers for us image algrithom calculation
    // double *rotateX;
    // double *translateX;
    // double valuerotate=0;
    // double valuetrans = 0;

    // rotateX = & valuerotate;    // address of storing rotation value
    // translateX = & valuetrans;  // address of storing translation value

    // ImageConsumer image_read_process;
    RobotControl roboctr;
    // std::thread task0(&ImageConsumer::ImageReader,image_read_process);
    // std::thread task1(&ImageConsumer::ImageProcesser,image_read_process,sensor);

    // //std::thread task4(testread,rotateX,translateX);
    // std::thread task2(readsensor);
    std::thread task3(&RobotControl::GeomagicControl,roboctr, &state);
    // task0.join();
    // task1.join();
    // //task4.join();
    // task2.join();
    task3.join();

    hdStopScheduler();
    hdDisableDevice(hHD);

    return 0;
    
}

