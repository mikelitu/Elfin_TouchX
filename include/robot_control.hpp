#pragma once
#include "robot_command.hpp"
#include "robot_kinematics.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h> 

struct OmniState {
    hduVector3Dd position;  //3x1 vector of position
    hduVector3Dd pre_position;
    hduVector3Dd velocity;  //3x1 vector of velocity
    hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
    hduVector3Dd inp_vel2;
    hduVector3Dd inp_vel3;
    hduVector3Dd out_vel1;
    hduVector3Dd out_vel2;
    hduVector3Dd out_vel3;
    hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
    hduVector3Dd pos_hist2;
    hduQuaternion rot;
    hduQuaternion pre_rot;
    hduVector3Dd joints;
    hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
    float thetas[7];
    int buttons[2];
    int buttons_prev[2];
    bool lock;
    bool close_gripper;
    hduVector3Dd lock_pos;
};


class RobotControl{
    public:
        RobotControl()
        {

        }

        void GeomagicControl(OmniState *state, Eigen::Matrix<double,1,6>& cur_joints, bool& init_exp);
        void Touch2Elfin(Eigen::Matrix<double,1,6>& cur_joints, Eigen::Matrix<double,1,6>& next_joint, OmniState *state, Eigen::Matrix<double,3,1> pos_error, Eigen::Matrix<double,3,1>& rot_err);
};