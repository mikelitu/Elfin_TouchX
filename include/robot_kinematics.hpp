#include <math.h>
#include <iostream>
#include <Eigen/Dense>

class ElfinModel
{
public:
    float d1,a2,d4,d6;// DH Parameters
public:
    ElfinModel();
    void GetKinematics(Eigen::Matrix4d& cur_kinematics, Eigen::Matrix<double,1,6>& cur_joints);
    void GetJacobian(Eigen::Matrix<double,6,6>& cur_jacobian, Eigen::Matrix<double,1,6>& cur_joints);
    void GetNextJoints(Eigen::Matrix<double,1,6>& next_joints, Eigen::Matrix<double,1,6>& cur_joints, Eigen::Matrix<double,3,1>& pos_error,Eigen::Matrix<double,3,1> roterror);
    void GetOri(Eigen::Matrix<double,1,6>& cur_joints, Eigen::Matrix<double,3,3>& R);
};