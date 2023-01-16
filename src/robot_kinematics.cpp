#include "robot_kinematics.hpp"
/**
 * @brief Construct a new Elfin Model:: Elfin Model object
 * 
 */
ElfinModel::ElfinModel(){
    d1 = 0.1925;
    a2 = 0.266;  
    d4 = 0.324;
    d6 = 0.155;
}

/**
 * @brief 
 * 
 * @param cur_kinematics 
 * @param cur_joints 
 */

void ElfinModel::GetKinematics(Eigen::Matrix4d& cur_kinematics, Eigen::Matrix<double,1,6>& cur_joints){
    double x1,x2,x3,x4,x5,x6;
    x1 = cur_joints(0,0);
    x2 = cur_joints(0,1);
    x3 = cur_joints(0,2);
    x4 = cur_joints(0,3);
    x5 = cur_joints(0,4);
    x6 = cur_joints(0,5);

    cur_kinematics.fill(0);
    cur_kinematics << -cos(x6) * (cos(x5) * (sin(x1) * sin(x4) - cos(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) + sin(x5) * (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2))) - sin(x6) * (cos(x4) * sin(x1) + sin(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))), 
        sin(x6)* (cos(x5) * (sin(x1) * sin(x4) - cos(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) + sin(x5) * (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2))) - cos(x6) * (cos(x4) * sin(x1) + sin(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))), 
        cos(x5)* (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)) - sin(x5) * (sin(x1) * sin(x4) - cos(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))), 
        d4* (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)) - d6 * (sin(x5) * (sin(x1) * sin(x4) - cos(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) - cos(x5) * (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2))) + a2 * cos(x1) * cos(x2 + M_PI / 2),
        -cos(x6) * (sin(x5) * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2)) - cos(x5) * (cos(x1) * sin(x4) + cos(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)))) - sin(x6) * (sin(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x1) * cos(x4)),
        sin(x6)* (sin(x5) * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2)) - cos(x5) * (cos(x1) * sin(x4) + cos(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)))) - cos(x6) * (sin(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x1) * cos(x4)), 
        sin(x5)* (cos(x1) * sin(x4) + cos(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) + cos(x5) * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2)),
        d4* (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2)) + d6 * (sin(x5) * (cos(x1) * sin(x4) + cos(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) + cos(x5) * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2))) + a2 * cos(x2 + M_PI / 2) * sin(x1),
        sin(x4)* sin(x6)* (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)) - cos(x6) * (sin(x5) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) + cos(x4) * cos(x5) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2))), 
        sin(x6)* (sin(x5) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) + cos(x4) * cos(x5) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2))) + cos(x6) * sin(x4) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)),
        cos(x5)* (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x4) * sin(x5) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)),
        d1 + d6 * (cos(x5) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x4) * sin(x5) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2))) + d4 * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) + a2 * sin(x2 + M_PI / 2),
        0, 0, 0, 1;
    // std::cout << "kenimatics = " << cur_kenimatics << std::endl;
}

/**
 * @brief 
 * 
 * @param cur_jacobian 
 * @param cur_joints 
 */

void ElfinModel::GetJacobian(Eigen::Matrix<double,6,6>& cur_jacobian, Eigen::Matrix<double,1,6>& cur_joints){
    double x1,x2,x3,x4,x5,x6;
    x1 = cur_joints(0,0);
    x2 = cur_joints(0,1);
    x3 = cur_joints(0,2);
    x4 = cur_joints(0,3);
    x5 = cur_joints(0,4);
    x6 = cur_joints(0,5);

    cur_jacobian.fill(0);
    cur_jacobian(0, 0) = -d4 * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2)) - d6 * (sin(x5) * (cos(x1) * sin(x4) + cos(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) + cos(x5) * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2))) - a2 * cos(x2 + M_PI / 2) * sin(x1);
    cur_jacobian(0, 1) = -d4 * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - d6 * (cos(x5) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x4) * sin(x5) * (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2))) - a2 * cos(x1) * sin(x2 + M_PI / 2);
    cur_jacobian(0, 2) = d4 * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) + d6 * (cos(x5) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x4) * sin(x5) * (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)));
    cur_jacobian(0, 3) = -d6 * sin(x5) * (cos(x4) * sin(x1) + sin(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)));
    cur_jacobian(0, 4) = -d6 * (cos(x5) * (sin(x1) * sin(x4) - cos(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) + sin(x5) * (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)));
    cur_jacobian(0, 5) = 0;
    cur_jacobian(1, 0) = d4 * (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)) - d6 * (sin(x5) * (sin(x1) * sin(x4) - cos(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) - cos(x5) * (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2))) + a2 * cos(x1) * cos(x2 + M_PI / 2);
    cur_jacobian(1, 1) = -d6 * (cos(x5) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x4) * sin(x5) * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2))) - d4 * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - a2 * sin(x1) * sin(x2 + M_PI / 2);
    cur_jacobian(1, 2) = d6 * (cos(x5) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x4) * sin(x5) * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2))) + d4 * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2));
    cur_jacobian(1, 3) = -d6 * sin(x5) * (sin(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x1) * cos(x4));
    cur_jacobian(1, 4) = -d6 * (sin(x5) * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2)) - cos(x5) * (cos(x1) * sin(x4) + cos(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))));
    cur_jacobian(1, 5) = 0;
    cur_jacobian(2, 0) = 0;
    cur_jacobian(2, 1) = d6 * (cos(x5) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)) + cos(x4) * sin(x5) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) + a2 * cos(x2 + M_PI / 2) + d4 * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2));
    cur_jacobian(2, 2) = -d6 * (cos(x5) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)) + cos(x4) * sin(x5) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) - d4 * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2));
    cur_jacobian(2, 3) = d6 * sin(x4) * sin(x5) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2));
    cur_jacobian(2, 4) = -d6 * (sin(x5) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) + cos(x4) * cos(x5) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)));
    cur_jacobian(2, 5) = 0;
    cur_jacobian(3, 0) = 0;
    cur_jacobian(3, 1) = sin(x1);
    cur_jacobian(3, 2) = -sin(x1);
    cur_jacobian(3, 3) = cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2);
    cur_jacobian(3, 4) = -cos(x4) * sin(x1) - sin(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2));
    cur_jacobian(3, 5) = cos(x5) * (cos(x1) * cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x1) * cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2)) - sin(x5) * (sin(x1) * sin(x4) - cos(x4) * (cos(x1) * cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + cos(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)));
    cur_jacobian(4, 0) = 0;
    cur_jacobian(4, 1) = -cos(x1);
    cur_jacobian(4, 2) = cos(x1);
    cur_jacobian(4, 3) = cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2);
    cur_jacobian(4, 4) = cos(x1) * cos(x4) - sin(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2));
    cur_jacobian(4, 5) = sin(x5) * (cos(x1) * sin(x4) + cos(x4) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) * sin(x1) + sin(x1) * sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2))) + cos(x5) * (cos(x2 + M_PI / 2) * sin(x1) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x1) * sin(x2 + M_PI / 2));
    cur_jacobian(5, 0) = 1;
    cur_jacobian(5, 1) = 0;
    cur_jacobian(5, 2) = 0;
    cur_jacobian(5, 3) = cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2);
    cur_jacobian(5, 4) = sin(x4) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2));
    cur_jacobian(5, 5) = cos(x5) * (cos(x2 + M_PI / 2) * cos(x3 + M_PI / 2) + sin(x2 + M_PI / 2) * sin(x3 + M_PI / 2)) - cos(x4) * sin(x5) * (cos(x2 + M_PI / 2) * sin(x3 + M_PI / 2) - cos(x3 + M_PI / 2) * sin(x2 + M_PI / 2));
}

/**
 * @brief 
 * 
 * @param next_joints 
 * @param cur_joints 
 * @param pos_error 
 * @param roterror 
 */

void ElfinModel::GetNextJoints(Eigen::Matrix<double,1,6>& next_joints, Eigen::Matrix<double,1,6>& cur_joints, Eigen::Matrix<double,3,1>& pos_error,Eigen::Matrix<double,3,1> roterror){
    Eigen::Matrix4d cur_kinematics;
    Eigen::Matrix<double,6,6> cur_jacobian;
    Eigen::Matrix<double,1,6> error_kinematics;


    // Calculate the cartesian position error
    GetJacobian(cur_jacobian,cur_joints);
    GetKinematics(cur_kinematics,cur_joints);
    Eigen::Matrix3d cur_rot;
    cur_rot = cur_kinematics.block(0,0,3,3);

    // input the rotation change in axis
    Eigen::MatrixXd kinematicRot_e;
    Eigen::MatrixXd kinematicPos_e;
    
    kinematicPos_e = cur_rot * pos_error;
    kinematicRot_e = cur_rot * roterror;

    for(int i=0;i<6;i++){
        if(i<3){
            error_kinematics(0,i) = kinematicPos_e(i); //next_kinematics(i,3)-cur_kinematics(i,3);
        }else{
            error_kinematics(0,i) = kinematicRot_e(i-3); //rotation_axis.axis()(i-3)*rotation_axis.angle();
        }
    }


    //std::cout << "error_kenimatics = " << error_kenimatics << std::endl;
    // Using damped joints method to get next joints
    double lamba = 0.001;
    Eigen::Matrix<double,6,6> jacobian_pseudo_inverse;
    jacobian_pseudo_inverse = (cur_jacobian.transpose())*((cur_jacobian*cur_jacobian.transpose() + lamba*Eigen::MatrixXd::Identity(6,6)).inverse());
    next_joints = cur_joints + (jacobian_pseudo_inverse*error_kinematics.transpose()).transpose();
    for(int i=0;i<6;i++){
        if(next_joints[i]>0){
            next_joints(i) = fmod(next_joints(i), 2*M_PI);
        }else{
            next_joints(i) = fmod(next_joints(i),-2*M_PI);
        }
    }
    //std::cout << "next_joints = " << next_joints << std::endl;
}

/**
 * @brief 
 * 
 * @param cur_joints 
 * @param R 
 */

void ElfinModel::GetOri(Eigen::Matrix<double,1,6>& cur_joints, Eigen::Matrix<double,3,3>& R)
{
    Eigen::Matrix4d cur_kinematics;
    
    //Calculate the current kinematics
    GetKinematics(cur_kinematics, cur_joints);
    R = cur_kinematics.block(0,0,3,3);
    //std::cout << "Rotation matrix: " << R << std::endl;
}
