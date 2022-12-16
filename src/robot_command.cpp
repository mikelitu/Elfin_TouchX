#include <chrono>
#include "robot_command.hpp"

DcsCommand::DcsCommand(){
    std::string string_ip = "192.168.0.10";
    strcpy(robot_ip,string_ip.c_str());
    robot_port = 10003;
    splits = ";";
}

void DcsCommand::ConnectTCPSocket(){
    struct sockaddr_in server_addr;
    sock_hans = socket(AF_INET,SOCK_STREAM, 0);
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(robot_ip);
    server_addr.sin_port = htons(robot_port);
    sock_hans = socket(AF_INET, SOCK_STREAM, 0);
    if(connect(sock_hans, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){
        std::cout << "Robot Connect!" << std::endl;
    }
}

void DcsCommand::SetSpeedRadio(double rate){
    command_msg = "SetOverride,0," + std::to_string(rate) + ",;";
    memset(send_msg, 0, 200);
    strcpy(send_msg, command_msg.c_str());
    send_len = send(sock_hans, send_msg, strlen(send_msg), 0);
    if(send_len < 0){
        std::cout << "socket error!" << std::endl;
    }else{
        memset(recv_msg, 0, 200);
        recv_len = recv(sock_hans, recv_msg, 200, 0);
        return_msg = recv_msg;
        std::cout << return_msg << std::endl;
    }
}

void DcsCommand::ReadCurrentJoint(Eigen::Matrix<double,1,6>& cur_joints){
    command_msg = "ReadActPos,0,;";
    memset(send_msg, 0, 200);
    strcpy(send_msg, command_msg.c_str());
    send_len = send(sock_hans, send_msg, strlen(send_msg), 0);
    if(send_len < 0){
        std::cout << "socket error!" << std::endl;
    }else{
        memset(recv_msg, 0, 500);
        recv_len = recv(sock_hans, recv_msg, 500, 0);
        sscanf(recv_msg,"ReadActPos,OK,%lf,%lf,%lf,%lf,%lf,%lf",
            &cur_joints(0),&cur_joints(1),&cur_joints(2),&cur_joints(3),&cur_joints(4),&cur_joints(5));
    }
    cur_joints = cur_joints*M_PI/180;
}

void DcsCommand::MoveJ(Eigen::Matrix<double,1,6>& target_joints){
    target_joints = target_joints*180/M_PI;
    command_msg = "MoveJ,0,";
    for(int i=0;i<6;i++){
        command_msg += std::to_string(target_joints[i]) + ",";
    }
    command_msg += ";";
    std::cout << command_msg << std::endl;
    memset(send_msg, 0, 200);
    strcpy(send_msg, command_msg.c_str());
    send_len = send(sock_hans, send_msg, strlen(send_msg), 0);
    if(send_len < 0){
        std::cout << "socket error!" << std::endl;
    }else{
        memset(recv_msg, 0, 500);
        recv_len = recv(sock_hans, recv_msg, 500, 0);
        return_msg = recv_msg;
        std::cout << return_msg << std::endl;
    }
}

bool DcsCommand::isMoving(){
    command_msg = "ReadRobotState,0,;";
    memset(send_msg, 0, 200);
    strcpy(send_msg, command_msg.c_str());
    send_len = send(sock_hans, send_msg, strlen(send_msg), 0);
    if(send_len < 0){
        std::cout << "socket error!" << std::endl;
    }else{
        memset(recv_msg, 0, 500);
        recv_len = recv(sock_hans, recv_msg, 500, 0);
        int movement_flag;
        sscanf(recv_msg,"ReadRobotState,OK,%d",&movement_flag);
        if(movement_flag){
            return true;
        }else{
            return false;
        }
    }
}

void DcsCommand::StartServo(double servo_time, double lookahead_time){
    command_msg = "StartServo,0," + std::to_string(servo_time) + "," + std::to_string(lookahead_time) + ",;";
    std::cout << command_msg << std::endl;
    memset(send_msg, 0, 200);
    strcpy(send_msg, command_msg.c_str());
    send_len = send(sock_hans, send_msg, strlen(send_msg), 0);
    if(send_len < 0){
        std::cout << "socket error!" << std::endl;
    }else{
        memset(recv_msg, 0, 500);
        recv_len = recv(sock_hans, recv_msg, 500, 0);
        return_msg = recv_msg;
        std::cout << return_msg << std::endl;
    }
}

void DcsCommand::PushServoJ(Eigen::Matrix<double,1,6>& joints){
    joints = joints*180/M_PI;
    command_msg = "PushServoJ,0,";
    for(int i=0;i<6;i++){
        command_msg += std::to_string(joints[i]) + ",";
    }
    command_msg += ";";
    // std::cout << command_msg << std::endl;
    memset(send_msg, 0, 200);
    strcpy(send_msg, command_msg.c_str());
    send_len = send(sock_hans, send_msg, strlen(send_msg), 0);
    if(send_len < 0){
        std::cout << "socket error!" << std::endl;
    }else{
        memset(recv_msg, 0, 500);
        recv_len = recv(sock_hans, recv_msg, 500, 0);
        return_msg = recv_msg;
        // std::cout << return_msg << std::endl;
    }
}