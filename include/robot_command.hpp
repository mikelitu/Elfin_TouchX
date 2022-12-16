#pragma once
#include <memory>
#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <Eigen/Dense>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>


class DcsCommand
{
public:
    int robot_port, sock_hans, send_len, recv_len;
    char robot_ip[16], send_msg[200], recv_msg[500];
    std::string splits, command_msg, return_msg;
public:
    DcsCommand();
    void ConnectTCPSocket();
    void SetSpeedRadio(double rate);
    void ReadCurrentJoint(Eigen::Matrix<double,1,6>& cur_joints);
    void MoveJ(Eigen::Matrix<double,1,6>& target_joints);
    bool isMoving();
    void StartServo(double servo_time,double lookahead_time);
    void PushServoJ(Eigen::Matrix<double,1,6>& joints);
};