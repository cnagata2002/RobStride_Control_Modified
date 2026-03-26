#include "motor.h"

#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <sys/select.h> 
#include <cerrno>

// #include <chrono>
// #include <thread>
// #include <fstream>

//includes for socketCAN
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>


/*
Initialize CAN
*/
can_settings::can_settings(int s_, const char* can_interface_, const int id_) 
    :socket(s_), can_interface(can_interface_), id(id_) {}

int init_can(const char* ifname){
    // initialize socket and interface
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(s < 0){
        perror("socket");
        return -1;
    }

    // set can interface name and index
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0){
        perror("ioctl");
        return -1;
    }

    // set up address
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // bind address
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0){
        perror("bind");
        return -1;
    }

    return s;
}

/*
constructor
*/
Motor::Motor(uint32_t id, can_settings* settings) 
    : motor_id(id), host_id(settings->id), can_interface(settings->can_interface)
    {
        s = settings->socket;
    }

/*
destructor
*/
Motor::~Motor() {
    disable();
    set_operation_mode(0);
    enable();
    set_torque(0.0);
}
    
/*
send frame
*/
bool Motor::send_frame(const uint8_t* data, uint8_t dlc, uint16_t id){
    //setup frame to send
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = dlc;
    if(data){
        memcpy(frame.data, data, dlc);
    }
    else{
        memset(frame.data, 0, 8);
    }
    //write frame
    int nbytes = write(this->s, &frame, sizeof(struct can_frame));
    // if(nbytes != sizeof(struct can_frame)){
    //     return false;
    // }
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "write failed, errno=" << errno
                << " (" << std::strerror(errno) << ")\n";
        return false;
    }
    return true;
}
bool Motor::send_frame(const uint8_t* data, uint8_t dlc){
    return send_frame(data, dlc, this->motor_id);
}

/*
read frame
*/
bool Motor::read_frame(struct can_frame* frame) {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100 ms timeout

    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(this->s, &rdfs);

    int ret = select(this->s + 1, &rdfs, nullptr, nullptr, &tv);
    if (ret <= 0) {  // -1 error, 0 timeout
        return false;
    }

    int nbytes = read(this-> s, frame, sizeof(struct can_frame));
    return nbytes == sizeof(struct can_frame);
}

/*
enable motor
*/
bool Motor::enable(){
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    return send_frame(data, 8);
}

/*
disable motor
*/
bool Motor::disable(){
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    return send_frame(data, 8);
}

/*
set zero
*/
bool Motor::set_zero(){
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
    return send_frame(data, 8);
}

/*
read status response 1
*/
bool Motor::status_response(motor_status* status){
    struct can_frame frame;
    if(read_frame(&frame)){
        uint32_t id11 = frame.can_id & CAN_SFF_MASK;
        if(id11 == this->host_id && frame.can_dlc == 8){
            status->motor_id = frame.data[0];
            status->motor_angle_raw = ((uint16_t)frame.data[1] << 8) | (uint16_t)frame.data[2];
            status->motor_speed_raw = (uint16_t)(frame.data[3] << 4) | (uint16_t)((frame.data[4] >> 4) & 0x0F);
            status->motor_torque_raw = ((uint16_t)(frame.data[4] & 0x0F) << 8) | (uint16_t)(frame.data[5]);
            status->motor_temp_raw = ((uint16_t)frame.data[6] << 8) | (uint16_t)frame.data[7];
            status->motor_angle = 25.14/65535.0 *(status->motor_angle_raw - 65535.0/2);
            status->motor_speed = 66/4095.0 *(status->motor_speed_raw - 4095.0/2);
            status->motor_torque = 28/4095.0 *(status->motor_torque_raw - 4095.0/2);
            status->motor_temp = status->motor_temp_raw/10.0;

        } else {
            std::cout << static_cast<int>(frame.can_dlc) << '\n';
            std::cout << static_cast<int>(id11) << '\n';
            return false;
        }
    } else {
        return false;
    }
    return true;
}

/*
set mit mode command
*/
bool Motor::mit_mode(mit_parameters* param){
    clamp(param);
    uint64_t target_angle_data = ((param->target_angle)/(2*ANGLE_LIM) + 0.5) * 0xFFFF;
    uint64_t target_speed_data = ((param->target_speed)/(2*SPEED_LIM) + 0.5) * 0xFFF;
    uint64_t kp_data = ((param->kp)/KP_LIM) * 0xFFF;
    uint64_t kd_data = ((param->kd)/KD_LIM) * 0xFFF;
    uint64_t target_torque_data = ((param->target_torque)/(2*TORQUE_LIM) + 0.5) * 0xFFF;

    uint64_t mit_data = ((target_angle_data & 0xFFFF) << 48) | 
        ((target_speed_data & 0x0FFF) << 36) |
        ((kp_data & 0x0FFF) << 24) |
        ((kd_data & 0x0FFF) << 12) |
        ((target_torque_data & 0x0FFF));
        
    uint8_t data[8];
    pack_n_be(&data[0], &mit_data, 8);
    return send_frame(data, 8);
}

/*
send torque command only
*/
bool Motor::set_torque(double tau){
    struct Motor::mit_parameters param;
    param.kp = 0.0;
    param.kd = 0.0;
    param.target_angle = 0.0;
    param.target_speed = 0.0;
    param.target_torque = tau;
    return mit_mode(&param);
}

/*
send speed command only
*/
bool Motor::set_speed(double target_speed, double current_limit){
    float current_limit_clamped = static_cast<float>(std::max(-CURRENT_LIM, std::min(CURRENT_LIM, current_limit)));
    float target_speed_clamped = static_cast<float>(std::max(-SPEED_LIM,std::min(SPEED_LIM, target_speed)));
    
    uint16_t modified_id = (2 << 8) | this->motor_id;
    uint8_t data[8];
    memcpy(&data[0], &target_speed_clamped, 4);     
    memcpy(&data[4], &current_limit_clamped, 4);

    return send_frame(data, 8, modified_id);
}

/*
send position command only
*/
bool Motor::set_angle(double target_angle, double target_speed){
    float target_angle_clamped = static_cast<float>(std::max(-ANGLE_LIM, std::min(ANGLE_LIM, target_angle)));
    float target_speed_clamped = static_cast<float>(std::max(-SPEED_LIM,std::min(SPEED_LIM, target_speed)));

    uint16_t modified_id = (1 << 8) | this->motor_id;
    uint8_t data[8];
    memcpy(&data[0], &target_angle_clamped, 4);
    memcpy(&data[4], &target_speed_clamped, 4);

    return send_frame(data, 8,modified_id);
}
/*
set operation mode
*/
bool Motor::set_operation_mode(int mode){
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,static_cast<uint8_t>(mode),0xFC};
    uint16_t modified_id = (static_cast<uint8_t>(mode) << 8) | this->motor_id;
    return send_frame(data, 8);
   
}

/*
read fault
*/
bool Motor::fault_read(fault_status* fault){
    struct can_frame frame;
    if(read_frame(&frame)){
        uint32_t id11 = frame.can_id & CAN_SFF_MASK;
        if(id11 == this->host_id && frame.can_dlc == 8){
            fault->motor_id = frame.data[0];
            fault->fault_value = frame.data[1] << 24 | 
                                frame.data[2] << 16 | 
                                frame.data[3] << 8 | 
                                frame.data[4];
            return true;
        } 
    }
    return false;
}

/*
clear all faults
*/
bool Motor::fault_clear(){
            uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};
            return send_frame(data, 8);
        }

/*
pack buffer lower ended
*/
void Motor::pack_n_le(uint8_t* buf, const void* val, size_t size) {
    for (size_t i = 0; i < size; i++) {
        buf[i] = ((uint8_t*)val)[i]; 
    }
}

/*
pack buffer big ended
*/
void Motor::pack_n_be(uint8_t* buf, const void* val, size_t size) {
    for (size_t i = 0; i < size; i++) {
        buf[i] = ((uint8_t*)val)[size - 1 - i]; 
    }
}

/*
clamp values
*/
void Motor::clamp(mit_parameters* param){
            double target_angle_clamped = std::max(-ANGLE_LIM, std::min(ANGLE_LIM, param->target_angle));
            double target_speed_clamped = std::max(-SPEED_LIM,std::min(SPEED_LIM, param->target_speed));
            double kp_clamped = std::max(0.0, std::min(KP_LIM, param->kp));
            double kd_clamped = std::max(0.0, std::min(KD_LIM, param->kd));
            double target_torque_clamped = std::max(-TORQUE_LIM, std::min(TORQUE_LIM, param->target_torque));

            param->target_angle = target_angle_clamped;
            param->target_speed = target_speed_clamped;
            param->kp = kp_clamped;
            param->kd = kd_clamped;
            param->target_torque = target_torque_clamped;
        }


