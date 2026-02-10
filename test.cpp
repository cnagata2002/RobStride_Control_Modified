#include <iostream>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <cerrno>


//includes for socketCAN
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>


const int MOTOR_ID_DEFAULT = 11;
const char* CAN_INTERFACE = "can0";
const int HOST_ID = 0xFF;

const double ANGLE_LIM = 4.0 * M_PI;
const double SPEED_LIM = 330.;
const double KP_LIM = 5000.;
const double KD_LIM = 5.0;
const double TORQUE_LIM = 14.0;

struct motor_status {
    uint8_t motor_id;
    uint16_t motor_angle;
    uint16_t motor_speed;
    uint16_t motor_torque;
    uint16_t motor_temp;
};

struct mit_parameters{
    double target_angle;
    double target_speed;
    double kp;
    double kd;
    double target_torque;
};

void pack_n_le(uint8_t* buf, const void* val, size_t size) {
    for (size_t i = 0; i < size; i++) {
        buf[i] = ((uint8_t*)val)[i]; 
    }
}

void pack_n_be(uint8_t* buf, const void* val, size_t size) {
    for (size_t i = 0; i < size; i++) {
        buf[i] = ((uint8_t*)val)[size - 1 - i]; 
    }
}

void clamp(mit_parameters* param){
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

/*
Initialize CAN
*/
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
send frame
*/
bool send_frame(int s, uint32_t id, const uint8_t* data, uint8_t dlc){
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
    int nbytes = write(s, &frame, sizeof(struct can_frame));
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

/*
read frame
*/
bool read_frame(int s, struct can_frame* frame) {
    //read frame
    int nbytes = read(s, frame, sizeof(struct can_frame));
    if(nbytes < 0){
        return false;
    }
    return true;
}

/*
enable motor
*/
bool enable(int s, uint32_t id){
    // uint64_t init_data = 0xFFFFFFFFFFFFFFFC;
    // uint8_t data[8];
    // pack_n_le(&data[0], &init_data, 8);
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    return send_frame(s, id, data, 8);
}

/*
disable motor
*/
bool disable(int s, uint32_t id){
    // uint64_t disable_data = 0xFFFFFFFFFFFFFFFD;
    // uint8_t data[8];
    // pack_n_le(&data[0], &disable_data, 8);
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    return send_frame(s, id, data, 8);
}

/*
read status response 1
*/
bool status_response(int s, motor_status* status){
    struct can_frame frame;
    if(read_frame(s, &frame)){
        if(frame.can_id == HOST_ID && frame.can_dlc == 8){
            status->motor_id = frame.data[0];
            status->motor_angle = ((uint16_t)frame.data[1] << 8) | (uint16_t)frame.data[2];
            status->motor_speed = (uint16_t)(frame.data[3] << 4) | (uint16_t)((frame.data[4] >> 4) & 0x0F);
            status->motor_torque = ((uint16_t)(frame.data[4] & 0x0F) << 8) | (uint16_t)(frame.data[5]);
            status->motor_temp = ((uint16_t)frame.data[6] << 8) | (uint16_t)frame.data[7];
        } else {
            return false;
        }
    } else {
        return false;
    }
    return true;
}

bool mit_mode(int s, uint32_t id, mit_parameters* param){
    clamp(param);
    uint64_t target_angle_data = ((param->target_angle)/(2*ANGLE_LIM) + 0.5) * 0xFFFF;
    uint64_t target_speed_data = ((param->target_speed)/(2*SPEED_LIM) + 0.5) * 0xFFFF;
    uint64_t kp_data = ((param->kp)/KP_LIM) * 0xFFFF;
    uint64_t kd_data = ((param->kd)/KD_LIM) * 0xFFFF;
    uint64_t target_torque_data = ((param->target_torque)/(2*TORQUE_LIM) + 0.5) * 0xFFFF;

    uint64_t mit_data = ((target_angle_data & 0xFFFF) << 48) | 
        ((target_speed_data & 0x0FFF) << 36) |
        ((kp_data & 0x0FFF) << 24) |
        ((kd_data & 0x0FFF) << 12) |
        ((target_torque_data & 0x0FFF));
        
    uint8_t data[8];
    pack_n_be(&data[0], &mit_data, 8);
    return send_frame(s, id, data, 8);
}

int main(int argc, char* argv[]){
    int motor_id = MOTOR_ID_DEFAULT;
    if (argc > 1) {
        motor_id = std::atoi(argv[1]);
    }
    int s = init_can(CAN_INTERFACE);

    if (s < -1){
        std::cout << "init failed" << '\n';
        return 0;
    } else { 
        std::cout << "init successful" << '\n';
    }

    if(enable(s, motor_id)){
        std::cout << "enabled" << '\n';
    } else{
        std::cout << "enable failed" << '\n';
        return 0;
       }

    return 0;
}

