#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <cerrno>
#include <chrono>
#include <thread>
#include <sys/select.h> 

#include <fstream>

//includes for socketCAN
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define PRINTVAL(descriptor, value) std::setw(strlen(descriptor)) << descriptor << std::setw(6) << value

const int MOTOR_ID_DEFAULT = 1;
const char* CAN_INTERFACE = "can0";
const int HOST_ID = 0xFD;

const double ANGLE_LIM = 4.0 * M_PI;
const double SPEED_LIM = 33.0;
const double KP_LIM = 500.;
const double KD_LIM = 5.0;
const double TORQUE_LIM = 14.0;

struct motor_status {
    uint8_t motor_id;
    uint16_t motor_angle_raw;
    uint16_t motor_speed_raw;
    uint16_t motor_torque_raw;
    uint16_t motor_temp_raw;
    double motor_angle;
    double motor_speed;
    double motor_torque;
    double motor_temp;

};

struct mit_parameters{
    double target_angle;
    double target_speed;
    double kp;
    double kd;
    double target_torque;
};

struct fault_status {
    uint8_t motor_id;
    uint32_t fault_value;
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
// bool read_frame(int s, struct can_frame* frame) {
//     //read frame
//     int nbytes = read(s, frame, sizeof(struct can_frame));
//     if(nbytes < 0){
//         return false;
//     }
//     return true;
// }
bool read_frame(int s, struct can_frame* frame) {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100 ms timeout

    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(s, &rdfs);

    int ret = select(s + 1, &rdfs, nullptr, nullptr, &tv);
    if (ret <= 0) {  // -1 error, 0 timeout
        return false;
    }

    int nbytes = read(s, frame, sizeof(struct can_frame));
    return nbytes == sizeof(struct can_frame);
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

bool set_zero(int s, uint32_t id){
    // uint64_t init_data = 0xFFFFFFFFFFFFFFFC;
    // uint8_t data[8];
    // pack_n_le(&data[0], &init_data, 8);
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
    return send_frame(s, id, data, 8);
}

/*
read status response 1
*/
bool status_response(int s, motor_status* status){
    struct can_frame frame;
    if(read_frame(s, &frame)){
        uint32_t id11 = frame.can_id & CAN_SFF_MASK;
        if(id11 == HOST_ID && frame.can_dlc == 8){
            status->motor_id = frame.data[0];
            status->motor_angle_raw = ((uint16_t)frame.data[1] << 8) | (uint16_t)frame.data[2];
            status->motor_speed_raw = (uint16_t)(frame.data[3] << 4) | (uint16_t)((frame.data[4] >> 4) & 0x0F);
            status->motor_torque_raw = ((uint16_t)(frame.data[4] & 0x0F) << 8) | (uint16_t)(frame.data[5]);
            status->motor_temp_raw = ((uint16_t)frame.data[6] << 8) | (uint16_t)frame.data[7];
            status->motor_angle = 25.14/65535.0 *(status->motor_angle_raw - 65535.0/2);
            status->motor_speed = 66/4096.0 *(status->motor_speed_raw - 4096.0/2);
            status->motor_torque = 28/4096.0 *(status->motor_torque_raw - 4096.0/2);
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

bool mit_mode(int s, uint32_t id, mit_parameters* param){
    clamp(param);
    uint64_t target_angle_data = ((param->target_angle)/(2*ANGLE_LIM) + 0.5) * 65535.0;
    uint64_t target_speed_data = ((param->target_speed)/(2*SPEED_LIM) + 0.5) * 4096.0;
    uint64_t kp_data = ((param->kp)/KP_LIM) * 4096.0;
    uint64_t kd_data = ((param->kd)/KD_LIM) * 4096.0;
    uint64_t target_torque_data = ((param->target_torque)/(2*TORQUE_LIM) + 0.5) * 4096.0;

    uint64_t mit_data = ((target_angle_data & 0xFFFF) << 48) | 
        ((target_speed_data & 0x0FFF) << 36) |
        ((kp_data & 0x0FFF) << 24) |
        ((kd_data & 0x0FFF) << 12) |
        ((target_torque_data & 0x0FFF));
        
    uint8_t data[8];
    pack_n_be(&data[0], &mit_data, 8);
    return send_frame(s, id, data, 8);
}

bool fault_read(int s, fault_status* fault){
    struct can_frame frame;
    if(read_frame(s, &frame)){
        uint32_t id11 = frame.can_id & CAN_SFF_MASK;
        if(id11 == HOST_ID && frame.can_dlc == 8){
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

bool fault_clear(int s, uint32_t id){
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};
    return send_frame(s, id, data, 8);
}

int main(int argc, char* argv[]){
    int motor_id = MOTOR_ID_DEFAULT;
    if (argc > 1) {
        motor_id = std::atoi(argv[1]);
    }
    int s = init_can(CAN_INTERFACE);

    if (s < 0){
        std::cout << "init failed" << '\n';
        return 0;
    } else { 
        std::cout << "init successful, s =" << s << '\n';
        
    }
    if(enable(s, motor_id)){
        std::cout << "enabled" << '\n';
        fault_clear(s, motor_id);
    } else{
        std::cout << "enable failed" << '\n';
        return 0;
       }

    if(set_zero(s, motor_id)){
        std::cout << "zeroed" << '\n';
    } else{
        std::cout << "zero failed" << '\n';
        return 0;
       }
    struct motor_status status;
    struct mit_parameters param;
    param.kp = 0.0;
    param.kd = 0.0;
    param.target_angle = 0.0;
    param.target_speed = 0.0;
    param.target_torque = 0.0;

    double I_motor = 0.017;
    double b_motor = 0.004;
    double tau_s_motor = 0.33;
    double tau_c_motor = 0.1;
    double omega_s = 0.22;

    double I_des = 0.017;
    double K_des = 1.0;
    double D_des = 0.0;
    // double theta_ddot_des = 0.0;
    double theta_ddot_des = -K_des/0.2;
    double theta_dot_des = 0.0;
    double theta_des = 0.0;

    double a = 0;

    double sign = 0;

    double freq = 2000.0;
    double dt = 1.0/freq;

    auto period = std::chrono::microseconds(static_cast<int>(dt * 1e6));
    auto start = std::chrono::steady_clock::now();
    auto next = start;

    std::ofstream file("log.txt");


    double motor_speed_prev = 0;
    double motor_acc = 0;
    double tau  = 0.01;
    double alpha = dt/(tau + dt);

    double tau_f_motor = 0;
    double tau_ext = 0;

    int data_log_counter = 0;
    int data_log_freq = 5;
    
    while (true) {
        auto loop_start = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(loop_start - start).count();

        next += period;

        mit_mode(s,motor_id, &param);
        if (status_response(s, &status)){ 
            std::cout 
                << std::fixed 
                << std::setprecision(3) 
                << PRINTVAL("angle: ",static_cast<double>(status.motor_angle)) 
                << PRINTVAL(" torq: ",static_cast<double>(status.motor_torque)) ;
            motor_acc = -(motor_acc + alpha * (((status.motor_speed - motor_speed_prev)/dt) - motor_acc));
            motor_speed_prev = status.motor_speed;

        } else{
            std::cout << "read failed" << '\n';
        }

        if (std::abs(status.motor_speed) > 0.05){
            sign = 1.0*std::tanh(status.motor_speed/1e-3);
        } else {
            sign = 1.0*std::tanh(status.motor_torque/1e-5);
        }
            
        tau_f_motor = sign*(tau_c_motor + (tau_s_motor - tau_c_motor) * std::exp(-std::abs(status.motor_speed)/omega_s)) + b_motor * status.motor_speed;

        tau_ext = status.motor_torque - I_motor*(motor_acc) - tau_f_motor;  

        a = theta_ddot_des + 1/I_des * (
            D_des * (theta_dot_des - status.motor_speed) + 
            K_des * (theta_des - status.motor_angle) + tau_ext);

        param.target_torque = I_motor * a + tau_f_motor - tau_ext;

        // param.target_torque = -0.5;
        param.target_torque = 0.0;

        std::cout 
        << PRINTVAL(" torq_cmd: ",param.target_torque)
        << PRINTVAL(" torq_est_ext: ", tau_ext)
        << PRINTVAL(" speed: ",status.motor_speed)
        << PRINTVAL(" acc: ",motor_acc);


        std::this_thread::sleep_until(next);

        auto loop_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = loop_end - loop_start;
        std::cout 
            << std::defaultfloat 
            << PRINTVAL(" time: " ,t)
            << "\n";

        if (data_log_counter >= data_log_freq) {
            file << t << "," << static_cast<double>(status.motor_angle) << "," << static_cast<double>(status.motor_torque) << "," << "\n";
            data_log_counter = 0;
        } else{
            data_log_counter++;
        }

    }   


    return 0;
}

