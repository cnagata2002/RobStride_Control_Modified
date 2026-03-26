#pragma once

#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <sys/select.h> 
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

struct can_settings {
    int socket;
    const char* can_interface;
    const int id;

    can_settings(int s_, const char* can_interface_, const int id_);
};

int init_can(const char* ifname);

class Motor {
    public:
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

        Motor(uint32_t id, can_settings* settings);
        ~Motor();
        bool send_frame(const uint8_t* data, uint8_t dlc, uint16_t id);
        bool send_frame(const uint8_t* data, uint8_t dlc);
        bool read_frame(struct can_frame* frame);
        bool enable();
        bool disable();
        bool set_zero();
        bool status_response(motor_status* status);
        bool mit_mode(mit_parameters* param);
        bool fault_read(fault_status* fault);
        bool fault_clear();
        bool set_torque(double tau);
        bool set_speed(double target_speed, double current_limit);
        bool set_angle(double target_angle, double target_speed);
        bool set_operation_mode(int mode);

    private:
        int s;
        const int motor_id;
        const char* can_interface;
        const int host_id;
        
        const double ANGLE_LIM = 4.0 * M_PI;
        const double SPEED_LIM = 33.0;
        const double KP_LIM = 500.;
        const double KD_LIM = 5.0;
        const double TORQUE_LIM = 14.0;
        const double CURRENT_LIM = 15.5;

        void pack_n_le(uint8_t* buf, const void* val, size_t size);
        void pack_n_be(uint8_t* buf, const void* val, size_t size);
        void clamp(mit_parameters* param);


};