#include "motor.h" 

#include <atomic>
#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

#define PRINTVAL(descriptor, value) std::setw(strlen(descriptor)) << descriptor << std::setw(6) << value

const int MOTOR_ID_DEFAULT = 1;
const char* CAN_INTERFACE = "can0";
const int HOST_ID = 0xFD;

std::atomic<bool> stop_requested(false);

void signal_handler(int) {
    stop_requested = true;  
}

int main(int argc, char* argv[]){
    std::signal(SIGINT, signal_handler);

    int s = init_can(CAN_INTERFACE);
    if (s < 0){
        std::cout << "socket init failed" << '\n';
        return 0;
    } else { 
        std::cout << "socket init successful, s =" << s << '\n';
        
    }
    struct can_settings settings(s, CAN_INTERFACE, HOST_ID);

    int motor_id = MOTOR_ID_DEFAULT;

    Motor motor0(motor_id, &settings);

    if(motor0.enable()){
        std::cout << "enabled" << '\n';
        motor0.fault_clear();
    } else{
        std::cout << "enable failed" << '\n';
        return 0;
       }

    if(motor0.set_zero()){
        std::cout << "zeroed" << '\n';
    } else{
        std::cout << "zero failed" << '\n';
        return 0;
       }

    Motor::motor_status status;

    double I_motor = 0.017;
    double b_motor = 0.004;
    double tau_s_motor = 0.33;
    double tau_c_motor = 0.1;
    double omega_s = 0.22;

    double I_des = 0.017;
    double K_des = 1.0;
    double D_des = 0.01;
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

    double motor_speed_prev = 0;
    double motor_acc = 0;
    double tau  = 0.01;
    double alpha = dt/(tau + dt);

    double tau_f_motor = 0;
    double tau_ext = 0;

    int data_log_counter = 0;
    int data_log_freq = 5;

    double target_torque = 0;

    double motor_speed_filtered = 0;

    double last_angle = 0;
    double manual_speed_filtered = 0;
    double tau_speed  = 0.05;
    double alpha_speed = dt/(tau_speed + dt);

    std::ofstream file("log.txt");

    while(!stop_requested){
        auto loop_start = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(loop_start - start).count();
        next += period;
        
        motor0.set_torque(target_torque);
        if (motor0.status_response(&status)){ 
            std::cout 
                << std::fixed 
                << std::setprecision(3) 
                << PRINTVAL("angle: ",static_cast<double>(status.motor_angle)) 
                << PRINTVAL(" torq: ",static_cast<double>(status.motor_torque)) ;
            motor_acc = -(motor_acc + alpha * (((status.motor_speed - motor_speed_prev)/dt) - motor_acc));
            motor_speed_prev = status.motor_speed;

            motor_speed_filtered = motor_speed_filtered + alpha * (status.motor_speed - motor_speed_filtered);

            manual_speed_filtered = manual_speed_filtered + alpha_speed * ((status.motor_angle-last_angle)/dt - manual_speed_filtered);
            last_angle = status.motor_angle;


        } else{
            std::cout << "read failed" << '\n';
        }

        if (std::abs(manual_speed_filtered) > 0.01){
            sign = 1.0*std::tanh((status.motor_speed)/1e-3);
        } else {
            sign = 1.0*std::tanh((manual_speed_filtered)/1e-3);
            // sign = -1.0;
        }
        // double eps = 0.05;
        // if (manual_speed_filtered > eps) {
        //     sign = 1;
        // } else if (manual_speed_filtered < -eps){
        //     sign = -1;
        // }
            
        tau_f_motor = sign*(tau_c_motor + (tau_s_motor - tau_c_motor) * std::exp(-std::abs(status.motor_speed)/omega_s)) + b_motor * status.motor_speed;

        tau_ext = status.motor_torque - I_motor*(motor_acc) - tau_f_motor;  

        a = theta_ddot_des + 1/I_des * (
            D_des * (theta_dot_des - status.motor_speed) + 
            K_des * (theta_des - status.motor_angle) + tau_ext);

        target_torque = I_motor * a + tau_f_motor - tau_ext;
        // target_torque = I_motor * a;

        std::cout 
            << PRINTVAL(" torq_cmd:",target_torque)
            << PRINTVAL(" sign:", sign)
            << PRINTVAL(" torq_est_ext:", tau_ext)
            << PRINTVAL(" speed:",status.motor_speed)
            << PRINTVAL(" acc:",motor_acc);

        std::this_thread::sleep_until(next);

        auto loop_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = loop_end - loop_start;
        std::cout 
            << std::defaultfloat 
            << PRINTVAL(" t:" ,t)
            << "\n";

        if (data_log_counter >= data_log_freq) {
            file << t << "," 
            << static_cast<double>(status.motor_angle) << "," 
            << static_cast<double>(status.motor_torque) << ","
            << static_cast<double>(sign) << "," 
            << static_cast<double>(status.motor_speed) << ","
            << static_cast<double>(motor_speed_filtered) << ","
            << static_cast<double>(manual_speed_filtered) << ","
            << "\n";
            data_log_counter = 0;
        } else{
            data_log_counter++;
        }
    }

    return 0;
}