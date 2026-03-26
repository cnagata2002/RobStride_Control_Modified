#include "motor.h" 

#include <atomic>
#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PRINTVAL(descriptor, value) std::setw(strlen(descriptor)) << descriptor << std::setw(6) << value

const int MOTOR_ID_DEFAULT = 1;
const char* CAN_INTERFACE = "can0";
const int HOST_ID = 0xFD;

std::atomic<bool> stop_requested(false);

void signal_handler(int) {
    stop_requested = true;  
}

int main(int argc, char* argv[]){
    Motor::motor_status status;
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

    motor0.disable();
    usleep(20000);

    if(motor0.set_operation_mode(2)){
        std::cout << "set mode successfully" << '\n';
    } else{
        std::cout << "set mode failed" << '\n';
    }
    usleep(20000);

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
    

    double freq = 2000.0;
    double dt = 1.0/freq;

    int data_log_counter = 0;
    int data_log_freq = 5;

    auto period = std::chrono::microseconds(static_cast<int>(dt * 1e6));
    auto start = std::chrono::steady_clock::now();
    auto next = start;

    std::ofstream file("log.txt");

    /*
    CODE HERE
    */
    double I_motor = 0.017;
    double b_motor = 0.004;
    double tau_s_motor = 0.35;
    double tau_c_motor = 0.09;
    double omega_s = 0.03;

    MatrixXd A(3,3);
    MatrixXd B(3,1);
    MatrixXd C_d(2,3);
    A << 0,1,0,0,-b_motor/I_motor,1/I_motor,0,0,0;
    B << 0, 1/I_motor, 0;
    C_d << 1,0,0,0,1,0;

    MatrixXd I = MatrixXd::Identity(3, 3);
    // MatrixXd A_d = (A*dt).exp();
    // MatrixXd B_d = A.inverse()*(A_d-I)*B;

    MatrixXd M(4,4);
    M << A, B,
        MatrixXd::Zero(1,4);
    MatrixXd Md = (M * dt).exp();
    MatrixXd A_d = Md.block(0,0,3,3);
    MatrixXd B_d = Md.block(0,3,3,1);

    VectorXd x_k = VectorXd::Zero(3);
    VectorXd x_k_pred = VectorXd::Zero(3);
    VectorXd u_k = VectorXd::Zero(1);
    VectorXd y_k = VectorXd::Zero(2);

    MatrixXd K_k = MatrixXd::Zero(3,3);
    MatrixXd P_k = MatrixXd::Zero(3,3);
    MatrixXd P_k_pred = MatrixXd::Zero(3,3);

    MatrixXd Q = MatrixXd::Zero(3,3);
    MatrixXd R = MatrixXd::Zero(2,2);

    P_k << 1e-5,0,0,
        0,1e-5,0,
        0,0,1e-5;

    Q << 1e-10,0,0,
        0,1e-10,0,
        0,0,1e-10;

    R << 1e-8,0,
        0,1e-7;

    double angle_kf = 0;
    double speed_kf = 0;
    double torque_kf = 0;


    while(!stop_requested){
        auto loop_start = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(loop_start - start).count();
        next += period;
        
        motor0.set_speed(0.0005*std::pow(t,3),5.0);
        if (motor0.status_response(&status)){ 
            std::cout 
                << std::fixed 
                << std::setprecision(3) 
                << PRINTVAL("angle:",static_cast<double>(status.motor_angle)) 
                << PRINTVAL(" speed:",static_cast<double>(status.motor_speed))
                << PRINTVAL(" torq:",static_cast<double>(status.motor_torque));
            y_k(0) = status.motor_angle;
            y_k(1) = status.motor_speed;
        } else{
            std::cout << "read failed" << '\n';
        }

        /*
        CODE HERE
        */
        u_k(0) = static_cast<double>(status.motor_torque);
        x_k_pred = A_d * x_k + B_d * u_k;
        P_k_pred = A_d * P_k * A_d.transpose() + Q;

        K_k = P_k_pred * C_d.transpose() * (C_d * P_k_pred * C_d.transpose() + R).inverse();
        
        x_k = x_k_pred + K_k * (y_k - C_d * x_k_pred);
        P_k = (I - K_k * C_d) * P_k_pred;

        angle_kf = x_k(0);
        speed_kf = x_k(1);
        torque_kf = x_k(2);

        std::cout 
        << PRINTVAL(" angle kf: ", angle_kf)
        << PRINTVAL(" speed kf: ", speed_kf)
        << PRINTVAL(" torque kf: ", torque_kf);


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
            << static_cast<double>(status.motor_speed) << ","
            << static_cast<double>(status.motor_torque) << ","
            << angle_kf << ","
            << speed_kf << ","
            << torque_kf << ","
            << "\n";
            data_log_counter = 0;
        } else{
            data_log_counter++;
        }
    }

    return 0;
}