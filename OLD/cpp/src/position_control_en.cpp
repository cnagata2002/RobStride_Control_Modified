/*
 * RobStride MIT Mode Position Control (C++ version)
 * Mode: Mode 0 (MIT Mode)
 * Communication: cyclic call to write_operation_frame
 *
 * Compile:
 * g++ -o position_control_mit position_control_mit.cpp -lpthread -std=c++17
 *
 * Run:
 * sudo ./position_control_mit <motor_id>
 * (sudo privileges required to access CAN hardware)
 */

#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <csignal>
#include <cmath>

// Linux SocketCAN headers
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// --- Global atomic variables for thread-safe communication ---
std::atomic<bool> running(true);
std::atomic<double> target_position(0.0);
std::atomic<double> kp(100.0); // High stiffness
std::atomic<double> kd(2.0);  // High damping

// --- Motor and protocol definitions ---
const int MOTOR_ID_DEFAULT = 11; // Default motor ID
const char* CAN_INTERFACE = "can0";
const int HOST_ID = 0xFF;

// Communication types
const uint32_t COMM_OPERATION_CONTROL = 1;
const uint32_t COMM_ENABLE = 3;
const uint32_t COMM_WRITE_PARAMETER = 18;

// Parameter IDs
const uint16_t PARAM_MODE = 0x7005;
const uint16_t PARAM_VELOCITY_LIMIT = 0x7017;
const uint16_t PARAM_TORQUE_LIMIT = 0x700B; // Note: according to protocol.py this should be 0x700B

// --- CAN frame packing helper functions ---

// Copy float into uint8_t array (little endian)
void pack_float_le(uint8_t* buf, float val) {
    memcpy(buf, &val, sizeof(float));
}

// Copy uint16_t into uint8_t array (little endian)
void pack_u16_le(uint8_t* buf, uint16_t val) {
    memcpy(buf, &val, sizeof(uint16_t));
}

// Pack uint16_t as big endian
void pack_u16_be(uint8_t* buf, uint16_t val) {
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

// --- Low-level CAN functions ---

/**
 * @brief Send a CAN frame
 */
bool send_frame(int s, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    struct can_frame frame;
    frame.can_id = can_id | CAN_EFF_FLAG; // Enable extended frame (29-bit)
    frame.can_dlc = dlc;
    if (data) {
        memcpy(frame.data, data, dlc);
    } else {
        memset(frame.data, 0, 8);
    }

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("write");
        return false;
    }
    return true;
}

/**
 * @brief Read a CAN frame (with timeout)
 */
bool read_frame(int s, struct can_frame* frame) {
    // Set 100ms timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(s, &rdfs);

    int ret = select(s + 1, &rdfs, NULL, NULL, &tv);
    if (ret == -1) {
        perror("select");
        return false;
    } else if (ret == 0) {
        // std::cerr << "read timeout" << std::endl;
        return false; // timeout
    }

    if (read(s, frame, sizeof(struct can_frame)) < 0) {
        perror("read");
        return false;
    }
    return true;
}

// --- RobStride protocol functions ---

bool enable_motor(int s, int motor_id) {
    uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

bool set_mode_raw(int s, int motor_id, int8_t mode) {
    uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], PARAM_MODE); // param ID
    data[4] = (uint8_t)mode;           // value (int8)
    return send_frame(s, ext_id, data, 8);
}

bool write_limit(int s, int motor_id, uint16_t param_id, float limit) {
    uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], param_id); // param ID
    pack_float_le(&data[4], limit);  // value (float)
    return send_frame(s, ext_id, data, 8);
}

bool write_operation_frame(int s, int motor_id, double pos, double kp_val, double kd_val) {
    // 1. Pack data (big endian!)
    // These scaling values should be imported from table.py; hardcoded here for simplicity
    const double POS_SCALE = 4 * M_PI; // rs-03
    const double VEL_SCALE = 50.0;     // rs-03
    const double KP_SCALE = 5000.0;    // rs-03
    const double KD_SCALE = 100.0;     // rs-03
    const double TQ_SCALE = 60.0;      // rs-03

    // Clamp and convert
    double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
    double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
    double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));
    
    uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
    uint16_t vel_u16 = 0x7FFF; // 0 velocity
    uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
    uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
    uint16_t torque_u16 = 0x7FFF; // 0 torque_ff

    uint8_t data[8];
    pack_u16_be(&data[0], pos_u16);
    pack_u16_be(&data[2], vel_u16);
    pack_u16_be(&data[4], kp_u16);
    pack_u16_be(&data[6], kd_u16);
    
    // 2. Build CAN ID
    uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;
    
    // 3. Send
    return send_frame(s, ext_id, data, 8);
}

bool read_operation_frame(int s) {
    struct can_frame frame;
    if (read_frame(s, &frame)) {
        if (!frame.can_id & CAN_EFF_FLAG) return false;
        
        uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
        if (comm_type == 2) { // Status packet
            return true;
        }
    }
    return false;
}

/**
 * @brief Control loop thread
 */
void control_loop(int s, int motor_id) {
    std::cout << "🔄 Control loop started (Mode 0 @ 50Hz)" << std::endl;
    
    while (running) {
        auto start = std::chrono::steady_clock::now();
        
        // 1. Send MIT frame (send only)
        write_operation_frame(s, motor_id, target_position.load(), kp.load(), kd.load());
        
        // 2. Read status frames (receive only), flush buffer
        while(read_operation_frame(s)); // Loop until buffer empty or timeout
        
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        // Fixed 50Hz loop
        std::this_thread::sleep_for(std::chrono::microseconds(20000) - elapsed);
    }
    std::cout << "⏹️ Control thread stopped" << std::endl;
}

/**
 * @brief Initialize SocketCAN
 */
int init_can(const char* ifname) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(s);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }

    return s;
}

void signal_handler(int signum) {
    std::cout << "\n🛑 Exit signal captured..." << std::endl;
    running = false;
}

int main(int argc, char* argv[]) {
    int motor_id = MOTOR_ID_DEFAULT;
    if (argc > 1) {
        motor_id = std::atoi(argv[1]);
    }
    
    std::cout << "🎯 MIT Position Console (ID: " << motor_id << ")" << std::endl;
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int s = init_can(CAN_INTERFACE);
    if (s < 0) {
        std::cerr << "❌ Unable to open CAN interface " << CAN_INTERFACE << std::endl;
        return 1;
    }
    std::cout << "📡 CAN bus " << CAN_INTERFACE << " connected successfully" << std::endl;

    // --- Initialize motor ---
    std::cout << "⚡ Enabling motor ID: " << motor_id << " ..." << std::endl;
    enable_motor(s, motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    std::cout << "⚙️ Switching to MIT mode (Mode 0)..." << std::endl;
    set_mode_raw(s, motor_id, 0);
    
    std::cout << "⚙️ Setting internal limits..." << std::endl;
    write_limit(s, motor_id, PARAM_VELOCITY_LIMIT, 20.0);
    write_limit(s, motor_id, PARAM_TORQUE_LIMIT, 20.0);
    
    std::cout << "🏠 Setting initial target to 0.0 ..." << std::endl;
    write_operation_frame(s, motor_id, 0.0, kp.load(), kd.load());
    std::cout << "✅ Initialization complete!" << std::endl;

    // Start control thread
    std::thread t(control_loop, s, motor_id);

    // --- Interactive main loop ---
    std::cout << "\n" << "========================================" << std::endl;
    std::cout << "👉 Enter a number (degrees) and press Enter to change position" << std::endl;
    std::cout << "👉 'kp <value>' (e.g.: kp 100) to adjust stiffness" << std::endl;
    std::cout << "👉 'kd <value>' (e.g.: kd 2.0) to adjust damping (anti-oscillation)" << std::endl;
    std::cout << "👉 '0' or 'home' to return to zero" << std::endl;
    std::cout << "👉 'q' to quit" << std::endl;
    std::cout << "⚠️  Current Kp=" << kp.load() << " | Kd=" << kd.load() << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::string line;
    while (running) {
        double current_pos_deg = target_position.load() * 180.0 / M_PI;
        printf("[%.1f°] >> ", current_pos_deg);
        std::flush(std::cout);

        if (!std::getline(std::cin, line)) {
            break;
        }

        if (line == "q" || line == "quit" || line == "exit") {
            running = false;
        } else if (line == "0" || line == "home") {
            target_position = 0.0;
            std::cout << " -> Target set: 0.0°" << std::endl;
        } else if (line.rfind("kp ", 0) == 0) {
            try {
                kp = std::stod(line.substr(3));
                std::cout << " -> Stiffness (Kp) set: " << kp.load() << std::endl;
            } catch (...) {
                std::cout << "❌ Invalid Kp. Example: kp 100.0" << std::endl;
            }
        } else if (line.rfind("kd ", 0) == 0) {
            try {
                kd = std::stod(line.substr(3));
                std::cout << " -> Damping (Kd) set: " << kd.load() << std::endl;
            } catch (...) {
                std::cout << "❌ Invalid Kd. Example: kd 2.0" << std::endl;
            }
        } else {
            try {
                double angle_deg = std::stod(line);
                angle_deg = std::max(-720.0, std::min(720.0, angle_deg));
                target_position = angle_deg * M_PI / 180.0;
                std::cout << " -> Target set: " << angle_deg << "°" << std::endl;
            } catch (...) {
                std::cout << "❌ Invalid input" << std::endl;
            }
        }
    }

    // Cleanup
    running = false;
    t.join();
    
    std::cout << "🏠 Returning to zero position..." << std::endl;
    write_operation_frame(s, motor_id, 0.0, kp.load(), kd.load());
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    std::cout << "🚫 Disabling motor..." << std::endl;
    // No disable in C++ SDK; manually send a frame with kp=0, kd=0
    write_operation_frame(s, motor_id, 0.0, 0.0, 0.0);
    
    close(s);
    std::cout << "👋 Program exited" << std::endl;
    
    return 0;
}
