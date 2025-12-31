// esp32_systemc_model.cpp
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include <random>
#include <chrono>

using namespace sc_core;
using namespace tlm;
using namespace std;

// ================================================================
// Constants and Configuration
// ================================================================
struct SystemConfig {
    // WiFi parameters
    static constexpr double WIFI_BANDWIDTH = 20.0; // Mbps
    static constexpr double WIFI_LATENCY = 5.0;    // ms
    static constexpr double PACKET_LOSS_RATE = 0.001; // 0.1%
    
    // Servo parameters
    static constexpr int NUM_SERVOS = 6;
    static constexpr double SERVO_UPDATE_RATE = 50.0; // Hz
    
    // Sensor parameters
    static constexpr double MPU_UPDATE_RATE = 100.0; // Hz
    static constexpr double FLEX_UPDATE_RATE = 50.0; // Hz
};

// ================================================================
// Network Packet Structure
// ================================================================
struct NetworkPacket {
    char command;
    sc_time timestamp;
    uint32_t sequence_number;
    bool is_ack;
    
    NetworkPacket(char cmd = 0) : command(cmd), timestamp(sc_time_stamp()) {
        static uint32_t counter = 0;
        sequence_number = counter++;
        is_ack = false;
    }
};

// ================================================================
// WiFi Channel Model
// ================================================================
class WiFiChannel : public sc_module {
public:
    tlm_utils::simple_target_socket<WiFiChannel> rx_socket;
    tlm_utils::simple_initiator_socket<WiFiChannel> tx_socket;
    
    SC_HAS_PROCESS(WiFiChannel);
    
    WiFiChannel(sc_module_name name, 
                double bandwidth = SystemConfig::WIFI_BANDWIDTH,
                double latency = SystemConfig::WIFI_LATENCY,
                double loss_rate = SystemConfig::PACKET_LOSS_RATE)
        : sc_module(name), bandwidth_Mbps(bandwidth), 
          latency_ms(latency), packet_loss_rate(loss_rate) {
        
        rx_socket.register_b_transport(this, &WiFiChannel::b_transport);
        SC_THREAD(channel_process);
        
        // Initialize random generator for packet loss
        random_engine.seed(time(NULL));
    }
    
private:
    double bandwidth_Mbps;
    double latency_ms;
    double packet_loss_rate;
    queue<NetworkPacket> tx_queue;
    queue<NetworkPacket> rx_queue;
    sc_event packet_event;
    std::default_random_engine random_engine;
    
    void b_transport(tlm_generic_payload &trans, sc_time &delay) {
        unsigned char *data = trans.get_data_ptr();
        NetworkPacket *packet = reinterpret_cast<NetworkPacket*>(data);
        
        if (trans.get_command() == TLM_WRITE_COMMAND) {
            // Simulate packet loss
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            if (dist(random_engine) > packet_loss_rate) {
                tx_queue.push(*packet);
                packet_event.notify();
            } else {
                cout << "[WiFi] Packet lost! Seq: " << packet->sequence_number 
                     << " at " << sc_time_stamp() << endl;
            }
        }
        
        delay += sc_time(latency_ms, SC_MS);
    }
    
    void channel_process() {
        while (true) {
            wait(packet_event);
            
            if (!tx_queue.empty()) {
                NetworkPacket packet = tx_queue.front();
                tx_queue.pop();
                
                // Calculate transmission time based on packet size and bandwidth
                double packet_size_bits = 8.0 * sizeof(NetworkPacket); // bits
                double tx_time_ms = (packet_size_bits / (bandwidth_Mbps * 1e6)) * 1e3;
                
                wait(sc_time(tx_time_ms, SC_MS));
                wait(sc_time(latency_ms, SC_MS));
                
                // Forward packet
                tlm_generic_payload trans;
                unsigned char *data = new unsigned char[sizeof(NetworkPacket)];
                memcpy(data, &packet, sizeof(NetworkPacket));
                trans.set_data_ptr(data);
                trans.set_data_length(sizeof(NetworkPacket));
                
                sc_time delay = SC_ZERO_TIME;
                tx_socket->b_transport(trans, delay);
                
                delete[] data;
            }
        }
    }
};

// ================================================================
// MPU6050 Sensor Model
// ================================================================
class MPU6050 : public sc_module {
public:
    sc_event data_ready;
    
    SC_HAS_PROCESS(MPU6050);
    
    MPU6050(sc_module_name name, uint8_t i2c_addr = 0x68)
        : sc_module(name), address(i2c_addr), 
          yaw(0.0), pitch(0.0), roll(0.0) {
        SC_THREAD(sensor_thread);
        
        // Initialize random motion
        random_engine.seed(time(NULL) + address);
        motion_dist = std::uniform_real_distribution<double>(-10.0, 10.0);
    }
    
    void get_data(double &y, double &p, double &r) {
        y = yaw;
        p = pitch;
        r = roll;
    }
    
private:
    uint8_t address;
    double yaw, pitch, roll;
    std::default_random_engine random_engine;
    std::uniform_real_distribution<double> motion_dist;
    
    void sensor_thread() {
        double dt = 1000.0 / SystemConfig::MPU_UPDATE_RATE; // ms
        
        while (true) {
            wait(sc_time(dt, SC_MS));
            
            // Simulate realistic motion with some noise
            yaw += motion_dist(random_engine) * 0.1;
            pitch += motion_dist(random_engine) * 0.1;
            roll += motion_dist(random_engine) * 0.1;
            
            // Keep angles in reasonable range
            yaw = fmod(yaw, 360.0);
            pitch = fmod(pitch, 180.0);
            roll = fmod(roll, 360.0);
            
            data_ready.notify();
        }
    }
};

// ================================================================
// Flex Sensor Model
// ================================================================
class FlexSensor : public sc_module {
public:
    sc_event value_changed;
    
    SC_HAS_PROCESS(FlexSensor);
    
    FlexSensor(sc_module_name name, int adc_channel)
        : sc_module(name), channel(adc_channel), value(0) {
        SC_THREAD(sensor_thread);
        
        random_engine.seed(time(NULL) + channel);
        flex_dist = std::uniform_int_distribution<int>(0, 4095);
    }
    
    int get_value() { return value; }
    
private:
    int channel;
    int value;
    std::default_random_engine random_engine;
    std::uniform_int_distribution<int> flex_dist;
    
    void sensor_thread() {
        double dt = 1000.0 / SystemConfig::FLEX_UPDATE_RATE; // ms
        
        while (true) {
            wait(sc_time(dt, SC_MS));
            
            // Simulate flex sensor reading with some noise
            int new_value = flex_dist(random_engine);
            if (abs(new_value - value) > 50) { // Significant change
                value = new_value;
                value_changed.notify();
            }
        }
    }
};

// ================================================================
// Hand Controller (AP Mode)
// ================================================================
class HandController : public sc_module {
public:
    tlm_utils::simple_initiator_socket<HandController> wifi_socket;
    
    SC_HAS_PROCESS(HandController);
    
    HandController(sc_module_name name) : sc_module(name) {
        // Initialize sensors
        mpu68 = new MPU6050("MPU68", 0x68);
        mpu69 = new MPU6050("MPU69", 0x69);
        flex1 = new FlexSensor("Flex1", 35);
        flex2 = new FlexSensor("Flex2", 34);
        
        SC_THREAD(controller_thread);
        
        // Initialize command mapping (from testhand.c)
        init_command_mapping();
    }
    
    ~HandController() {
        delete mpu68;
        delete mpu69;
        delete flex1;
        delete flex2;
    }
    
private:
    MPU6050 *mpu68;
    MPU6050 *mpu69;
    FlexSensor *flex1;
    FlexSensor *flex2;
    
    map<char, string> command_desc;
    char last_yaw_char = 'H';
    char last_pitch_char = 'A';
    char last_flex2_char = '5';
    char last_roll_char = 'X';
    char last_pitch69_char = 'U';
    char last_flex1_char = '1';
    
    void init_command_mapping() {
        command_desc['L'] = "Yaw left >60";
        command_desc['l'] = "Yaw left >30";
        command_desc['H'] = "Yaw home";
        command_desc['r'] = "Yaw right >30";
        command_desc['R'] = "Yaw right >60";
        command_desc['A'] = "Pitch -20 to 5";
        command_desc['B'] = "Pitch 20-40";
        command_desc['C'] = "Pitch 40-55";
        command_desc['D'] = "Pitch 55-90";
        command_desc['5'] = "Flex2 <1500";
        command_desc['6'] = "Flex2 1500-1600";
        command_desc['7'] = "Flex2 >1600";
        command_desc['X'] = "Roll ±160-180";
        command_desc['Y'] = "Roll 120-150";
        command_desc['Z'] = "Roll 61-119";
        command_desc['W'] = "Roll 21-60";
        command_desc['V'] = "Roll 0-20";
        command_desc['P'] = "Pitch69 >+40";
        command_desc['Q'] = "Pitch69 >+20";
        command_desc['U'] = "Pitch69 ±20";
        command_desc['T'] = "Pitch69 <-20";
        command_desc['S'] = "Pitch69 <-40";
        command_desc['1'] = "Flex1 <100";
        command_desc['2'] = "Flex1 100-400";
        command_desc['3'] = "Flex1 400-800";
        command_desc['4'] = "Flex1 >800";
    }
    
    // Mapping functions (simplified from testhand.c)
    char get_yaw_char(double yaw) {
        if (yaw < -60) return 'L';
        if (yaw < -30) return 'l';
        if (yaw > 60) return 'R';
        if (yaw > 30) return 'r';
        return 'H';
    }
    
    char get_pitch_char(double pitch) {
        if (pitch >= -20 && pitch <= 5) return 'A';
        if (pitch >= 20 && pitch <= 40) return 'B';
        if (pitch >= 40 && pitch <= 55) return 'C';
        if (pitch >= 55 && pitch <= 90) return 'D';
        return last_pitch_char;
    }
    
    char get_roll_char(double roll) {
        if (roll < 0) roll += 360;
        
        if ((roll >= 0 && roll <= 20) || (roll >= 340 && roll <= 360)) return 'V';
        if (roll >= 21 && roll <= 60) return 'W';
        if (roll >= 61 && roll <= 119) return 'Z';
        if (roll >= 120 && roll <= 150) return 'Y';
        if ((roll >= 151 && roll <= 200) || (roll >= 300 && roll <= 339)) return 'X';
        return last_roll_char;
    }
    
    char get_flex_char(int value, bool is_flex1) {
        if (is_flex1) {
            if (value < 100) return '1';
            if (value < 400) return '2';
            if (value < 800) return '3';
            return '4';
        } else {
            if (value < 1500) return '5';
            if (value < 1600) return '6';
            return '7';
        }
    }
    
    void send_command(char cmd) {
        NetworkPacket packet(cmd);
        tlm_generic_payload trans;
        unsigned char *data = new unsigned char[sizeof(NetworkPacket)];
        memcpy(data, &packet, sizeof(NetworkPacket));
        trans.set_data_ptr(data);
        trans.set_data_length(sizeof(NetworkPacket));
        trans.set_command(TLM_WRITE_COMMAND);
        
        sc_time delay = SC_ZERO_TIME;
        wifi_socket->b_transport(trans, delay);
        
        cout << "[HandCtrl] Sent: '" << cmd << "' - " << command_desc[cmd] 
             << " at " << sc_time_stamp() << endl;
        
        delete[] data;
    }
    
    void controller_thread() {
        double mpu68_yaw, mpu68_pitch, mpu68_roll;
        double mpu69_yaw, mpu69_pitch, mpu69_roll;
        
        while (true) {
            wait(sc_time(40, SC_MS)); // Main loop delay (from testhand.c)
            
            // Read sensor data
            mpu68->get_data(mpu68_yaw, mpu68_pitch, mpu68_roll);
            mpu69->get_data(mpu69_yaw, mpu69_pitch, mpu69_roll);
            int flex1_val = flex1->get_value();
            int flex2_val = flex2->get_value();
            
            // Determine commands
            char yaw_cmd = get_yaw_char(mpu68_yaw);
            char pitch_cmd = get_pitch_char(mpu68_pitch);
            char roll_cmd = get_roll_char(mpu69_roll);
            char flex1_cmd = get_flex_char(flex1_val, true);
            char flex2_cmd = get_flex_char(flex2_val, false);
            
            // Send commands if changed
            if (yaw_cmd != last_yaw_char) {
                send_command(yaw_cmd);
                last_yaw_char = yaw_cmd;
                wait(sc_time(15, SC_MS)); // Inter-command delay
            }
            
            if (pitch_cmd != last_pitch_char) {
                send_command(pitch_cmd);
                last_pitch_char = pitch_cmd;
                wait(sc_time(15, SC_MS));
            }
            
            if (flex2_cmd != last_flex2_char) {
                send_command(flex2_cmd);
                last_flex2_char = flex2_cmd;
                wait(sc_time(15, SC_MS));
            }
            
            if (roll_cmd != last_roll_char) {
                send_command(roll_cmd);
                last_roll_char = roll_cmd;
                wait(sc_time(15, SC_MS));
            }
            
            if (flex1_cmd != last_flex1_char) {
                send_command(flex1_cmd);
                last_flex1_char = flex1_cmd;
                wait(sc_time(15, SC_MS));
            }
        }
    }
};

// ================================================================
// Servo Controller (STA Mode)
// ================================================================
class ServoController : public sc_module {
public:
    tlm_utils::simple_target_socket<ServoController> wifi_socket;
    
    SC_HAS_PROCESS(ServoController);
    
    ServoController(sc_module_name name) : sc_module(name) {
        wifi_socket.register_b_transport(this, &ServoController::b_transport);
        SC_THREAD(servo_update_thread);
        
        // Initialize servo angles (from testarm.c)
        current_angles[0] = 90;   // Yaw home
        current_angles[1] = 180;  // Pitch A
        current_angles[2] = 60;   // Flex2 5
        current_angles[3] = 0;    // Unused
        current_angles[4] = 0;    // Roll X
        current_angles[5] = 90;   // Pitch69 U
        current_angles[6] = 180;  // Flex1 1
        
        init_angle_mapping();
    }
    
private:
    int current_angles[16]; // Support up to 16 servos
    
    map<char, int> yaw_angles;
    map<char, int> pitch_angles;
    map<char, int> roll_angles;
    map<char, int> pitch69_angles;
    map<char, int> flex1_angles;
    map<char, int> flex2_angles;
    
    void init_angle_mapping() {
        // From testarm.c lookup tables
        yaw_angles['L'] = 10;
        yaw_angles['l'] = 50;
        yaw_angles['H'] = 90;
        yaw_angles['r'] = 140;
        yaw_angles['R'] = 170;
        
        pitch_angles['A'] = 180;
        pitch_angles['B'] = 150;
        pitch_angles['C'] = 120;
        pitch_angles['D'] = 90;
        
        roll_angles['X'] = 0;
        roll_angles['Y'] = 45;
        roll_angles['Z'] = 90;
        roll_angles['W'] = 135;
        roll_angles['V'] = 180;
        
        pitch69_angles['P'] = 180;
        pitch69_angles['Q'] = 135;
        pitch69_angles['U'] = 90;
        pitch69_angles['T'] = 45;
        pitch69_angles['S'] = 0;
        
        flex1_angles['1'] = 180;
        flex1_angles['2'] = 150;
        flex1_angles['3'] = 120;
        flex1_angles['4'] = 90;
        
        flex2_angles['5'] = 60;
        flex2_angles['6'] = 120;
        flex2_angles['7'] = 180;
    }
    
    void b_transport(tlm_generic_payload &trans, sc_time &delay) {
        unsigned char *data = trans.get_data_ptr();
        NetworkPacket *packet = reinterpret_cast<NetworkPacket*>(data);
        
        if (trans.get_command() == TLM_WRITE_COMMAND) {
            process_command(packet->command);
            
            // Calculate network latency
            sc_time latency = sc_time_stamp() - packet->timestamp;
            cout << "[ServoCtrl] Recv: '" << packet->command 
                 << "' Latency: " << latency.to_seconds() * 1000 << " ms" << endl;
        }
        
        delay = SC_ZERO_TIME;
    }
    
    void process_command(char cmd) {
        int servo_channel = -1;
        int target_angle = -1;
        
        // MPU68 Yaw (Servo 0)
        if (cmd == 'L' || cmd == 'l' || cmd == 'H' || cmd == 'r' || cmd == 'R') {
            servo_channel = 0;
            target_angle = yaw_angles[cmd];
        }
        // MPU68 Pitch (Servo 1)
        else if (cmd == 'A' || cmd == 'B' || cmd == 'C' || cmd == 'D') {
            servo_channel = 1;
            target_angle = pitch_angles[cmd];
        }
        // Flex2 (Servo 2)
        else if (cmd == '5' || cmd == '6' || cmd == '7') {
            servo_channel = 2;
            target_angle = flex2_angles[cmd];
        }
        // MPU69 Roll (Servo 4)
        else if (cmd == 'X' || cmd == 'Y' || cmd == 'Z' || cmd == 'W' || cmd == 'V') {
            servo_channel = 4;
            target_angle = roll_angles[cmd];
        }
        // MPU69 Pitch (Servo 5)
        else if (cmd == 'P' || cmd == 'Q' || cmd == 'U' || cmd == 'T' || cmd == 'S') {
            servo_channel = 5;
            target_angle = pitch69_angles[cmd];
        }
        // Flex1 (Servo 6)
        else if (cmd == '1' || cmd == '2' || cmd == '3' || cmd == '4') {
            servo_channel = 6;
            target_angle = flex1_angles[cmd];
        }
        
        if (servo_channel != -1 && target_angle != -1) {
            // Simulate servo movement
            int current = current_angles[servo_channel];
            if (current != target_angle) {
                // Calculate movement time (approx 60°/second)
                double angle_diff = abs(target_angle - current);
                double move_time_ms = (angle_diff / 60.0) * 1000.0;
                
                wait(sc_time(move_time_ms, SC_MS));
                
                current_angles[servo_channel] = target_angle;
                
                cout << "[ServoCtrl] Servo " << servo_channel 
                     << " moved: " << current << "° -> " << target_angle 
                     << "° (took " << move_time_ms << " ms)" << endl;
            }
        }
    }
    
    void servo_update_thread() {
        // Background thread for servo maintenance
        while (true) {
            wait(sc_time(1000, SC_MS)); // Check every second
            
            // Print current servo status
            cout << "[ServoCtrl] Status - S0:" << current_angles[0]
                 << " S1:" << current_angles[1]
                 << " S2:" << current_angles[2]
                 << " S4:" << current_angles[4]
                 << " S5:" << current_angles[5]
                 << " S6:" << current_angles[6] << endl;
        }
    }
};

// ================================================================
// Power Monitor
// ================================================================
class PowerMonitor : public sc_module {
public:
    SC_HAS_PROCESS(PowerMonitor);
    
    PowerMonitor(sc_module_name name) : sc_module(name) {
        SC_THREAD(monitor_thread);
        total_energy = 0.0;
        last_update = SC_ZERO_TIME;
    }
    
    double get_total_energy() const { return total_energy; }
    
private:
    double total_energy; // Joules
    sc_time last_update;
    
    void monitor_thread() {
        // Power model parameters (approximate)
        double handctrl_power = 0.15; // W (ESP32 + sensors)
        double servoctrl_power = 0.20; // W (ESP32 + PCA9685 + servos)
        double wifi_tx_power = 0.10; // W during transmission
        double wifi_rx_power = 0.08; // W during reception
        
        double current_power = handctrl_power + servoctrl_power;
        
        while (true) {
            wait(sc_time(100, SC_MS)); // Update every 100ms
            
            sc_time now = sc_time_stamp();
            sc_time delta = now - last_update;
            
            if (last_update != SC_ZERO_TIME) {
                double delta_seconds = delta.to_seconds();
                total_energy += current_power * delta_seconds;
            }
            
            last_update = now;
            
            cout << "[Power] Current: " << current_power << " W, "
                 << "Total: " << total_energy << " J at " << now << endl;
        }
    }
};

// ================================================================
// Testbench
// ================================================================
int sc_main(int argc, char* argv[]) {
    cout << "==================================================" << endl;
    cout << "ESP32 Hand-Arm SystemC Simulation" << endl;
    cout << "==================================================" << endl;
    
    // Create modules
    HandController handctrl("HandController");
    ServoController servoctrl("ServoController");
    WiFiChannel wifi("WiFiChannel");
    PowerMonitor power("PowerMonitor");
    
    // Bind connections
    handctrl.wifi_socket.bind(wifi.rx_socket);
    wifi.tx_socket.bind(servoctrl.wifi_socket);
    
    // Start simulation
    sc_start(10.0, SC_SEC); // Simulate for 10 seconds
    
    // Print summary
    cout << "\n==================================================" << endl;
    cout << "Simulation Complete" << endl;
    cout << "==================================================" << endl;
    cout << "Total Energy Consumed: " << power.get_total_energy() << " J" << endl;
    cout << "Average Power: " << power.get_total_energy() / 10.0 << " W" << endl;
    
    return 0;
}