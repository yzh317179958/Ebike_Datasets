#include "motor.h"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <bitset>
#include <set>
#include "crc8.h"

static const std::vector<uint8_t> header = {0x46, 0x44};

// 构造函数
Motor::Motor(serial::Serial& serial) 
    : m_serial(serial) {}

// 构建数据包
std::vector<uint8_t> Motor::build_packet(uint8_t object_id, uint8_t command, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> data_packet;
    
    // 识别标志 (0x46, 0x44)
    data_packet.push_back(0x46);
    data_packet.push_back(0x44);
    
    // 长度占位 (稍后计算)
    data_packet.push_back(0x00);
    data_packet.push_back(0x00);
    
    // 协议版本 (0x0A)
    data_packet.push_back(0x0B);
    
    // 属性部分
    data_packet.push_back(object_id);     // 对象ID
    data_packet.push_back(command);       // 命令字
    data_packet.push_back(static_cast<uint8_t>(data.size())); // 属性数据长度
    data_packet.insert(data_packet.end(), data.begin(), data.end()); // 数据
    
    // 计算长度字段
    uint16_t length = 5 + data.size();
    data_packet[2] = static_cast<uint8_t>(length >> 8);
    data_packet[3] = static_cast<uint8_t>(length & 0xFF);
    
    // 计算CRC
    std::vector<uint8_t> crc_data(data_packet.begin() + 2, data_packet.end());
    uint8_t crc = CRC8(crc_data.data(), static_cast<unsigned int>(crc_data.size()));
    data_packet.push_back(crc);
    
    return data_packet;
}

// 转把电机控制
void Motor::controlRotorMotor(uint16_t speed, int16_t position) {
    // 标记此电机已被控制
    controlled_motors.insert(0x01);
    
    speed = std::min(speed, static_cast<uint16_t>(1000));
    position = std::max(std::min(position, static_cast<int16_t>(900)), 
                       static_cast<int16_t>(-900));

    std::vector<uint8_t> rotor_motor_data;
    rotor_motor_data.push_back(static_cast<uint8_t>(speed >> 8));
    rotor_motor_data.push_back(static_cast<uint8_t>(speed & 0xFF));
    rotor_motor_data.push_back(static_cast<uint8_t>(position >> 8));
    rotor_motor_data.push_back(static_cast<uint8_t>(position & 0xFF));

    auto rotor_data_packet = build_packet(0x01, 0x01, rotor_motor_data);
    m_serial.write(rotor_data_packet);
}

// 左辅助轮电机控制
void Motor::controlLeftAuxiliaryWheelMotor(uint16_t speed, uint16_t position) {
    // 标记此电机已被控制
    controlled_motors.insert(0x02);
    
    speed = std::min(speed, static_cast<uint16_t>(1000));
    position = std::min(position, static_cast<uint16_t>(1000));

    std::vector<uint8_t> left_wheel_data;
    left_wheel_data.push_back(static_cast<uint8_t>(speed >> 8));
    left_wheel_data.push_back(static_cast<uint8_t>(speed & 0xFF));
    left_wheel_data.push_back(static_cast<uint8_t>(position >> 8));
    left_wheel_data.push_back(static_cast<uint8_t>(position & 0xFF));

    auto packet = build_packet(0x02, 0x20, left_wheel_data);
    m_serial.write(packet);
}

// 右辅助轮电机控制
void Motor::controlRightAuxiliaryWheelMotor(uint16_t speed, uint16_t position) {
    // 标记此电机已被控制
    controlled_motors.insert(0x03);
    
    speed = std::min(speed, static_cast<uint16_t>(1000));
    position = std::min(position, static_cast<uint16_t>(1000));

    std::vector<uint8_t> right_wheel_data;
    right_wheel_data.push_back(static_cast<uint8_t>(speed >> 8));
    right_wheel_data.push_back(static_cast<uint8_t>(speed & 0xFF));
    right_wheel_data.push_back(static_cast<uint8_t>(position >> 8));
    right_wheel_data.push_back(static_cast<uint8_t>(position & 0xFF));

    auto packet = build_packet(0x03, 0x20, right_wheel_data);
    m_serial.write(packet);
}

// 前刹车电机控制
void Motor::controlFrontBrakeMotor(uint16_t speed, uint16_t position) {
    // 标记此电机已被控制
    controlled_motors.insert(0x04);
    
    speed = std::min(speed, static_cast<uint16_t>(1000));
    position = std::min(position, static_cast<uint16_t>(1000));

    std::vector<uint8_t> front_brake_data;
    front_brake_data.push_back(static_cast<uint8_t>(speed >> 8));
    front_brake_data.push_back(static_cast<uint8_t>(speed & 0xFF));
    front_brake_data.push_back(static_cast<uint8_t>(position >> 8));
    front_brake_data.push_back(static_cast<uint8_t>(position & 0xFF));

    auto packet = build_packet(0x04, 0x40, front_brake_data);
    m_serial.write(packet);
}

// 后刹车电机控制
void Motor::controlRearBrakeMotor(uint16_t speed, uint16_t position) {
    // 标记此电机已被控制
    controlled_motors.insert(0x05);
    
    speed = std::min(speed, static_cast<uint16_t>(1000));
    position = std::min(position, static_cast<uint16_t>(1000));

    std::vector<uint8_t> rear_brake_data;
    rear_brake_data.push_back(static_cast<uint8_t>(speed >> 8));
    rear_brake_data.push_back(static_cast<uint8_t>(speed & 0xFF));
    rear_brake_data.push_back(static_cast<uint8_t>(position >> 8));
    rear_brake_data.push_back(static_cast<uint8_t>(position & 0xFF));

    auto packet = build_packet(0x05, 0x40, rear_brake_data);
    m_serial.write(packet);
//     std::cout << "完整数据包: ";
//     for (auto byte : packet) {
//         std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
//     }
//     std::cout << std::dec << std::endl;
}

// 轮毂电机控制
void Motor::controlHubMotor(uint8_t level, uint16_t speed, uint8_t brake) {
    // 标记此电机已被控制
    controlled_motors.insert(0x06);
    
    level = std::max(std::min(level, static_cast<uint8_t>(3)), static_cast<uint8_t>(1));
    speed = std::min(speed, static_cast<uint16_t>(1000));
    brake &= 0x03;
    
    std::vector<uint8_t> payload;
    payload.push_back(level);
    payload.push_back(static_cast<uint8_t>(speed >> 8));
    payload.push_back(static_cast<uint8_t>(speed & 0xFF));
    payload.push_back(brake);
    
    auto packet = build_packet(0x06, 0x50, payload);
    m_serial.write(packet);
}

bool Motor::tryExtractFrame(std::vector<uint8_t>& frame) {
    // 查找帧头
    auto it = std::search(m_receive_buffer.begin(), m_receive_buffer.end(), 
                         header.begin(), header.end());
    
    if (it == m_receive_buffer.end()) {
        // 如果没有找到帧头且缓冲区过大，清理缓冲区
        if (m_receive_buffer.size() > 200) {
            m_receive_buffer.clear();
        }
        return false;
    }
    
    // 计算偏移量
    size_t offset = it - m_receive_buffer.begin();
    
    // 确保有足够的数据读取长度字段
    if (m_receive_buffer.size() < offset + 4) {
        return false;
    }
    
    // 读取长度字段（大端）
    uint16_t length_field = (m_receive_buffer[offset + 2] << 8) | m_receive_buffer[offset + 3];
    
    // 长度字段合理性检查
    if (length_field < 3 || length_field > 150) {
        // 无效长度，移除帧头
        m_receive_buffer.erase(m_receive_buffer.begin() + offset, 
                              m_receive_buffer.begin() + offset + 2);
        return false;
    }
    
    size_t total_length = 4 + length_field; // 帧头(2) + 长度字段(2) + 数据
    
    if (m_receive_buffer.size() < offset + total_length) {
        return false; // 数据不完整
    }
    
    // 提取完整帧
    frame.assign(m_receive_buffer.begin() + offset, 
                m_receive_buffer.begin() + offset + total_length);
    
    // 检查协议版本
    if (frame.size() > 4 && frame[4] != 0x0B) {
        // 移除无效帧
        m_receive_buffer.erase(m_receive_buffer.begin() + offset, 
                              m_receive_buffer.begin() + offset + total_length);
        return false;
    }
    
    // 从缓冲区中移除已处理的数据
    m_receive_buffer.erase(m_receive_buffer.begin() + offset, 
                          m_receive_buffer.begin() + offset + total_length);
    
    // std::cout << "接收缓冲区数据 (大小=" << m_receive_buffer.size() << "): ";
    // for (uint8_t byte : m_receive_buffer) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
    //               << static_cast<int>(byte) << " ";
    // }
    // std::cout << std::dec << std::endl;

    return true;
}

// 处理接收到的帧
bool Motor::processReceivedFrame() {
    m_receive_buffer.clear();
    // 读取串口数据
    size_t available = m_serial.available();
    if (available > 0) {
        std::vector<uint8_t> data(available);
        size_t read = m_serial.read(data.data(), available);
        // 直接输出原始接收数据
        // std::cout << "原始接收数据: ";
        // for (uint8_t byte : data) {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
        //               << static_cast<int>(byte) << " ";
        // }
        std::cout << std::dec << std::endl;
        m_receive_buffer.insert(m_receive_buffer.end(), data.begin(), data.begin() + read);
    }
    
    // 限制每周期处理帧数
    const int MAX_FRAMES_PER_CYCLE = 20;
    int framesProcessed = 0;
    bool allFramesValid = true;
    

    // 尝试提取并处理帧
    std::vector<uint8_t> frame;
    while (framesProcessed < MAX_FRAMES_PER_CYCLE && tryExtractFrame(frame)) {
        // 输出原始帧数据
        // std::cout << "接收帧: ";
        // for (uint8_t byte : frame) {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
        //               << static_cast<int>(byte) << " ";
        // }
        // std::cout << std::dec << std::endl;
        
        // 处理帧
        if (!processFrame(frame)) {
            allFramesValid = false;
        }
        
        frame.clear(); // 清除当前帧
        framesProcessed++;
    }
    
    // 如果缓冲区中仍有数据但无法形成完整帧，清理缓冲区
    if (!m_receive_buffer.empty() && framesProcessed == 0) {
        // 如果残留数据过多，清理缓冲区
        if (m_receive_buffer.size() > 50) {
            m_receive_buffer.clear();
        }
    }
    
    return allFramesValid;
}

// 处理单个帧
bool Motor::processFrame(const std::vector<uint8_t>& frame) {
    // 最小帧长检查
    if (frame.size() < 9) {
        return false;
    }
    
    // 检查帧头
    if (frame[0] != 0x46 || frame[1] != 0x44) {
        return false;
    }
    
    // 读取长度字段（大端）
    uint16_t length_field = (frame[2] << 8) | frame[3];
    
    // 长度字段合理性检查
    if (length_field < 3 || length_field > 150) {
        return false;
    }
    
    // 计算期望的总帧长
    size_t expected_length = 4 + length_field;
    
    if (frame.size() < expected_length) {
        return false;
    }
    
    // 检查CRC
    size_t crc_start_index = 2;
    size_t crc_end_index = 2 + length_field + 1;
    
    if (crc_end_index >= frame.size()) {
        return false;
    }
    
    std::vector<uint8_t> crc_data(frame.begin() + crc_start_index, 
                                 frame.begin() + crc_end_index);
    
    uint8_t calculated_crc = CRC8(crc_data.data(), crc_data.size());
    uint8_t received_crc = frame[crc_end_index];
    
    if (calculated_crc != received_crc) {
        return false;
    }
    
    // 提取关键字段
    uint8_t protocol_version = frame[4];
    uint8_t object_id = frame[5];
    uint8_t command = frame[6];
    uint8_t attr_len = frame[7];
    
    // 检查属性数据长度
    size_t attr_end_index = 8 + attr_len;
    if (expected_length < attr_end_index + 1) {
        return false;
    }
    
    // 处理应答帧
    if (command == 0x01 || command == 0x20 || command == 0x40 || command == 0x50) {
        if (attr_len != 0) {
            return false;
        }
        
        // // 输出应答帧信息
        // std::cout << "收到有效应答: 对象ID " << static_cast<int>(object_id)
        //           << ", 命令字 " << static_cast<int>(command) << std::endl;
        return true;
    }
    // 处理实时参数帧
    else if (command == 0x10 || command == 0x30 || command == 0x60) {
        if (attr_len == 0) {
            return false;
        }
        
        std::vector<uint8_t> data(frame.begin() + 8, frame.begin() + 8 + attr_len);
        
        switch (command) {
            case 0x10: // 转把电机
                parseRotorRealtime(data);
                break;
            case 0x30: // 辅助轮
                // 根据对象ID判断是左还是右辅助轮
                if (object_id == 0x02) {
                    parseAuxWheelRealtime(data, true); // true表示左辅助轮
                } else if (object_id == 0x03) {
                    parseAuxWheelRealtime(data, false); // false表示右辅助轮
                }
                break;
            case 0x60: // 轮毂电机
                parseHubMotorRealtime(data);
                break;
            default:
                return false;
        }
        return true;
    }
    
    return false;
}


// 解析转把电机实时参数
void Motor::parseRotorRealtime(const std::vector<uint8_t>& data) {
    if (data.size() < 14) return;
    
    // 更新转把电机状态
    rotor_state_.voltage = (data[0] << 8) | data[1];
    rotor_state_.motor_status = data[2];
    rotor_state_.speed = static_cast<int16_t>((data[3] << 8) | data[4]);
    rotor_state_.torque = static_cast<float>((data[5] << 8) | data[6]) / 10.0f;
    rotor_state_.position = (static_cast<int32_t>(data[7]) << 24) |
                           (static_cast<int32_t>(data[8]) << 16) |
                           (static_cast<int32_t>(data[9]) << 8) |
                           static_cast<int32_t>(data[10]);
    rotor_state_.fault_code = (data[11] << 8) | data[12];
    rotor_state_.error_code = data[13];
}

// 解析辅助轮实时参数
void Motor::parseAuxWheelRealtime(const std::vector<uint8_t>& data, bool is_left) {
    if (data.size() < 12) return;
    
    // 根据左右辅助轮更新相应状态
    AuxWheelMotorState& state = is_left ? left_aux_state_ : right_aux_state_;
    
    uint16_t position_raw = (data[0] << 8) | data[1];
    state.position = (position_raw & 0x7FFF) * 0.087f;
    if (position_raw & 0x8000) state.position = -state.position;
    
    uint16_t speed_raw = (data[2] << 8) | data[3];
    state.speed = (speed_raw & 0x7FFF) * 0.732f;
    if (speed_raw & 0x8000) state.speed = -state.speed;
    
    uint16_t load_raw = (data[4] << 8) | data[5];
    state.load = (load_raw & 0x3FF) / 10.0f;
    if (load_raw & 0x400) state.load = -state.load;
    
    state.voltage = data[6];
    state.temperature = data[7];
    state.status = data[8];
    state.moving_flag = data[9];
    state.current = (data[10] << 8) | data[11];
}

// 解析轮毂电机实时参数
void Motor::parseHubMotorRealtime(const std::vector<uint8_t>& data) {
    if (data.size() < 3) return;
    
    // 更新轮毂电机状态
    hub_state_.speed = (data[0] << 8) | data[1];
    hub_state_.error_code = data[2];
}