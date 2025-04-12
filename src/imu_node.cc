#include <iostream>
#include <cstdio>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include "imu_node.h"

using namespace drivers::common;
using namespace drivers::serial_driver;

static dm_imu_data_t imu_data_;

static const uint16_t CRC16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129,
    0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318,
    0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B,
    0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A,
    0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED,
    0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
    0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F,
    0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE,
    0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1,
    0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290,
    0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3,
    0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2,
    0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865,
    0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54,
    0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07,
    0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36,
    0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

static uint16_t Get_CRC16(const uint8_t *ptr, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        uint8_t index = (crc >> 8 ^ ptr[i]);
        crc = ((crc << 1) ^ CRC16_table[index]);
    }
    return crc;
}

struct Quaternion
{
    double w, x, y, z;
};
 
std::tuple <double, double, double, double> ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
 
    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
 
    return std::make_tuple(q.w, q.x, q.y, q.z);
}

class imu_node : public rclcpp::Node {
public:
    imu_node(std::string path) : Node("imu_node"), path_(path) {
        try {
            // init serial
            io_context_ = std::make_shared <IoContext> (1);
            serial_driver_ = std::make_shared <SerialDriver> (*io_context_);

            conn(path);

            // init publisher
            publisher_ = this->create_publisher <sensor_msgs::msg::Imu> ("imu", 10);
            
            // init timer
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&imu_node::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "imu_node inited");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            return;
        }
    };
    ~imu_node() {
        this->serial_driver_->port()->close();   
        RCLCPP_INFO(this->get_logger(), "imu_node closed"); 
    }
private:
    std::shared_ptr <SerialDriver> serial_driver_;
    std::shared_ptr <IoContext> io_context_;
    std::shared_ptr <rclcpp::TimerBase> timer_;
    std::string path_;

    rclcpp::Publisher <sensor_msgs::msg::Imu> ::SharedPtr publisher_;

    bool conn(std::string path, bool debug = false) {
        SerialPortConfig cfg(
            2000000,
            FlowControl::NONE,
            Parity::NONE,
            StopBits::ONE 
        );

        auto fp = popen(("ls " + path).c_str(), "r");
        if(fp) {
            std::array <char, 25> buf;
            fscanf(fp, "%s", buf.begin());
            if(buf[0] == '/') {
                path.assign(buf.begin(), buf.end());
            }
            pclose(fp);
        }

        RCLCPP_INFO(this->get_logger(), "%s", path.c_str());

        serial_driver_->init_port(path, cfg);
        
        auto port = serial_driver_->port();

        try {
            port->open();
        } catch(const std::exception &e) {
            if(debug)
                RCLCPP_ERROR(this->get_logger(), "error opening serial port: %s", e.what());
        }

        if(port->is_open()) {
            port->async_receive([this](const std::vector<uint8_t> &data, const size_t &size) {
                this->rx_callback(data, size);
            });
        } else {
            return false;
        }
        return true;
    }

    // 19 bytes
    void rx_solve(const uint8_t *buf) {
        static_assert(sizeof(imu_data_.pkg.accel) == 19);
        static_assert(sizeof(imu_data_.pkg.gyro) == 19);
        static_assert(sizeof(imu_data_.pkg.euler) == 19);
        switch(buf[3]) {
            case 0x01:
                std::copy_n(buf, 19, reinterpret_cast<uint8_t*>(&imu_data_.pkg.accel));
                break;
            case 0x02:
                std::copy_n(buf, 19, reinterpret_cast<uint8_t*>(&imu_data_.pkg.gyro));
                break;
            case 0x03:
                std::copy_n(buf, 19, reinterpret_cast<uint8_t*>(&imu_data_.pkg.euler));
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown id: %x", buf[3]);
                return;
        }
    }

    void rx_callback(const std::vector<uint8_t> &data, const size_t &size) {
        for(size_t i = 0; i < size; i++) {
            if(data[i] == 0x55 && data[i+1] == 0xAA && data[i+18] == 0x0A) {
                if(Get_CRC16(data.data() + i, 16) == *reinterpret_cast <const uint16_t*> (data.data() + i + 16)) {
                    rx_solve(data.data() + i);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "CRC ERROR");
                }
            }
        }
    }

    void timer_callback() {
        if(!serial_driver_->port()->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "attempt to re-connect");
            conn(path_);
            return;
        }
        try {
            serial_driver_->port()->send_break();
        } catch(const std::system_error &e) {
            RCLCPP_ERROR(this->get_logger(), "attempt to re-connect");
            conn(path_);
        }

        sensor_msgs::msg::Imu tmp;
        tmp.header.stamp = this->get_clock()->now();
        tmp.header.set__frame_id("map");
        tmp.angular_velocity.x = imu_data_.pkg.gyro.x;
        tmp.angular_velocity.y = imu_data_.pkg.gyro.y;
        tmp.angular_velocity.z = imu_data_.pkg.gyro.z;
        tmp.linear_acceleration.x = imu_data_.pkg.accel.x;
        tmp.linear_acceleration.y = imu_data_.pkg.accel.y;
        tmp.linear_acceleration.z = imu_data_.pkg.accel.z;
        std::tie(tmp.orientation.w, tmp.orientation.x, tmp.orientation.y, tmp.orientation.z) = ToQuaternion (
            imu_data_.pkg.euler.y / 180 * M_PI, imu_data_.pkg.euler.p / 180 * M_PI, imu_data_.pkg.euler.r / 180 * M_PI
        );
        publisher_->publish(tmp);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared <imu_node> ("/dev/ttyACM*"));
    rclcpp::shutdown();
    return 0;
}