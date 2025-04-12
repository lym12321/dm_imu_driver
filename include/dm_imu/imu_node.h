#include <cstdint>

struct dm_imu_data_t {    
    struct dm_imu_header_t {
        uint8_t sof = 0x55, sign = 0xAA;
        uint8_t slave_id, reg_id;
    } __attribute__((packed));

    struct dm_imu_accel_pkg_t {
        dm_imu_header_t header;
        float x, y, z;
        uint16_t crc;
        uint8_t tail = 0x0A;
    } __attribute__((packed));

    struct dm_imu_gyro_pkg_t {
        dm_imu_header_t header;
        float x, y, z;
        uint16_t crc;
        uint8_t tail = 0x0A;
    } __attribute__((packed));

    struct dm_imu_euler_pkg_t {
        dm_imu_header_t header;
        float r, p, y;
        uint16_t crc;
        uint8_t tail = 0x0A;
    } __attribute__((packed));

    struct {
        dm_imu_accel_pkg_t accel;
        dm_imu_gyro_pkg_t gyro;
        dm_imu_euler_pkg_t euler;
    } __attribute__((packed)) pkg;
};