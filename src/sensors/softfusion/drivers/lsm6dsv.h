#pragma once

#include <cstdint>
#include <array>
#include <algorithm>

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

// Driver uses acceleration range at 8g
// and gyroscope range at 1000dps
// Gyroscope ODR = 480Hz, accel ODR = 480Hz

template <typename I2CImpl>
struct LSM6DSV
{
    static constexpr uint8_t Address = 0x6a;
    static constexpr auto Name = "LSM6DSV";
    static constexpr auto Type = ImuID::LSM6DSV;

    static constexpr float Freq = 480;

    static constexpr float GyrTs=1.0/Freq;
    static constexpr float AccTs=1.0/Freq;
    static constexpr float MagTs=1.0/Freq;

    static constexpr float GyroSensitivity = 28.571428571f;
    static constexpr float AccelSensitivity = 4098.360655738f;

    I2CImpl i2c;
    LSM6DSV(I2CImpl i2c)
    : i2c(i2c) {}

    struct Regs {
        struct WhoAmI {
            static constexpr uint8_t reg = 0x0f;
            static constexpr uint8_t value = 0x70;
        };
        static constexpr uint8_t OutTemp = 0x20;
        struct Ctrl1XLODR {
            static constexpr uint8_t reg = 0x10;
            static constexpr uint8_t value = (0b1000); //480Hz
        };
        struct Ctrl2GODR {
            static constexpr uint8_t reg = 0x11;
            static constexpr uint8_t value = (0b1000 << 4); //480Hz
        };
        struct Ctrl3C {
            static constexpr uint8_t reg = 0x12;
            static constexpr uint8_t valueSwReset = 1;
            static constexpr uint8_t value = (1 << 6) | (1 << 2); //BDU = 1, IF_INC = 1
        };
        struct Ctrl6GFS {
            static constexpr uint8_t reg = 0x15;
            static constexpr uint8_t value = (0b0011); //1000dps
        };
        struct Ctrl8XLFS {
            static constexpr uint8_t reg = 0x17;
            static constexpr uint8_t value = (0b10); //8g
        };
        struct FifoCtrl3BDR {
            static constexpr uint8_t reg = 0x09;
            static constexpr uint8_t value = (0b1000) | (0b1000 << 4); //gyro and accel batched at 480Hz
        };
        struct FifoCtrl4Mode {
            static constexpr uint8_t reg = 0x0a;
            static constexpr uint8_t value = (0b110); //continuous mode
        };

        static constexpr uint8_t FifoStatus = 0x1b;
        static constexpr uint8_t FifoData = 0x78;
    };

    bool initialize()
    {
        // perform initialization step
        i2c.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
        delay(20);
        i2c.writeReg(Regs::Ctrl1XLODR::reg, Regs::Ctrl1XLODR::value);
        i2c.writeReg(Regs::Ctrl2GODR::reg, Regs::Ctrl2GODR::value);
        i2c.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);
        i2c.writeReg(Regs::Ctrl6GFS::reg, Regs::Ctrl6GFS::value);
        i2c.writeReg(Regs::Ctrl8XLFS::reg, Regs::Ctrl8XLFS::value);
        i2c.writeReg(Regs::FifoCtrl3BDR::reg, Regs::FifoCtrl3BDR::value);
        i2c.writeReg(Regs::FifoCtrl4Mode::reg, Regs::FifoCtrl4Mode::value);
        return true;
    }

    float getDirectTemp() const
    {
        const auto value = static_cast<int16_t>(i2c.readReg16(Regs::OutTemp));
        float result = ((float)value / 256.0f) + 25.0f;

        return result;
    }

    struct FifoData {
        uint8_t tag;
        uint16_t x;
        uint16_t y;
        uint16_t z;
    };

    template <typename AccelCall, typename GyroCall>
    void bulkRead(AccelCall &&processAccelSample, GyroCall &&processGyroSample) {
        const auto read_result = i2c.readReg16(Regs::FifoStatus);
        if (read_result & 0x4000) { // overrun!
            // disable and re-enable fifo to clear it
            printf("Fifo overrun, resetting\n");
            i2c.writeReg(Regs::FifoCtrl4Mode::reg, 0);
            i2c.writeReg(Regs::FifoCtrl4Mode::reg, Regs::FifoCtrl4Mode::value);
            return;
        }
        const auto unread_bytes = read_result & 0x1ff / 16 / 3 * 7;
        constexpr auto single_measurement_bytes = sizeof(FifoData);
        
        std::array<FifoData, 10> read_buffer; // max 10 packages
        const auto bytes_to_read = std::min(
            static_cast<size_t>(read_buffer.size()) * sizeof(FifoData),
            static_cast<size_t>(unread_bytes)) \
                    / single_measurement_bytes * single_measurement_bytes;

        i2c.readBytes(Regs::FifoData, bytes_to_read, reinterpret_cast<uint8_t *>(read_buffer.data()));
        for (uint16_t i=0; i < bytes_to_read / sizeof(FifoData); i++) {
            switch ((read_buffer[i].tag & 0xf4) >> 3) {
                case 0x01:
                    processGyroSample(reinterpret_cast<const int16_t *>(&read_buffer[i].x), GyrTs);
                    break;
                case 0x02:
                    processAccelSample(reinterpret_cast<const int16_t *>(&read_buffer[i].x), AccTs);
                    break;
            }
        }
    }


};

} // namespace