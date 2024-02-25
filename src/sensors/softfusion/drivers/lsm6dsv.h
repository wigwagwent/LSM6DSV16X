#pragma once

#include <cstdint>
#include <array>
#include <algorithm>

namespace SlimeVR::Sensors::SoftFusion::Drivers
{

// Driver uses acceleration range at 8g
// and gyroscope range at 1000dps
// Gyroscope ODR = 480Hz, accel ODR = 120Hz

template <typename I2CImpl>
struct LSM6DSV
{
    uint32_t currentTimestamp = 0;
    uint32_t previousGyroTimestamp = 0;
    uint32_t previousAccelTimestamp = 0;

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
        struct HAODRCtrl {
            static constexpr uint8_t reg = 0x62;
            static constexpr uint8_t value = (0b00); //1st ODR table
        };
        struct FunctionsCtrl {
            static constexpr uint8_t reg = 0x50;
            static constexpr uint8_t value = (0b01000000); //Enable timestamping
        };
        struct Ctrl1XLODR {
            static constexpr uint8_t reg = 0x10;
            static constexpr uint8_t value = (0b0010110); //120Hz, HAODR
        };
        struct Ctrl2GODR {
            static constexpr uint8_t reg = 0x11;
            static constexpr uint8_t value = (0b0011000); //480Hz, HAODR
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
            static constexpr uint8_t value = (0b0110) | (0b1000 << 4); //gyro and accel batched at 480Hz
        };
        struct FifoCtrl4Mode {
            static constexpr uint8_t reg = 0x0a;
            static constexpr uint8_t value = (0b01000110); //continuous mode, timestamp in FIFO
        };

        static constexpr uint8_t FifoStatus = 0x1b;
        static constexpr uint8_t FifoData = 0x78;
    };

    bool initialize()
    {
        // perform initialization step
        i2c.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
        delay(20);
        i2c.writeReg(Regs::HAODRCtrl::reg, Regs::HAODRCtrl::value);
        i2c.writeReg(Regs::FunctionsCtrl::reg, Regs::FunctionsCtrl::value);
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

    static constexpr size_t FullFifoEntrySize = 7;

    template <typename AccelCall, typename GyroCall>
    void bulkRead(AccelCall &&processAccelSample, GyroCall &&processGyroSample) {
        const auto fifo_status = i2c.readReg16(Regs::FifoStatus);
        const auto available_data = fifo_status & 0x1ff;
        const auto fifo_bytes = available_data * 7;

        static int print_counter = 0;
        if (print_counter == 16) {
            printf("%d\n", available_data);
            print_counter = 0;
        }
        print_counter++;
        
        std::array<uint8_t, FullFifoEntrySize * 8> read_buffer; // max 8 readings
        const auto bytes_to_read = std::min(static_cast<size_t>(read_buffer.size()),
            static_cast<size_t>(fifo_bytes)) / FullFifoEntrySize * FullFifoEntrySize;
        i2c.readBytes(Regs::FifoData, bytes_to_read, read_buffer.data());
        for (auto i=0u; i<bytes_to_read; i+=FullFifoEntrySize) {
            uint8_t tag = read_buffer[i] >> 3;

            uint8_t *raw = &read_buffer[i + 1];
            int16_t xyz[3];

            switch (tag) {
                case 0x01: // Gyro NC
                    memcpy(xyz, raw, sizeof(xyz));
                    processGyroSample(xyz, float((currentTimestamp - previousGyroTimestamp) * 21.75 / 1e6));
                    previousGyroTimestamp = currentTimestamp;
                    break;
                case 0x02: // Accel NC
                    memcpy(xyz, raw, sizeof(xyz));
                    processAccelSample(xyz, float((currentTimestamp - previousAccelTimestamp) * 21.75 / 1e6));
                    previousAccelTimestamp = currentTimestamp;
                    break;
                case 0x04: // Timestamp
                    // Multiply by 21.75 to convert to microseconds, divide by 1e6 to convert to seconds
                    // currentTimestamp = raw[0] | (raw[1] << 8) | (raw[2] << 16) | (raw[3] << 24);
                    break;
            }
        }      
    }


};

} // namespace