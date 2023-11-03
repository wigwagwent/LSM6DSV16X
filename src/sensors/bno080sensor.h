/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#ifndef SENSORS_BNO080SENSOR_H
#define SENSORS_BNO080SENSOR_H

#include "sensor.h"
#include <BNO080.h>
#include "../motionprocessing/types.h"
#include <vqf.h>

constexpr uint16_t magDelayTime = 50; //milliseconds

class BNO080Sensor : public Sensor
{
    struct SensorVQFParams: VQFParams {
            SensorVQFParams() : VQFParams() {
                #ifndef VQF_NO_MOTION_BIAS_ESTIMATION
                motionBiasEstEnabled = false;
                #endif
                tauAcc = 2.0f;
                restMinT = 2.0f;
                restThGyr = 0.6f; // 400 norm
                restThAcc = 0.06f; // 100 norm
            }
        };

public:
    BNO080Sensor(uint8_t id, uint8_t type, uint8_t address, float rotation, uint8_t sclPin, uint8_t sdaPin, uint8_t intPin)
        : Sensor("BNO080Sensor", type, id, address, rotation, sclPin, sdaPin), m_IntPin(intPin), vqf(vqfParams, (float)magDelayTime * 1e-3) {};
    ~BNO080Sensor(){};
    void motionSetup() override final;
    void postSetup() override {
        lastData = millis();
    }

    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;
    SensorStatus getSensorState() override final;

private:
    BNO080 imu{};

    uint8_t m_IntPin;

    uint8_t tap;
    unsigned long lastData = 0;
    unsigned long lastCalibrationMessage = 0;
    uint8_t lastReset = 0;
    BNO080Error lastError{};
    Quat maglessQuaternion{};
    bool newMaglessQuat = false;

    // Magnetometer specific members
    float Mxyz[3] = {};
    Quat magEspQuaternion{};
    bool newMagEspQuat = false;

    Quat magQuaternion{};
    bool newMagQuat = false;
    uint8_t magCalibrationAccuracy = 0;
    float magneticAccuracyEstimate = 999;
    bool newMagData = false;

    SensorVQFParams vqfParams {};
    VQF vqf;

    void quatToVqfFormat(Quat quat, sensor_real_t vqf[4]);
    void vqfToQuatFormat(sensor_real_t vqf[4], Quat &quat);
};

#endif
