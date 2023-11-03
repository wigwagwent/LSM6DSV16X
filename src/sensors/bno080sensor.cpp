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

#include "sensors/bno080sensor.h"
#include "utils.h"
#include "GlobalVars.h"

void BNO080Sensor::motionSetup()
{
#ifdef DEBUG_SENSOR
    imu.enableDebugging(Serial);
#endif
    if(!imu.begin(addr, Wire, m_IntPin)) {
        m_Logger.fatal("Can't connect to %s at address 0x%02x", getIMUNameByType(sensorType), addr);
        ledManager.pattern(50, 50, 200);
        return;
    }

    m_Logger.info("Connected to %s on 0x%02x.", getIMUNameByType(sensorType), addr);

    this->imu.enableLinearAccelerometer(10);

    if ((sensorType == IMU_BNO085 || sensorType == IMU_BNO086) && BNO_USE_ARVR_STABILIZATION) {
        imu.enableARVRStabilizedGameRotationVector(10); //No mag
    } else {
        imu.enableGameRotationVector(10);
    }  

    if ((sensorType == IMU_BNO085 || sensorType == IMU_BNO086) && BNO_USE_ARVR_STABILIZATION) {
        imu.enableARVRStabilizedRotationVector(10); //Mag
    } else {
        imu.enableRotationVector(10);
    }

    imu.enableGyro(1000);
    imu.enableAccelerometer(1000);
    imu.enableMagnetometer(magDelayTime);

    lastReset = 0;
    lastData = millis();
    lastCalibrationMessage = millis();
    working = true;
    configured = true;
}

void BNO080Sensor::motionLoop()
{
    //Look for reports from the IMU
    while (imu.dataAvailable())
    {
        lastReset = 0;
        lastData = millis();

        if (imu.hasNewGameQuat()) // Magless quat
        {
            imu.getGameQuat(maglessQuaternion.x, maglessQuaternion.y, maglessQuaternion.z, maglessQuaternion.w, calibrationAccuracy);
            newMaglessQuat = true;
            sensor_real_t magless[4];
            quatToVqfFormat(maglessQuaternion, magless);
            vqf.setQuat6D(magless);
        }
            
        if (imu.hasNewQuat()) // New quaternion if context
        {
            imu.getQuat(magQuaternion.x, magQuaternion.y, magQuaternion.z, magQuaternion.w, magneticAccuracyEstimate, calibrationAccuracy);
            newMagQuat = true;
        }


        if(imu.hasNewMagData()) { 
            imu.getMag(Mxyz[0], Mxyz[1], Mxyz[2], magCalibrationAccuracy);
            vqf.updateMag(Mxyz);
        }

        if (imu.getTapDetected())
        {
            tap = imu.getTapDetector();
        }
        if (m_IntPin == 255 || imu.I2CTimedOut())
            break;
    }



    if (newMagQuat && newMaglessQuat) {
        uint8_t acc;
        imu.getLinAccel(acceleration.x, acceleration.y, acceleration.z, acc);
        setAccelerationReady();

        sensor_real_t vqf9D[4];
        vqf.getQuat9D(vqf9D);
        vqfToQuatFormat(vqf9D, magEspQuaternion);


        //fusedRotation = maglessQuaternion;
        //fusedRotation = magQuaternion;
        //fusedRotation = magEspQuaternion;
        fusedRotation = magQuaternion * magEspQuaternion.inverse(); //difference between them
        fusedRotation *= sensorOffset;
        setFusedRotationReady();
    }

    if (lastCalibrationMessage + 20000 < millis())
    {
        lastCalibrationMessage = millis();
        m_Logger.info("Calibration Status: A: %d, G: %d, M: %d",
            imu.getAccelAccuracy(), imu.getGyroAccuracy(), imu.getMagAccuracy()
        );
    }


    if (lastData + 1000 < millis() && configured)
    {
        while(true) {
            BNO080Error error = imu.readError();
            if(error.error_source == 255)
                break;
            lastError = error;
            m_Logger.error("BNO08X error. Severity: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d",
                error.severity, error.error_sequence_number, error.error_source, error.error, error.error_module, error.error_code);
        }
        statusManager.setStatus(SlimeVR::Status::IMU_ERROR, true);
        working = false;
        lastData = millis();
        uint8_t rr = imu.resetReason();
        if (rr != lastReset)
        {
            lastReset = rr;
            networkConnection.sendSensorError(this->sensorId, rr);
        }

        m_Logger.error("Sensor %d doesn't respond. Last reset reason:", sensorId, lastReset);
        m_Logger.error("Last error: %d, seq: %d, src: %d, err: %d, mod: %d, code: %d",
                lastError.severity, lastError.error_sequence_number, lastError.error_source, lastError.error, lastError.error_module, lastError.error_code);
    }
}

void BNO080Sensor::quatToVqfFormat(Quat quat, sensor_real_t vqf[4]) {
    vqf[0] = quat.w;
    vqf[1] = quat.x;
    vqf[2] = quat.y;
    vqf[3] = quat.z;
}

void BNO080Sensor::vqfToQuatFormat(sensor_real_t vqf[4], Quat &quat) {
    quat.w = vqf[0];
    quat.x = vqf[1];
    quat.y = vqf[2];
    quat.z = vqf[3];
}

SensorStatus BNO080Sensor::getSensorState() {
    return lastReset > 0 ? SensorStatus::SENSOR_ERROR : isWorking() ? SensorStatus::SENSOR_OK : SensorStatus::SENSOR_OFFLINE;
}

void BNO080Sensor::startCalibration(int calibrationType)
{
    // BNO does automatic calibration,
    // it's always enabled except accelerometer
    // that is disabled 30 seconds after startup
}
