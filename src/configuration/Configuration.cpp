/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

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

#include <LittleFS.h>

#include "Configuration.h"
#include "consts.h"
#include "utils.h"

#define DIR_CALIBRATIONS "/calibrations"
#define DIR_TEMPERATURE_CALIBRATIONS "/tempcalibrations"

namespace SlimeVR {
    namespace Configuration {
        void Configuration::setup() {
            if (m_Loaded) {
                return;
            }

            bool status = LittleFS.begin();
            if (!status) {
                this->m_Logger.warn("Could not mount LittleFS, formatting");

                status = LittleFS.format();
                if (!status) {
                    this->m_Logger.warn("Could not format LittleFS, aborting");
                    return;
                }

                status = LittleFS.begin();
                if (!status) {
                    this->m_Logger.error("Could not mount LittleFS, aborting");
                    return;
                }
            }

            if (LittleFS.exists("/config.bin")) {
                m_Logger.trace("Found configuration file");

                File file = LittleFS.open("/config.bin", "r");

                file.read((uint8_t*)&m_Config.version, sizeof(uint32_t));

                if (m_Config.version < CURRENT_CONFIGURATION_VERSION) {  //Check Configuation Version Update
                    m_Logger.debug("Configuration is outdated: v%d < v%d", m_Config.version, CURRENT_CONFIGURATION_VERSION);

                    //Currently only supports updating the sensor configuration like adding a version to all sensors not migrating the config file
                    if (!runMigrationsFS(m_Config.version)) {
                        m_Logger.error("Failed to migrate configuration from v%d to v%d", m_Config.version, CURRENT_CONFIGURATION_VERSION);
                        return;
                    } else {
                        m_Logger.debug("File system configuration migrated from v%d to v%d", m_Config.version, CURRENT_CONFIGURATION_VERSION);
                    }
                } else {
                    m_Logger.info("Found up-to-date system configuration v%d", m_Config.version);
                }

                loadCalibrations(); //Check Sensor Version Update
                bool migrate = false;
                for (size_t i = 0; i < m_Calibrations.size(); i++) {
                    CalibrationConfig& c = m_Calibrations[i];
                    if (!areSensorsUpdated(c)) {
                        migrate = true;
                        if (!runMigrationsSensor(c)) {
                            m_Logger.error("Failed to migrate sensor %d", i);
                            return;
                        }
                    }
                }
                if (migrate) {
                    m_Logger.info("Calibration configuration migration done");
                } else {
                    m_Logger.info("Found up-to-date sensor configuration(s)");
                }

                file.seek(0);
                file.read((uint8_t*)&m_Config, sizeof(DeviceConfig));
                file.close();
            } else {
                m_Logger.info("No configuration file found, creating new one");
                m_Config.version = CURRENT_CONFIGURATION_VERSION;
                save();
            }

            loadCalibrations();

            m_Loaded = true;

            m_Logger.info("Loaded calibrataion configuration");

#ifdef DEBUG_CONFIGURATION
            print();
#endif
        }

        void Configuration::save(uint16_t configSize) {
            for (size_t i = 0; i < m_Calibrations.size(); i++) {
                CalibrationConfig config = m_Calibrations[i];
                if (config.type == CalibrationConfigType::NONE) {
                    continue;
                }

                char path[17];
                sprintf(path, DIR_CALIBRATIONS"/%d", i);

                m_Logger.trace("Saving calibration data for %d", i);

                File file = LittleFS.open(path, "w");
                file.write((uint8_t*)&config, configSize);
                file.close();
            }

            {
                File file = LittleFS.open("/config.bin", "w");
                file.write((uint8_t*)&m_Config, sizeof(DeviceConfig));
                file.close();
            }

            m_Logger.debug("Saved configuration");
        }

        void Configuration::reset() {
            LittleFS.format();

            m_Calibrations.clear();
            m_Config.version = CURRENT_CONFIGURATION_VERSION;
            save();

            m_Logger.debug("Reset configuration");
        }

        int32_t Configuration::getVersion() const {
            return m_Config.version;
        }

        size_t Configuration::getCalibrationCount() const {
            return m_Calibrations.size();
        }

        CalibrationConfig Configuration::getCalibration(size_t sensorID) const {
            if (sensorID >= m_Calibrations.size()) {
                return {};
            }

            return m_Calibrations.at(sensorID);
        }

        void Configuration::setCalibration(size_t sensorID, const CalibrationConfig& config) {
            size_t currentCalibrations = m_Calibrations.size();

            if (sensorID >= currentCalibrations) {
                m_Calibrations.resize(sensorID + 1);
            }

            m_Calibrations[sensorID] = config;
        }

        void Configuration::loadCalibrations(uint16_t configSize) {
#ifdef ESP32
            {
                File calibrations = LittleFS.open(DIR_CALIBRATIONS);
                if (!calibrations) {
                    m_Logger.warn("No calibration data found, creating new directory...");

                    if (!LittleFS.mkdir(DIR_CALIBRATIONS)) {
                        m_Logger.error("Failed to create directory: " DIR_CALIBRATIONS);
                        return;
                    }

                    calibrations = LittleFS.open(DIR_CALIBRATIONS);
                }

                if (!calibrations.isDirectory()) {
                    calibrations.close();

                    m_Logger.warn("Found file instead of directory: " DIR_CALIBRATIONS);

                    if (!LittleFS.remove(DIR_CALIBRATIONS)) {
                        m_Logger.error("Failed to remove directory: " DIR_CALIBRATIONS);
                        return;
                    }

                    if (!LittleFS.mkdir(DIR_CALIBRATIONS)) {
                        m_Logger.error("Failed to create directory: " DIR_CALIBRATIONS);
                        return;
                    }

                    calibrations = LittleFS.open(DIR_CALIBRATIONS);
                }

                m_Logger.trace("Found calibration data directory");

                while (File f = calibrations.openNextFile()) {
                    if (f.isDirectory()) {
                        continue;
                    }

                    m_Logger.trace("Found calibration data file: %s", f.name());

                    uint8_t sensorId = strtoul(f.name(), nullptr, 10);
                    CalibrationConfig calibrationConfig = {};
                    f.read((uint8_t*)&calibrationConfig, configSize);
                    f.close();

                    m_Logger.trace("Found sensor calibration for %s at index %d", calibrationConfigTypeToString(calibrationConfig.type), sensorId);

                    setCalibration(sensorId, calibrationConfig);
                }

                calibrations.close();
            }
#else
            {
                if (!LittleFS.exists(DIR_CALIBRATIONS)) {
                    m_Logger.warn("No calibration data found, creating new directory...");

                    if (!LittleFS.mkdir(DIR_CALIBRATIONS)) {
                        m_Logger.error("Failed to create directory: " DIR_CALIBRATIONS);
                        return;
                    }

                    // There's no calibrations here, so we're done
                    return;
                }

                Dir calibrations = LittleFS.openDir(DIR_CALIBRATIONS);
                while (calibrations.next()) {
                    File f = calibrations.openFile("r");
                    if (!f.isFile()) {
                        continue;
                    }

                    CalibrationConfig calibrationConfig = {};
                    f.read((uint8_t*)&calibrationConfig, configSize);

                    uint8_t sensorId = strtoul(calibrations.fileName().c_str(), nullptr, 10);
                    m_Logger.trace("Found sensor calibration for %s at index %d", calibrationConfigTypeToString(calibrationConfig.type), sensorId);

                    setCalibration(sensorId, calibrationConfig);
                }
            }
#endif
        }

        bool Configuration::loadTemperatureCalibration(uint8_t sensorId, GyroTemperatureCalibrationConfig& config) {
#ifdef ESP32
            {
                File calibrations = LittleFS.open(DIR_TEMPERATURE_CALIBRATIONS);
                if (!calibrations) {
                    m_Logger.warn("No temperature calibration data found, creating new directory...");

                    if (!LittleFS.mkdir(DIR_TEMPERATURE_CALIBRATIONS)) {
                        m_Logger.error("Failed to create directory: " DIR_TEMPERATURE_CALIBRATIONS);
                        return false;
                    }

                    calibrations = LittleFS.open(DIR_TEMPERATURE_CALIBRATIONS);
                }

                if (!calibrations.isDirectory()) {
                    calibrations.close();

                    m_Logger.warn("Found file instead of directory: " DIR_TEMPERATURE_CALIBRATIONS);

                    if (!LittleFS.remove(DIR_TEMPERATURE_CALIBRATIONS)) {
                        m_Logger.error("Failed to remove directory: " DIR_TEMPERATURE_CALIBRATIONS);
                        return false;
                    }

                    if (!LittleFS.mkdir(DIR_TEMPERATURE_CALIBRATIONS)) {
                        m_Logger.error("Failed to create directory: " DIR_TEMPERATURE_CALIBRATIONS);
                        return false;
                    }

                    return false;
                }
                
                calibrations.close();

                m_Logger.debug("Found temperature calibration data directory");

                char path[32];
                sprintf(path, DIR_TEMPERATURE_CALIBRATIONS"/%d", sensorId);
                if (LittleFS.exists(path)) {
                    File f = LittleFS.open(path, "r");
                    if (f.isDirectory()) {
                        return false;
                    }

                    if (f.size() == sizeof(GyroTemperatureCalibrationConfig)) {
                        CalibrationConfigType storedConfigType;
                        f.read((uint8_t*)&storedConfigType, sizeof(CalibrationConfigType));
                        if (storedConfigType != config.type) {
                            f.close();
                            m_Logger.debug(
                                "Found incompatible sensor temperature calibration (expected %s, found %s) sensorId:%d, skipping",
                                calibrationConfigTypeToString(config.type),
                                calibrationConfigTypeToString(storedConfigType),
                                sensorId
                            );
                            return false;
                        }

                        f.seek(0);
                        f.read((uint8_t*)&config, sizeof(GyroTemperatureCalibrationConfig));
                        f.close();
                        m_Logger.debug("Found sensor temperature calibration for %s sensorId:%d", calibrationConfigTypeToString(config.type), sensorId);
                        return true;
                    } else {
                        m_Logger.debug("Found incompatible sensor temperature calibration (size mismatch) sensorId:%d, skipping", sensorId);
                    }
                }
                return false;
            }
#else
            {
                if (!LittleFS.exists(DIR_TEMPERATURE_CALIBRATIONS)) {
                    m_Logger.warn("No temperature calibration data found, creating new directory...");

                    if (!LittleFS.mkdir(DIR_TEMPERATURE_CALIBRATIONS)) {
                        m_Logger.error("Failed to create directory: " DIR_TEMPERATURE_CALIBRATIONS);
                        return false;
                    }

                    // There's no calibrations here, so we're done
                    return false;
                }

                char path[32];
                sprintf(path, DIR_TEMPERATURE_CALIBRATIONS"/%d", sensorId);
                if (LittleFS.exists(path)) {
                    File f = LittleFS.open(path, "r");
                    if (!f.isFile()) {
                        return false;
                    }

                    if (f.size() == sizeof(GyroTemperatureCalibrationConfig)) {
                        CalibrationConfigType storedConfigType;
                        f.read((uint8_t*)&storedConfigType, sizeof(CalibrationConfigType));
                        if (storedConfigType != config.type) {
                            f.close();
                            m_Logger.debug(
                                "Found incompatible sensor temperature calibration (expected %s, found %s) sensorId:%d, skipping",
                                calibrationConfigTypeToString(config.type),
                                calibrationConfigTypeToString(storedConfigType),
                                sensorId
                            );
                            return false;
                        }

                        f.seek(0);
                        f.read((uint8_t*)&config, sizeof(GyroTemperatureCalibrationConfig));
                        f.close();
                        m_Logger.debug("Found sensor temperature calibration for %s sensorId:%d", calibrationConfigTypeToString(config.type), sensorId);
                        return true;
                    } else {
                        m_Logger.debug("Found incompatible sensor temperature calibration (size mismatch) sensorId:%d, skipping", sensorId);
                    }
                }
                
                return false;
            }
#endif
        }

        bool Configuration::saveTemperatureCalibration(uint8_t sensorId, const GyroTemperatureCalibrationConfig& config) {
            if (config.type == CalibrationConfigType::NONE) {
                return false;
            }

            char path[32];
            sprintf(path, DIR_TEMPERATURE_CALIBRATIONS"/%d", sensorId);

            m_Logger.trace("Saving temperature calibration data for sensorId:%d", sensorId);

            File file = LittleFS.open(path, "w");
            file.write((uint8_t*)&config, sizeof(GyroTemperatureCalibrationConfig));
            file.close();

            m_Logger.debug("Saved temperature calibration data for sensorId:%i", sensorId);
            return true;
        }

        void* memcpy_index(void *to, const void *from, size_t index, size_t size) {
            from = ((char*)from)+index;
            return memcpy(to, from, size);
        }

        bool Configuration::runMigrationsFS(uint32_t version) {
            static_assert(sizeof(CalibrationConfig) == 200,"A migration script needs to be made with NoneCalibrationConfig::file_size updated");
            //The struct size was increased from 120 to 200
            if (version == 1) {
                loadCalibrations(120);
                save(200);
                version++;
            }

            // Each sensor now has a calibration version
            if (version == 2) {
                loadCalibrations(200);
                struct MigrationCalibrationConfigV2 {
                    CalibrationConfigType type;
                    uint32_t version;
                    uint8_t calibration_data[190];
                };

                for (size_t i = 0; i < getCalibrationCount(); i++)
                {
                    MigrationCalibrationConfigV2 sensor = {};

                    //memcpy(&sensor, &m_Calibrations[i], sizeof(CalibrationConfigType));
                    memcpy(&sensor.type, &m_Calibrations[i].type, 4);

                    sensor.version = 1;

                    memcpy_index(&sensor.calibration_data, &m_Calibrations[i], 4, 190);

					memcpy(&m_Calibrations[i], &sensor, 200);       
				}
                save(200);
                version++;
            }

            m_Config.version = version;
            save();
            if (version == CURRENT_CONFIGURATION_VERSION) {
                return true;
            }
            return false;
        }

        bool Configuration::areSensorsUpdated(CalibrationConfig& c) {
            switch (c.type) {
                case CalibrationConfigType::NONE:
                    return true;

                case CalibrationConfigType::BMI160: {
                    if (c.version == BMI160CalibrationLatestVersion) {
                        return true;
                    }
                } break;

                case CalibrationConfigType::ICM20948: {
                    if (c.version == ICM20948CalibrationLatestVersion) {
                        return true;
                    }
                } break;

                case CalibrationConfigType::MPU9250: {
                    if (c.version == MPU9250CalibrationLatestVersion) {
                        return true;
                    }
                } break;

                case CalibrationConfigType::MPU6050: {
                    if (c.version == MPU6050CalibrationLatestVersion) {
                        return true;
                    }
                } break;
            }
            return false;
        }

        bool Configuration::runMigrationsSensor(CalibrationConfig& c) {
            uint32_t version = c.version;
            switch (c.type) {
                case CalibrationConfigType::NONE:
                    return true;

// #if ((IMU == IMU_BMI160) || (SECOND_IMU == IMU_BMI160))
                case CalibrationConfigType::BMI160: {

                    // Added gyrocope sensitivity calibration (G_sens)
                    if (c.version == 1) {
                        struct MigrationBMI160CalibrationV1 {
                            uint8_t data[108];
                            float G_Sens[3];
                            float temp;
                        };
                        MigrationBMI160CalibrationV1 cal = {};
                        memcpy(&cal, &c.data, 112);
                        cal.temp = cal.G_Sens[0];
                        cal.G_Sens[0] = 1.0;
                        cal.G_Sens[1] = 1.0;
                        cal.G_Sens[2] = 1.0;
                        memcpy(&c.data, &cal, 124);
                        c.version++;
                    }

                    if (c.version != version) {
                        save();
                    }
                    return areSensorsUpdated(c);
                } break;
// #endif

                case CalibrationConfigType::ICM20948: {

                    if (c.version != version) {
                        save();
                    }
                    return areSensorsUpdated(c);
                } break;

                case CalibrationConfigType::MPU9250: {
                    
                    if (c.version != version) {
                        save();
                    }
                    return areSensorsUpdated(c);
                } break;

                case CalibrationConfigType::MPU6050: {
                    
                    if (c.version != version) {
                        save();
                    }
                    return areSensorsUpdated(c);
                } break;
            }
            return false;
        }

        void Configuration::print() {
            m_Logger.info("Configuration:");
            m_Logger.info("  Version: %d", m_Config.version);
            m_Logger.info("  %d Calibrations:", m_Calibrations.size());

            for (size_t i = 0; i < m_Calibrations.size(); i++) {
                const CalibrationConfig& c = m_Calibrations[i];
                m_Logger.info("    - [%3d] %s", i, calibrationConfigTypeToString(c.type));
                m_Logger.info("            Version    : %d", c.version);

                switch (c.type) {
                case CalibrationConfigType::NONE:
                    break;

                case CalibrationConfigType::BMI160:
                    m_Logger.info("            A_B        : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.bmi160.A_B));

                    m_Logger.info("            A_Ainv     :");
                    for (uint8_t i = 0; i < 3; i++) {
                        m_Logger.info("                         %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.bmi160.A_Ainv[i]));
                    }

                    m_Logger.info("            G_off      : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.bmi160.G_off));
                    m_Logger.info("            G_sens     : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.bmi160.G_Sens));
                    m_Logger.info("            Temperature: %f", c.data.bmi160.temperature);

                    break;

                case CalibrationConfigType::ICM20948:
                    m_Logger.info("            G: %d, %d, %d", UNPACK_VECTOR_ARRAY(c.data.icm20948.G));
                    m_Logger.info("            A: %d, %d, %d", UNPACK_VECTOR_ARRAY(c.data.icm20948.A));
                    m_Logger.info("            C: %d, %d, %d", UNPACK_VECTOR_ARRAY(c.data.icm20948.C));

                    break;

                case CalibrationConfigType::MPU9250:
                    m_Logger.info("            A_B   : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.A_B));

                    m_Logger.info("            A_Ainv:");
                    for (uint8_t i = 0; i < 3; i++) {
                        m_Logger.info("                    %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.A_Ainv[i]));
                    }

                    m_Logger.info("            M_B   : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.M_B));

                    m_Logger.info("            M_Ainv:");
                    for (uint8_t i = 0; i < 3; i++) {
                        m_Logger.info("                    %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.M_Ainv[i]));
                    }

                    m_Logger.info("            G_off  : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu9250.G_off));

                    break;

                case CalibrationConfigType::MPU6050:
                    m_Logger.info("            A_B  : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu6050.A_B));
                    m_Logger.info("            G_off: %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.mpu6050.G_off));

                    break;

                case CalibrationConfigType::ICM42688:
                    m_Logger.info("            A_B   : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.icm42688.A_B));

                    m_Logger.info("            A_Ainv:");
                    for (uint8_t i = 0; i < 3; i++) {
                        m_Logger.info("                    %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.icm42688.A_Ainv[i]));
                    }

                    m_Logger.info("            M_B   : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.icm42688.M_B));

                    m_Logger.info("            M_Ainv:");
                    for (uint8_t i = 0; i < 3; i++) {
                        m_Logger.info("                    %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.icm42688.M_Ainv[i]));
                    }

                    m_Logger.info("            G_off  : %f, %f, %f", UNPACK_VECTOR_ARRAY(c.data.icm42688.G_off));

                    break;
                }
            }
        }
    }
}
