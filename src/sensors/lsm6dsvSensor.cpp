/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 Eiren Rain & SlimeVR contributors

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

#include "sensors/lsm6dsvSensor.h"

#include "GlobalVars.h"
#include "lsm6dsvSensor.h"
#include "utils.h"

#ifdef LSM6DSV_INTERRUPT
volatile bool imuEvent = false;
void IRAM_ATTR interruptHandler() { imuEvent = true; }
#endif  // LSM6DSV_INTERRUPT

LSM6DSVSensor::LSM6DSVSensor(
	uint8_t id,
	uint8_t type,
	uint8_t address,
	float rotation,
	uint8_t sclPin,
	uint8_t sdaPin,
	uint8_t intPin
) : Sensor("LSM6DSVSensor", type, id, address, rotation, sclPin, sdaPin), 
	  imu(&Wire, addr << 1),  // We shift the address left 1 to work with the library
	  m_IntPin(intPin)
#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
	  , sfusion(
		  1.0f / LSM6DSV_FIFO_DATA_RATE,
		  1.0f / LSM6DSV_FIFO_DATA_RATE,
		  -1.0f )
#endif //LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP
	  {};

void LSM6DSVSensor::motionSetup() {
	uint8_t deviceId = 0;
	if (imu.ReadID(&deviceId) == LSM6DSV_ERROR) {
		m_Logger.fatal(
			"The %s at 0x%02x returned an error when reading the device ID of: 0x%02x",
			getIMUNameByType(sensorType),
			addr,
			deviceId
		);
		ledManager.pattern(50, 50, 200);
		status = LSM6DSV_ERROR;
		return;
	}

	if (deviceId != LSM6DSV_ID) {
		m_Logger.fatal(
			"The %s at 0x%02x returned an invalid device ID of: 0x%02x when it should "
			"have been: 0x%02x",
			getIMUNameByType(sensorType),
			addr,
			deviceId,
			LSM6DSV_ID
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	m_Logger.info("Connected to %s on 0x%02x", getIMUNameByType(sensorType), addr);

	status |= imu.Device_Reset(LSM6DSV_RESET_GLOBAL);

	status |= imu.begin();
	
	status |= imu.Set_G_FS(LSM6DSV_GYRO_MAX);
	status |= imu.Set_X_FS(LSM6DSV_ACCEL_MAX);

#ifdef LSM6DSV_USE_HIGH_PERFORMANCE_MODE
	status |= imu.Set_G_ODR(LSM6DSV_GYRO_RATE, LSM6DSV_GYRO_HIGH_PERFORMANCE_MODE);
	status |= imu.Set_X_ODR(LSM6DSV_ACCEL_RATE, LSM6DSV_ACC_HIGH_PERFORMANCE_MODE);
#else
	status |= imu.Set_High_Accuracy_ODR(LSM6DSV_GYRO_RATE, LSM6DSV_ACCEL_RATE);
#endif

	status |= imu.FIFO_Set_X_BDR(LSM6DSV_FIFO_DATA_RATE);

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
	status |= imu.FIFO_Set_G_BDR(LSM6DSV_FIFO_DATA_RATE);
#endif  // LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP

	status |= imu.FIFO_Set_Timestamp_Decimation(
		lsm6dsv_fifo_timestamp_batch_t::LSM6DSV_TMSTMP_DEC_1
	);

	// Enable IMU
	status |= imu.FIFO_Enable_Timestamp();
	status |= imu.Enable_X();
	status |= imu.Enable_G();

	status |= imu.Set_X_Filter_Mode(0, LSM6DSV_GYRO_LPF);
	status |= imu.Set_G_Filter_Mode(0, LSM6DSV_ACCEL_LPF);

	// Set FIFO mode to "continuous", so old data gets thrown away
	status |= imu.FIFO_Set_Mode(LSM6DSV_STREAM_MODE);

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ONBOARD)
	status |= imu.Set_SFLP_ODR(LSM6DSV_FIFO_DATA_RATE);
	status |= imu.Set_SFLP_Batch(true, true, false);
#endif  // LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ONBOARD

#ifdef LSM6DSV_USE_ONBOARD_TAP_DETECTION
#ifdef LSM6DSV_INTERRUPT
	attachInterrupt(m_IntPin, interruptHandler, RISING);
	status |= imu.Enable_Single_Tap_Detection(LSM6DSV_INT1_PIN);  // Tap recommends an interrupt
#else
	status |= imu.Enable_Single_Tap_Detection(LSM6DSV_INT2_PIN);  // Just poll to see if an event happened jank but works
#endif  // LSM6DSV_INTERRUPT

	status |= imu.Set_Tap_Threshold(LSM6DSV_TAP_THRESHOLD);
	status |= imu.Set_Tap_Shock_Time(LSM6DSV_TAP_SHOCK_TIME);
	status |= imu.Set_Tap_Quiet_Time(LSM6DSV_TAP_QUITE_TIME);
#endif

	// ! set these again
	// status |= imu.Set_X_FS(LSM6DSV_ACCEL_MAX);
	// status |= imu.Set_X_ODR(LSM6DSV_ACCEL_RATE, LSM6DSV_ACC_HIGH_PERFORMANCE_MODE);

	status |= imu.FIFO_Reset();

	if (status != LSM6DSV_OK) {
		m_Logger.fatal(
			"Errors occured enabling imu features on %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
	checkForCalibrationRequirements();
	if (status == LSM6DSV_ERROR) {
		return;
	}
#endif  // LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP

	lastData = millis();
	working = true;
	configured = true;
	lastTempRead = millis();

	isAccelCalibrated = hasAccelCalibration();
	isGyroCalibrated = hasGyroCalibration();
}

constexpr float mgPerG = 1000.0f;
constexpr float mdpsPerDps = 1000.0f;
constexpr float dpsPerRad = 57.295779578552f;
constexpr uint8_t fifoFrameSize = 4;  // X BDR, (G BDR || Rotation), Gravity, Timestamp

void LSM6DSVSensor::motionLoop() {
#ifdef LSM6DSV_USE_ONBOARD_TAP_DETECTION
#ifdef LSM6DSV_INTERRUPT
	if (imuEvent) {
		LSM6DSV_Event_Status_t eventStatus;
		status |= imu.Get_X_Event_Status(&eventStatus);

		if (eventStatus.TapStatus) {
			tap++;
		}
		imuEvent = false;
	}
#else
	LSM6DSV_Event_Status_t eventStatus;
	status |= imu.Get_X_Event_Status(&eventStatus);

	if (eventStatus.TapStatus) {
		tap++;
	}
#endif  // LSM6DSV_INTERRUPT
#endif

	if (millis() - lastTempRead > LSM6DSV_TEMP_READ_INTERVAL * 1000) {
		lastTempRead = millis();

		int16_t rawTemp;
		if (imu.Get_Temp_Raw(&rawTemp) != LSM6DSV_OK) {
			m_Logger.error(
				"Error getting temperature on %s at address 0x%02x",
				getIMUNameByType(sensorType),
				addr
			);
			status = LSM6DSV_ERROR;
		} else {
			float actualTemp = lsm6dsv_from_lsb_to_celsius(rawTemp);
			if (fabsf(actualTemp - temperature) > 0.01f) {
				temperature = actualTemp;
				newTemperature = true;
			}
		}
	}

	if (lastData + 1000 < millis() && configured) {  // Errors
		m_Logger.error(
			"The %s at address 0x%02x, has not responded in the last second",
			getIMUNameByType(sensorType),
			addr
		);
		statusManager.setStatus(SlimeVR::Status::IMU_ERROR, true);
		working = false;
		lastData = millis();  // reset time counter for error message
	}

	uint16_t fifo_samples = 0;
	if (imu.FIFO_Get_Num_Samples(&fifo_samples) == LSM6DSV_ERROR) {
		m_Logger.error(
			"Error getting number of samples in FIFO on %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		status = LSM6DSV_ERROR;
		return;
	}

	readFifo(fifo_samples);

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
	fusedRotation = sfusion.getQuaternionQuat() * sensorOffset;
	if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES
		|| !lastFusedRotationSent.equalsWithEpsilon(fusedRotation)) {
		newFusedRotation = true;
		lastFusedRotationSent = fusedRotation;
	}
	lastData = millis();

	sensor_real_t linAcceleration[3];
	sfusion.getLinearAcc(linAcceleration);
	acceleration.x = linAcceleration[0];
	acceleration.y = linAcceleration[1];
	acceleration.z = linAcceleration[2];
	newAcceleration = true;

#endif  // LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP

#ifdef DEBUG_SENSOR
	Vector3 rotation = lastFusedRotationSent.get_euler() * dpsPerRad;
	m_Logger.trace(
		",%f,%f,%f,%d,%f,%d,%f,%f,%f,%f,%f,%f",
		rotation.x,
		rotation.y,
		rotation.z,
		currentDataTime,
		lsm6dsv_from_lsb_to_nsec(currentDataTime - previousDataTime) * 1e-9,
		millis(),
		rawGyro[0],
		rawGyro[1],
		rawGyro[2],
		rawAcceleration[0],
		rawAcceleration[1],
		rawAcceleration[2]
	);
#endif  // DEBUG_SENSOR
}

SensorStatus LSM6DSVSensor::getSensorState() {
	return isWorking()
			 ? (status == LSM6DSV_OK ? SensorStatus::SENSOR_OK
									 : SensorStatus::SENSOR_ERROR)
			 : SensorStatus::SENSOR_OFFLINE;
}

LSM6DSVStatusTypeDef LSM6DSVSensor::runSelfTest() {
	m_Logger.info(
		"%s Self Test started on address: 0x%02x",
		getIMUNameByType(sensorType),
		addr
	);

	if (imu.Test_IMU(LSM6DSV_XL_ST_NEGATIVE, LSM6DSV_GY_ST_NEGATIVE) == LSM6DSV_ERROR) {
		return LSM6DSV_ERROR;
	}
	m_Logger.info(
		"%s Self Test Passed on address: 0x%02x",
		getIMUNameByType(sensorType),
		addr
	);

	return LSM6DSV_OK;
}

void LSM6DSVSensor::sendData() {
	if (newFusedRotation) {
		newFusedRotation = false;
		networkConnection.sendRotationData(
			sensorId,
			&fusedRotation,
			DATA_TYPE_NORMAL,
			calibrationAccuracy
		);

#ifdef DEBUG_SENSOR
		m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(fusedRotation));
#endif  // DEBUG_SENSOR
	}

#if SEND_ACCELERATION
	if (newAcceleration)  // Returns acceleration in G's
	{
		newAcceleration = false;
		networkConnection.sendSensorAcceleration(sensorId, acceleration);
	}
#endif  // SEND_ACCELERATION

#ifdef LSM6DSV_USE_ONBOARD_TAP_DETECTION
	if (tap != 0) {
		networkConnection.sendSensorTap(sensorId, tap);
		tap = 0;
	}
#endif

	if (newTemperature) {
		newTemperature = false;
		networkConnection.sendTemperature(sensorId, temperature);
	}
}

LSM6DSVStatusTypeDef LSM6DSVSensor::readFifo(uint16_t fifo_samples) {
	for (uint16_t i = 0; i < fifo_samples; i++) {
		uint8_t tag;
		if (imu.FIFO_Get_Tag(&tag) != LSM6DSV_OK) {
			m_Logger.error(
				"Failed to get FIFO data tag on %s at address 0x%02x",
				getIMUNameByType(sensorType),
				addr
			);
			return LSM6DSV_ERROR;
		}

		switch (tag) {
			case lsm6dsv_fifo_out_raw_t::LSM6DSV_TIMESTAMP_TAG: {
				if (i % fifoFrameSize != 0) {
					return LSM6DSV_OK;  // If we are not requesting a full data set
										// then stop reading
				}
				previousDataTime = currentDataTime;
				if (imu.FIFO_Get_Timestamp(&currentDataTime)) {
					m_Logger.error(
						"Failed to get timestamp data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV_ERROR;
				}

				// newTemperature = false;

				break;
			}

			case lsm6dsv_fifo_out_raw_t::LSM6DSV_XL_NC_TAG: {  // accel
				int32_t intAcceleration[3];
				if (imu.FIFO_Get_X_Axes(intAcceleration) != LSM6DSV_OK) {
					m_Logger.error(
						"Failed to get accelerometer data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV_ERROR;
				}
#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
				handleAccelSample(intAcceleration);
#endif  // LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP

				break;
			}

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
			case lsm6dsv_fifo_out_raw_t::LSM6DSV_GY_NC_TAG: {
				int32_t angularVelocity[3];
				if (imu.FIFO_Get_G_Axes(angularVelocity) != LSM6DSV_OK) {
					m_Logger.error(
						"Failed to get accelerometer data on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV_ERROR;
				}
				handleGyroSample(angularVelocity);
				break;
			}
#endif  // LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ONBOARD)
			case lsm6dsv_fifo_out_raw_t::LSM6DSV_SFLP_GAME_ROTATION_VECTOR_TAG: {
				float quat[4];
				if (imu.FIFO_Get_Rotation_Vector(quat) != LSM6DSV_OK) {
					m_Logger.error(
						"Failed to get game rotation vector on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV_ERROR;
				}

				fusedRotation = Quat(quat[0], quat[1], quat[2], quat[3]) * sensorOffset;
				if (ENABLE_INSPECTION || !OPTIMIZE_UPDATES
					|| !lastFusedRotationSent.equalsWithEpsilon(fusedRotation)) {
					newFusedRotation = true;
					lastFusedRotationSent = fusedRotation;
				}
				lastData = millis();
				break;
			}

			case lsm6dsv_fifo_out_raw_t::LSM6DSV_SFLP_GRAVITY_VECTOR_TAG: {
				float gravityVector[3];
				if (imu.FIFO_Get_Gravity_Vector(gravityVector) != LSM6DSV_OK) {
					m_Logger.error(
						"Failed to get gravity vector on %s at address 0x%02x",
						getIMUNameByType(sensorType),
						addr
					);
					return LSM6DSV_ERROR;
				}
				gravityVector[0] /= mgPerG;
				gravityVector[1] /= mgPerG;
				gravityVector[2] /= mgPerG;

				sensor_real_t linAcceleration[3];
				SlimeVR::Sensors::SensorFusion::calcLinearAcc(
					rawAcceleration,
					gravityVector,
					linAcceleration
				);
				acceleration.x = linAcceleration[0];
				acceleration.y = linAcceleration[1];
				acceleration.z = linAcceleration[2];
				newAcceleration = true;
				break;
			}

#endif  // LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ONBOARD
		}
	}
	return LSM6DSV_OK;
}

#if (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
// Used for calibration (Blocking)
LSM6DSVStatusTypeDef LSM6DSVSensor::readNextFifoFrame() {
	uint16_t fifo_samples = 0;
	while (fifo_samples < fifoFrameSize) {
		if (imu.FIFO_Get_Num_Samples(&fifo_samples) == LSM6DSV_ERROR) {
			m_Logger.error(
				"Error getting number of samples in FIFO on %s at address 0x%02x",
				getIMUNameByType(sensorType),
				addr
			);
			return LSM6DSV_ERROR;
		}
	}

	return readFifo(fifoFrameSize);
}

void LSM6DSVSensor::loadIMUCalibration() {
	SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
	// If no compatible calibration data is found, the calibration data will just be
	// zero-ed out
	switch (sensorCalibration.type) {
		case SlimeVR::Configuration::CalibrationConfigType::LSM6DSV:
			m_Calibration = sensorCalibration.data.lsm6dsv;
			break;

		case SlimeVR::Configuration::CalibrationConfigType::NONE:
			m_Logger.warn(
				"No calibration data found for sensor %d, ignoring...",
				sensorId
			);
			m_Logger.info("Calibration is advised");
			break;

		default:
			m_Logger.warn(
				"Incompatible calibration data found for sensor %d, ignoring...",
				sensorId
			);
			m_Logger.info("Calibration is advised");
	}
}

#ifdef LSM6DSV_GYRO_OFFSET_CAL
void LSM6DSVSensor::calibrateGyro() {
    #if LSM6DSV_CALIBRATION_GYRO_SECONDS == 0
        m_Logger.debug("Skipping gyro calibration");
        return;
    #endif

    // Wait for sensor to calm down before calibration
    constexpr uint8_t GYRO_CALIBRATION_DELAY_SEC = 3;
    constexpr float GYRO_CALIBRATION_DURATION_SEC = BMI160_CALIBRATION_GYRO_SECONDS;
    m_Logger.info("Put down the device and wait for baseline gyro reading calibration (%.1f seconds)", GYRO_CALIBRATION_DURATION_SEC);
    ledManager.on();
    for (uint8_t i = GYRO_CALIBRATION_DELAY_SEC; i > 0; i--) {
        m_Logger.info("%i...", i);
        delay(1000);
    }
    ledManager.off();

	int16_t rawTemp;
    if (imu.Get_Temp_Raw(&rawTemp) != LSM6DSV_OK) {
        m_Logger.error("Error: can't read temperature");
    }
    m_Calibration.temperature = lsm6dsv_from_lsb_to_celsius(rawTemp);

    #ifdef DEBUG_SENSOR
        m_Logger.trace("Calibration temperature: %f", temperature);
    #endif

	uint8_t drdyStatus;
	if (imu.Get_G_DRDY_Status(&drdyStatus) != LSM6DSV_OK) {
        m_Logger.error("Fatal error: couldn't get gyroscope drdy status!");
		return;
	}
    if (drdyStatus == 0) {
        m_Logger.error("Fatal error: gyroscope drdy = 0 (dead?)");
        return;
    }

    ledManager.pattern(100, 100, 3);
    ledManager.on();
    m_Logger.info("Gyro calibration started...");

    constexpr uint16_t gyroCalibrationSamples =
        GYRO_CALIBRATION_DURATION_SEC / (1.0f / LSM6DSV_GYRO_RATE);
    float rawGxyz[3] = {0};
    for (int i = 0; i < gyroCalibrationSamples; i++) {
		int16_t rawGyro[3];
		if (imu.Get_G_AxesRaw_When_Aval(rawGyro) != LSM6DSV_OK) {
			m_Logger.error("Fatal error: couldn't get gyro sample!");
			return;
		}

        rawGxyz[0] += rawGyro[0] / mdpsPerDps / dpsPerRad;
        rawGxyz[1] += rawGyro[1] / mdpsPerDps / dpsPerRad;
        rawGxyz[2] += rawGyro[2] / mdpsPerDps / dpsPerRad;
    }
    ledManager.off();
    m_Calibration.G_off[0] = ((double)rawGxyz[0]) / gyroCalibrationSamples;
    m_Calibration.G_off[1] = ((double)rawGxyz[1]) / gyroCalibrationSamples;
    m_Calibration.G_off[2] = ((double)rawGxyz[2]) / gyroCalibrationSamples;

    #ifdef DEBUG_SENSOR
        m_Logger.trace("Gyro calibration results: %f %f %f", UNPACK_VECTOR_ARRAY(m_Calibration.G_off));
    #endif
}
#endif  // LSM6DSV_GYRO_OFFSET_CAL

#ifdef LSM6DSV_ACCEL_OFFSET_CAL
void LSM6DSVSensor::calibrateAccel() {
    MagnetoCalibration* magneto = new MagnetoCalibration();

    // Blink calibrating led before user should rotate the sensor
	m_Logger.info("Put the device into 6 unique orientations (all sides), leave it still and do not hold/touch for 3 seconds each");
    constexpr uint8_t ACCEL_CALIBRATION_DELAY_SEC = 3;
    ledManager.on();
    for (uint8_t i = ACCEL_CALIBRATION_DELAY_SEC; i > 0; i--) {
        m_Logger.info("%i...", i);
        delay(1000);
    }
    ledManager.off();

	RestDetectionParams calibrationRestDetectionParams;
	calibrationRestDetectionParams.restMinTimeMicros = 3 * 1e6;
	calibrationRestDetectionParams.restThAcc = 0.25f;
	RestDetection calibrationRestDetection(
		calibrationRestDetectionParams,
		1 / LSM6DSV_GYRO_RATE,
		1 / LSM6DSV_ACCEL_RATE
	);

	constexpr uint16_t expectedPositions = 6;
	constexpr uint16_t numSamplesPerPosition = 96;

	uint16_t numPositionsRecorded = 0;
	uint16_t numCurrentPositionSamples = 0;
	bool waitForMotion = true;

	float* accelCalibrationChunk = new float[numSamplesPerPosition * 3];
	ledManager.pattern(100, 100, 6);
	ledManager.on();
	m_Logger.info("Gathering accelerometer data...");
	m_Logger.info("Waiting for position %i, you can leave the device as is...", numPositionsRecorded + 1);
	while (true) {
		int32_t intAccel[3];
		if (imu.Get_X_Axes(intAccel) != LSM6DSV_OK) {
			m_Logger.error("Fatal error: couldn't get acceleration from imu!");
			return;
		}
		float scaled[3] = {
			(float)(intAccel[0] / mgPerG * CONST_EARTH_GRAVITY),
			(float)(intAccel[1] / mgPerG * CONST_EARTH_GRAVITY),
			(float)(intAccel[2] / mgPerG * CONST_EARTH_GRAVITY),
		};

		calibrationRestDetection.updateAcc(LSM6DSV_ACCEL_RATE, scaled);

		if (waitForMotion) {
			if (!calibrationRestDetection.getRestDetected()) {
				waitForMotion = false;
			}
			delayMicroseconds(1 / LSM6DSV_ACCEL_RATE);
			continue;
		}

		if (calibrationRestDetection.getRestDetected()) {
			const uint16_t i = numCurrentPositionSamples * 3;
			accelCalibrationChunk[i + 0] = intAccel[0];
			accelCalibrationChunk[i + 1] = intAccel[1];
			accelCalibrationChunk[i + 2] = intAccel[2];
			numCurrentPositionSamples++;

			if (numCurrentPositionSamples >= numSamplesPerPosition) {
				for (int i = 0; i < numSamplesPerPosition; i++) {
					magneto->sample(
						accelCalibrationChunk[i * 3 + 0],
						accelCalibrationChunk[i * 3 + 1],
						accelCalibrationChunk[i * 3 + 2]
					);
				}
				numPositionsRecorded++;
				numCurrentPositionSamples = 0;
				if (numPositionsRecorded < expectedPositions) {
					ledManager.pattern(50, 50, 2);
					ledManager.on();
					m_Logger.info("Recorded, waiting for position %i...", numPositionsRecorded + 1);
					waitForMotion = true;
				}
			}
		} else {
			numCurrentPositionSamples = 0;
		}

		if (numPositionsRecorded >= expectedPositions) break;

		delayMicroseconds(1 / LSM6DSV_ACCEL_RATE);
	}
	ledManager.off();
	m_Logger.debug("Calculating accelerometer calibration data...");
	delete[] accelCalibrationChunk;

    float A_BAinv[4][3];
    magneto->current_calibration(A_BAinv);
    delete magneto;

    m_Logger.debug("Finished calculating accelerometer calibration");
    m_Logger.debug("Accelerometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.A_B[i] = A_BAinv[0][i];
        m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
        m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
        m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    }
    m_Logger.debug("}");
}
#endif  // LSM6DSV_ACCEL_OFFSET_CAL

#ifdef LSM6DSV_GYRO_SENSITIVITY_CAL
void LSM6DSVSensor::calibrateGyroSensitivity() {
	m_Logger.info(
		"Gyro sensitivity calibration started on sensor #%d of type %s at address "
		"0x%02x",
		getSensorId(),
		getIMUNameByType(sensorType),
		addr
	);

	Quat rawRotationInit;
	Vector3 rawRotationFinal;
	uint8_t count = 0;
	float gyroCount[3];  // Use this to determine the axis spun
	float calculatedScale[3] = {1.0f, 1.0f, 1.0f};

	unsigned long prevLedTime = millis();
	constexpr uint16_t ledFlashDuration = 600;

	ledManager.off();

	m_Logger.info("");
	m_Logger.info(
		"  Step 0: Let the tracker sit, the light will flash when you should reorient "
		"the tracker"
	);
	m_Logger.info(
		"  Step 1: Move the tracker to a corner or edge that aligns the tracker to the "
		"same position every time"
	);
	m_Logger.info(
		"      NOTE: You might also want to unplug the USB so it doesn't affect spins"
	);
	m_Logger.info(
		"  Step 2: Let the tracker rest until the solid light turns on, you might need "
		"to hold it"
	);
	m_Logger.info("    against a wall depending on the case and orientation");
	m_Logger.info(
		"  Step 3: Rotate the tracker in the yaw axis for %d full rotations and align "
		"it with the previous edge ",
		LSM6DSV_GYRO_SENSITIVITY_SPINS
	);
	m_Logger.info(
		"      NOTE: The yaw axis is the direction of looking left or right with your "
		"head, perpendicular to gravity"
	);
	m_Logger.info("      NOTE: The light will turn off after you start moving it");

	m_Logger.info(
		"  Step 4: Wait for the flashing light then rotate the tracker 90 degrees "
		" to a new axis and "
	);
	m_Logger.info("    align with an edge. Repeat steps 2 and 3");

	m_Logger.info(
		"  Step 5: Wait for the flashing light then rotate the tracker 90 degrees so "
		"the last "
		"axis is up and"
	);
	m_Logger.info("    aligned with an edge. Repeat steps 2 and 3");

	imu.FIFO_Reset();
	delayMicroseconds(100);
	while (!sfusion.getRestDetected())  // Wait for rest
	{
		readNextFifoFrame();
	}

	while (count < 3) {
		ledManager.on();
		prevLedTime = millis();
		m_Logger.info("Move the tracker to a new axis then let sit");
		while (sfusion.getRestDetected()) {
			unsigned long now = millis();
			if (now - ledFlashDuration > prevLedTime) {
				// ledManager.toggle();
				ledManager.on();
				prevLedTime = millis();
			}
			if (sfusion.getRestDetected() && sfusion.isUpdated()) {
				rawRotationInit = sfusion.getQuaternionQuat();
				sfusion.clearUpdated();
			}
			readNextFifoFrame();
		}
		ledManager.off();
		while (!sfusion.getRestDetected()) {
			readNextFifoFrame();
		}

		ledManager.on();  // The user should rotate
		m_Logger.info("Rotate the tracker %d times", LSM6DSV_GYRO_SENSITIVITY_SPINS);
		gyroCount[0] = 0.0f;
		gyroCount[1] = 0.0f;
		gyroCount[2] = 0.0f;
		rawRotationInit = sfusion.getQuaternionQuat();
		while (sfusion.getRestDetected()) {
			if (sfusion.getRestDetected() && sfusion.isUpdated()) {
				rawRotationInit = sfusion.getQuaternionQuat();
				sfusion.clearUpdated();
			}
			readNextFifoFrame();
		}
		ledManager.off();

		while (!sfusion.getRestDetected()) {
			readNextFifoFrame();
			gyroCount[0] += rawGyro[0];
			gyroCount[1] += rawGyro[1];
			gyroCount[2] += rawGyro[2];
		}
		uint8_t isUp;
		rawRotationFinal
			= (sfusion.getQuaternionQuat() * rawRotationInit.inverse()).get_euler()
			* dpsPerRad;

		if (abs(gyroCount[0]) > abs(gyroCount[1])
			&& abs(gyroCount[0]) > abs(gyroCount[2])) {  // Spun in X
			imu.Get_6D_Orientation_XH(&isUp);
			if ((!isUp && gyroCount[0] > 0) || (isUp && gyroCount[0] < 0)) {
				calculatedScale[0]
					= (1.0
					   / (1.0
						  - ((-rawRotationFinal.y)
							 / (360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("X, Diff: %f", -rawRotationFinal.y);
			} else {
				calculatedScale[0]
					= (1.0
					   / (1.0
						  - ((rawRotationFinal.y)
							 / (360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("-X, Diff: %f", rawRotationFinal.y);
			}
		}

		else if (abs(gyroCount[1]) > abs(gyroCount[0]) && abs(gyroCount[1]) > abs(gyroCount[2])) {  // Spun in Y
			imu.Get_6D_Orientation_YH(&isUp);
			if ((isUp && gyroCount[1] > 0) || (!isUp && gyroCount[1] < 0)) {
				calculatedScale[1]
					= (1.0
					   / (1.0
						  - ((-rawRotationFinal.y)
							 / (360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("Y, Diff: %f", -rawRotationFinal.y);
			} else {
				calculatedScale[1]
					= (1.0
					   / (1.0
						  - ((rawRotationFinal.y)
							 / (360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("-Y, Diff: %f", rawRotationFinal.y);
			}
		}

		else if (abs(gyroCount[2]) > abs(gyroCount[0]) && abs(gyroCount[2]) > abs(gyroCount[1])) {  // Spun in Z
			imu.Get_6D_Orientation_ZH(&isUp);
			if ((isUp && gyroCount[2] > 0) || (!isUp && gyroCount[2] < 0)) {
				calculatedScale[2]
					= (1.0
					   / (1.0
						  - ((-rawRotationFinal.y)
							 / (360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("Z, Diff: %f", -rawRotationFinal.y);
			} else {
				calculatedScale[2]
					= (1.0
					   / (1.0
						  - ((rawRotationInit.y)
							 / (360.0f * LSM6DSV_GYRO_SENSITIVITY_SPINS))));
				m_Logger.info("-Z, Diff: %f", rawRotationInit.y);
			}
		}
		count++;
	}

	m_Calibration.G_sensitivity[0] = calculatedScale[0];
	m_Calibration.G_sensitivity[1] = calculatedScale[1];
	m_Calibration.G_sensitivity[2] = calculatedScale[2];

	m_Logger.info(
		"Gyro Sensitivity calibration results: %f %f %f",
		m_Calibration.G_sensitivity[0],
		m_Calibration.G_sensitivity[1],
		m_Calibration.G_sensitivity[2]
	);
}
#endif  // LSM6DSV_GYRO_SENSITIVITY_CAL

void LSM6DSVSensor::checkForCalibrationRequirements() {
	// We need to wait for a bit for the acceleration values to normalize
	// otherwise we get bogus values
	delay(100);

	int32_t intAcceleration[3];
	status |= imu.Get_X_Axes(intAcceleration);
	if (status != LSM6DSV_OK) {
		m_Logger.fatal(
			"The %s at 0x%02x returned an error while getting its acceleration!",
			getIMUNameByType(sensorType),
			addr
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	// TODO: implement axis remapping
	float zAccel = intAcceleration[2] / mgPerG;

	if (zAccel > -0.75) {
		loadIMUCalibration();
		return;
	}

#ifndef LSM6DSV_NO_SELF_TEST_ON_FACEDOWN
	if (runSelfTest() != LSM6DSV_OK) {
		m_Logger.fatal(
			"The %s at 0x%02x returned an error during the self test "
			"(maybe it wasn't on a flat surface?)",
			getIMUNameByType(sensorType),
			addr
		);
		ledManager.pattern(50, 50, 200);
		state = LSM6DSV_ERROR;
		return;
	}
#endif  // LSM6DSV_NO_SELF_TEST_ON_FACEDOWN

	m_Logger.info("Flip right side up in the next 5 seconds to start calibration.");
	delay(5000);

	status |= imu.Get_X_Axes(intAcceleration);
	if (status != LSM6DSV_OK) {
		m_Logger.fatal(
			"The %s at 0x%02x returned an error while getting its acceleration!",
			getIMUNameByType(sensorType),
			addr
		);
		ledManager.pattern(50, 50, 200);
		return;
	}

	zAccel = intAcceleration[2] / mgPerG;

	if (zAccel < 0.75) {
		loadIMUCalibration();
		return;
	}

	startCalibration(0);
}

void LSM6DSVSensor::startCalibration(int calibrationType) {
	m_Calibration.G_sensitivity[0] = 1.0f;
	m_Calibration.G_sensitivity[1] = 1.0f;
	m_Calibration.G_sensitivity[2] = 1.0f;

#ifdef LSM6DSV_GYRO_OFFSET_CAL
	calibrateGyro();
#endif

#ifdef LSM6DSV_ACCEL_OFFSET_CAL
	calibrateAccel();
#endif

#ifdef LSM6DSV_GYRO_SENSITIVITY_CAL
	calibrateGyroSensitivity();
#endif

	saveCalibration();
	if (status == LSM6DSV_ERROR) {
		return;
	}

	m_Logger.info("Calibration finished, enjoy");
}

void LSM6DSVSensor::saveCalibration() {
	m_Logger.debug("Saving the calibration data");

	SlimeVR::Configuration::CalibrationConfig calibration;
	calibration.type = SlimeVR::Configuration::CalibrationConfigType::LSM6DSV;
	calibration.data.lsm6dsv = m_Calibration;
	configuration.setCalibration(sensorId, calibration);
	configuration.save();
}

bool LSM6DSVSensor::hasAccelCalibration() {
    for (int i = 0; i < 3; i++) {
        if (m_Calibration.A_B[i] != 0.0 ||
            m_Calibration.A_Ainv[0][i] != 0.0 ||
            m_Calibration.A_Ainv[1][i] != 0.0 ||
            m_Calibration.A_Ainv[2][i] != 0.0)
            return true;
    }
    return false;
}

bool LSM6DSVSensor::hasGyroCalibration() {
    for (int i = 0; i < 3; i++) {
        if (m_Calibration.G_off[i] != 0.0)
            return true;
    }
    return false;
}

void LSM6DSVSensor::handleGyroSample(int32_t gyroSample[3]) {
	// TODO: tempcal here
	float rawGyro[3];
	rawGyro[0] = (float)gyroSample[0] / mdpsPerDps;
	rawGyro[1] = (float)gyroSample[1] / mdpsPerDps;
	rawGyro[2] = (float)gyroSample[2] / mdpsPerDps;

	// convert to rads/s
	rawGyro[0] /= dpsPerRad;
	rawGyro[1] /= dpsPerRad;
	rawGyro[2] /= dpsPerRad;

#ifdef LSM6DSV_GYRO_OFFSET_CAL
	rawGyro[0] -= m_Calibration.G_off[0];
	rawGyro[1] -= m_Calibration.G_off[1];
	rawGyro[2] -= m_Calibration.G_off[2];
#endif  // LSM6DSV_GYRO_OFFSET_CAL
#ifdef LSM6DSV_GYRO_SENSITIVITY_CAL
	rawGyro[0] *= m_Calibration.G_sensitivity[0];
	rawGyro[1] *= m_Calibration.G_sensitivity[1];
	rawGyro[2] *= m_Calibration.G_sensitivity[2];
#endif  // LSM6DSV_GYRO_SENSITIVITY_CAL

	sfusion.updateGyro(
		rawGyro,
		lsm6dsv_from_lsb_to_nsec(currentDataTime - previousDataTime) * 1e-9
	);

    optimistic_yield(100);
}

void LSM6DSVSensor::handleAccelSample(int32_t accelSample[3]) {
	float acceleration[3] = {
		(float)accelSample[0],
		(float)accelSample[1],
		(float)accelSample[2],
	};
	applyCalibrationAndScale(acceleration);
    sfusion.updateAcc(
		acceleration, 
		lsm6dsv_from_lsb_to_nsec(currentDataTime - previousDataTime) * 1e-9
	);

    optimistic_yield(100);
}

void LSM6DSVSensor::applyCalibrationAndScale(float acceleration[3]) {
    if (isAccelCalibrated) {
        #if useFullCalibrationMatrix == true
            float tmp[3];
            for (uint8_t i = 0; i < 3; i++)
                tmp[i] = (acceleration[i] - m_Calibration.A_B[i]);
            acceleration[0] = m_Calibration.A_Ainv[0][0] * tmp[0] + m_Calibration.A_Ainv[0][1] * tmp[1] + m_Calibration.A_Ainv[0][2] * tmp[2];
            acceleration[1] = m_Calibration.A_Ainv[1][0] * tmp[0] + m_Calibration.A_Ainv[1][1] * tmp[1] + m_Calibration.A_Ainv[1][2] * tmp[2];
            acceleration[2] = m_Calibration.A_Ainv[2][0] * tmp[0] + m_Calibration.A_Ainv[2][1] * tmp[1] + m_Calibration.A_Ainv[2][2] * tmp[2];
        #else
            for (uint8_t i = 0; i < 3; i++)
                Axyz[i] = (Axyz[i] - calibration->A_B[i]);
        #endif
    }
    acceleration[0] *= CONST_EARTH_GRAVITY / mgPerG;
    acceleration[1] *= CONST_EARTH_GRAVITY / mgPerG;
    acceleration[2] *= CONST_EARTH_GRAVITY / mgPerG;
}

#endif  // (LSM6DSV_FUSION_SOURCE == LSM6DSV_FUSION_ESP)
