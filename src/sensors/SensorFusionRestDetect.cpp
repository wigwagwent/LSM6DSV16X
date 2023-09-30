#include "SensorFusionRestDetect.h"

namespace SlimeVR
{
    namespace Sensors
    {
        #if !SENSOR_FUSION_WITH_RESTDETECT
        void SensorFusionRestDetect::updateAcc(sensor_real_t Axyz[3], sensor_real_t deltat)
        {
            if (deltat < 0) deltat = accTs;
            restDetection.updateAcc(deltat * 1e6, Axyz);
            SensorFusion::updateAcc(Axyz, deltat);
        }

        void SensorFusionRestDetect::updateGyro(sensor_real_t Gxyz[3], sensor_real_t deltat)
        {
            if (deltat < 0) deltat = gyrTs;
            restDetection.updateGyr(deltat * 1e6, Gxyz);
            SensorFusion::updateGyro(Gxyz, deltat);
        }
        #endif

        bool SensorFusionRestDetect::getRestDetected()
        {
            #if !SENSOR_FUSION_WITH_RESTDETECT
                return restDetection.getRestDetected();
            #elif SENSOR_USE_VQF
                return vqf.getRestDetected();
            #endif
        }

        bool SensorFusionRestDetect::getMoveDetected()
        {
            return restDetection.getMoveDetected();

        }

        void SensorFusionRestDetect::updateRestDetectionParameters(RestDetectionParams newParams)
        {
            restDetection.updateRestDetectionParameters(newParams);
        }
    }
}