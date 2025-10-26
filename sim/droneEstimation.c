#include "droneEstimation.h"
#include "math.h"

void attitudeComplementaryFilter(DRONE_T* drone)
{

    float biasToGyro = 0.99;

    //gyro part
    float gyroAngleEstimate;
    gyroAngleEstimate = drone->estimation.angle + drone->sensors.gyroscope * drone->dt;

    //accelerometer part
    float accelerometerAngleEstimate;
    accelerometerAngleEstimate = atan2f(drone->sensors.accelerometer.x, drone->sensors.accelerometer.y);

    //combine both parts
    drone->estimation.angle = biasToGyro * gyroAngleEstimate + (1-biasToGyro) * accelerometerAngleEstimate;
}