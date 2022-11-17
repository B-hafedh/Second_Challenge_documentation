/*the code below calculates the acceleration, rotation, and temperature
 from the MPU6050 sensor using kalman filter, over I2C interface. */


#include <math.h> 
#include "mpu6050.h"

#define RAD2DEG 57.295779513082320876798154814105 // radians to degrees factor(180/pi).
#define WHOAMI 0x75 // Device ID (contains I2C address of the MPU6050: 0x68 by default).
#define PWR_MGMT 0x6B // Power Management Register address (configure the power mode of the sensor).
#define SMPLRT_DIV 0x19 // Sample Rate divider Register address.
#define ACCEL_CONFIG 0x1C //Accelerometer Configuration Register address.
#define ACCEL_XOUT_H 0x3B // X axis accelerometer measurement Register address.
#define TEMP_OUT_H 0x41   // Temperature sensor measurement Register address.
#define GYRO_CONFIG 0x1B // Gyroscope Configuration Register address.
#define GYRO_XOUT_H 0x43 //  X axis Gyroscope measurement Register address.
#define MPU6050_ADDR 0xD0 //Target device address.
const uint16_t I2C_TIMEOUT = 100; //Used to check for errors in I2C communication.
const double Acc_Z_corrector = 14418.0; // Z Axis Acceleration Sensitivity Scale Factor.
uint32_t timer;
/* Use a kalman filter to reduce errors and get accurate measurements.*/
Filter_t FilterX = {   //Initialize Kalman filter variables:
    .Q_ANGLE = 0.001f, // Process noise for the accelerometer.
    .Q_BIAS = 0.003f,  // Process noise for the gyroscope(gyro drifting). 
    .R_MEASURE = 0.03f // Measurement noise.
    };

Filter_t FilterY = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f,
};
// The MPU6050 sensor configuration.
uint8_t MPU_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHOAMI, 1, &check, 1, I2C_TIMEOUT);// verify the device ID (WHOAMI).

    if (check == 104) // Verify that the device I2C address is 0x68.
    {
        // Set the PWR_MGMT register value to 0 to wake up the sensor.
        Data = 0; 
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT, 1, &Data, 1, I2C_TIMEOUT);
        // Set the Sampling Rate to 1Khz.
        Data = 0x07; 
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, I2C_TIMEOUT);
        /* Accelerometer configuration (self test(ST) & full scale range [2g,4g,8g,16g]):
           XA_ST=0,YA_ST=0,ZA_ST=0, and AFS_SEL=0 (2g)*/
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, I2C_TIMEOUT);
        /* Gyroscopic configuration (self test(ST) & full scale range ±[250,500,1000,2000]°/s):
           XG_ST=0,YG_ST=0,ZG_ST=0, and FS_SEL=0 (±250°/s)*/
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, I2C_TIMEOUT);
        return 0;
    }
    return 1;
}

void Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)// Read accelerometer measurements.
{
    uint8_t Rec_Data[6];
    /* Read 6 bytes of data (2bytes for each axis) starting from ACCEL_XOUT_H register 
    and store them in Rec_Data table. */ 
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);

    // set the Accelerometer RAW values to the corresponding values from Rec_Data.
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /* Convert the RAW values into acceleration in 'g' by dividing each value by the Sensitivity Scale Factor:
       we set AFS_SEL=0 which corresponds to 16384.0 according to the datasheet */
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector;
}

void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) // Read Gyroscope measurements.
{
    uint8_t Rec_Data[6];
    /* Read 6 bytes of data (2bytes for each axis) starting from GYRO_XOUT_H register 
    and store them in Rec_Data table. */ 
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);

    // set the Gyroscope RAW values to the corresponding values from Rec_Data.
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /* Convert the RAW values into degree per second (°/s) by dividing each value by the Sensitivity Scale Factor:
       we set FS_SEL=0 which corresponds to 131.0 according to the datasheet*/
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) // Read Temperature sensor measurements.
{
    uint8_t Rec_Data[2];
    int16_t temp;
    //Read 2 bytes of data starting from TEMP_OUT_H register.
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2, I2C_TIMEOUT);

    /*Convert the temperature values into degree (°C) by dividing
      by Sensitivity scale(340.0) + Temperature Offset(36.53) according to the datasheet*/
    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) //// Read all sensors measurements.
{
    uint8_t Rec_Data[14];
    int16_t temp;
    /* Read 14 bytes of data from Accelerometer, Gyroscope, and Temperature sensors
       starting  from ACCEL_XOUT_H register. */
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 14, I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
    
    // roll and pitch angles calculation from accelerometer measurements.
    // Source: https://www.nxp.com/docs/en/application-note/AN3461.pdf (page 10, Eq28 and Eq29).
    double dt = (double)(HAL_GetTick() - timer) / 1000; // Calculate delta time (in seconds).
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD2DEG;// roll value in degree (Eq28).
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD2DEG; // pitch value in degree (Eq29). 
    /* Restrict the pitch angle to lie between -90° and +90° to eliminate duplicate solutions,
       for more details see: https://www.nxp.com/docs/en/application-note/AN3461.pdf page 11.*/
    if ((pitch < -90 && DataStruct->FilterAngleY > 90) || (pitch > 90 && DataStruct->FilterAngleY < -90))
    {
        FilterY.angle = pitch;
        DataStruct->FilterAngleY = pitch;
    }
    else
    {   
        DataStruct->FilterAngleY = Filter_getAngle(&FilterY, pitch, DataStruct->Gy, dt); // Calculate the Y axis angle using Kalman filter.
    }
    if (fabs(DataStruct->FilterAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->FilterAngleX = Filter_getAngle(&FilterX, roll, DataStruct->Gx, dt);// Calculate the X axis angle using Kalman filter.
}
/* Angle estimation based on Kalman filter equations, for more details see:
   https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/ */
double Filter_getAngle(Filter_t *Filter, double newAngle, double newRate, double dt)
{
    // Estimate the current state.
    double rate = newRate - Filter->bias; // Unbiased rate.
    Filter->angle += dt * rate; // angle estimation.
    // Update the estimation error covariance.
    Filter->P[0][0] += dt * (dt * Filter->P[1][1] - Filter->P[0][1] - Filter->P[1][0] + Filter->Q_ANGLE);
    Filter->P[0][1] -= dt * Filter->P[1][1];
    Filter->P[1][0] -= dt * Filter->P[1][1];
    Filter->P[1][1] += Filter->Q_BIAS * dt;
    
    double S = Filter->P[0][0] + Filter->R_MEASURE; // Error estimation.
    // Kalman gain calculation.
    double K[2]; 
    K[0] = Filter->P[0][0] / S;
    K[1] = Filter->P[1][0] / S;
    // Update the angle estimation.
    double y = newAngle - Filter->angle; // angle difference.
    Filter->angle += K[0] * y;
    Filter->bias += K[1] * y;
    // Update the estimation error covariance.
    double P00_temp = Filter->P[0][0];
    double P01_temp = Filter->P[0][1];

    Filter->P[0][0] -= K[0] * P00_temp;
    Filter->P[0][1] -= K[0] * P01_temp;
    Filter->P[1][0] -= K[1] * P00_temp;
    Filter->P[1][1] -= K[1] * P01_temp;

    return Filter->angle; // return the estimated angle.
};