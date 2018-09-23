/* ========================================
 *
 * Copyright Samuel Walsh, 2014
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Samuel Walsh.
 *
 * ========================================
*/
#include <project.h>
#include <mpu6050.h>
#include <stdio.h>

int main()
{
    int numberofTests = 250;
    char buf[50]; //just to hold text values in for writing to UART
    int i = 0;
    
    //RAW VALUES
    int16_t accelX,accelY,accelZ;
    int16_t gyroX,gyroY,gyroZ;
    //OFFSET VALUES
    float accelXOFF,accelYOFF,accelZOFF;
    float gyroXOFF,gyroYOFF,gyroZOFF;
    //REAL VALUES
    float   AX, AY, AZ; //acceleration floats
    float   GX, GY, GZ; //gyroscope floats
    
    uint8 devId;
	I2C_MPU6050_Start();
	SERIAL_Start();
    
    SERIAL_UartPutString("UART STARTED \n \r");
    CyGlobalIntEnable;
	MPU6050_init();
    SERIAL_UartPutString("MPU ADRESS SET \n \r");
	MPU6050_initialize();
    SERIAL_UartPutString("MPU INITIALIZED \n \r");
    SERIAL_UartPutString("TESTING CONNECTION.... \n \r");
	SERIAL_UartPutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");
    SERIAL_UartPutString("Raw values from gyroscope..\n\r");
        
    for(i=0; i<100; i++)
    { 
      accelX = MPU6050_getAccelerationX();
      accelY = MPU6050_getAccelerationY();
      accelZ = MPU6050_getAccelerationZ();   
      gyroX = MPU6050_getRotationX();
      gyroY = MPU6050_getRotationY();
      gyroZ = MPU6050_getRotationZ();
    
      accelXOFF += accelX;
      accelYOFF += accelY;
      accelZOFF += accelZ;
    
      gyroXOFF += gyroX;
      gyroYOFF += gyroY;
      gyroZOFF += gyroZ;    
    
      sprintf(buf, "Acceleration X:%d Y:%d Z:%d \t Gyro: X:%d Y:%d Z:%d \t", accelX,accelY,accelZ,gyroX,gyroY,gyroZ);
      SERIAL_UartPutString(buf);
      SERIAL_UartPutString("\n\r");
      CyDelay(5);
    }
      accelXOFF = accelXOFF/100;
      accelYOFF = accelYOFF/100;
      accelZOFF = accelZOFF/100;
    
      gyroXOFF = gyroXOFF/100;
      gyroYOFF = gyroYOFF/100;
      gyroZOFF = gyroZOFF/100;
      SERIAL_UartPutString("\n\nTest finished, offset values are shown below\n\n\r");
      sprintf(buf, "Acceleration OFFSET X:%d Y:%d Z:%d \t Gyro: X:%d Y:%d Z:%d \t", (int)accelXOFF,(int)accelYOFF,(int)accelZOFF,(int)gyroXOFF,(int)gyroYOFF,(int)gyroZOFF);
      SERIAL_UartPutString(buf);
      SERIAL_UartPutString("\r\n\nConverting Values to G\'s\n\n\r");  
      while(1)
      {
      //Convert values to G's
      accelX = MPU6050_getAccelerationX();
      accelY = MPU6050_getAccelerationY();
      accelZ = MPU6050_getAccelerationZ();   
      gyroX = MPU6050_getRotationX();
      gyroY = MPU6050_getRotationY();
      gyroZ = MPU6050_getRotationZ();
    
      AX = (float)((accelX-accelXOFF)/16384.00);
      AY = ((float)accelY-accelYOFF)/16384.00; //16384 is just 32768/2 to get our 1G value
      AZ = ((float)accelZ-(accelZOFF-16384))/16384.00; //remove 1G before dividing
    
      GX = ((float)gyroX-gyroXOFF)/131.07; //131.07 is just 32768/250 to get us our 1deg/sec value
      GY = ((float)gyroY-gyroYOFF)/131.07;
      GZ = ((float)gyroZ-gyroZOFF)/131.07; 
    
      sprintf(buf, "AX:%f \t AY:%f \t AZ:%f \t GX:%f \t GY:%f \t GZ:%f \t \n \r",AX,AY,AZ,GX,GY,GZ);
      SERIAL_UartPutString(buf);
    
      CyDelay(15);
        
    }
}
