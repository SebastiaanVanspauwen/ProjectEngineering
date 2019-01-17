/* INCLUDES */
#include <project.h>
#include <stdio.h>
#include <math.h>
#include "millis.h"
#include <stdbool.h>
/* DOCS */
//MPU6050 https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
//HMC5983 https://aerocontent.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5983_3_Axis_Compass_IC.pdf
//        https://github.com/D1W0U/Arduino-HMC5983/blob/master/HMC5983.h

//MS5611 https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036


#define NUMBER_OF_TESTS 1000
#define MPU6050_SENSOR_ADRESS 0x69 //Gyro,Accel
#define MS5611_SENSOR_ADRESS 0x77 //Altimeter
#define HMC5983_SENSOR_ADRESS 0x1E //Compass
#define MIN_MOTOR_SPEED 0
#define MAX_MOTOR_SPEED 15
#define MIN_PWM 1000
#define MAX_PWM 2000
#define MOTOR_FRONT 0x52
#define MOTOR_RIGHT 0x54
#define MOTOR_BACK 0x56
#define MOTOR_LEFT 0x58
#define DEBUG_TICK_RATE 25
#define MOTOR_TICK_RATE 25
/* Variabelen */
//UART
char buf[200] = "";

//Others
uint8_t databuffer[6];
uint8_t status[3];
float rad_to_deg = 180/3.141592654;
bool firstRun = true;
bool shouldRun = false;
bool debugPrint = false;
//Gyro
int16_t fullRawGyroXData, fullRawGyroYData, fullRawGyroZData = 0;
float fullRawGyroX, fullRawGyroY, fullRawGyroZ = 0;
float gyroErrorX, gyroErrorY, gyroErrorZ = 0;
float gyroAngleX, gyroAngleY, gyroAngleZ = 0;
bool gyroError = false;

//Accel
int16_t fullRawAccelXData, fullRawAccelYData, fullRawAccelZData = 0;
float accelRawAngleX, accelRawAngleY, accelRawAngleZ = 0;
float accelAngleErrorX = 0, accelAngleErrorY = 0;
float accelAngleX = 0, accelAngleY = 0;
float totalAngleX = 0, totalAngleY = 0;
float rawPowX, rawPowY, rawPowZ = 0;
bool accelError = false;   

//Tijd
int16_t time;
float millisCount;
float prevMillisCount;
float elapsedTime;

float millisCountDebug;
float prevMillisCountDebug;
float motorCount;
float prevMotorCount;
//Pid Variabelen (Gewenste hoeken, errors)
float roll_desired_angle = 0;
float pitch_desired_angle = 0;  
float yaw_desired_rotation = 0;
float roll_error = 0;
float pitch_error = 0; 
float roll_pid_p = 0;
float pitch_pid_p = 0;
float roll_previous_error = 0;
float pitch_previous_error = 0;

//Pid constanten (aanpassen voor PID controller)
float input_THROTTLE = 1500; //testing

double roll_kp = 3.5;//3.55
double roll_ki = 0.006;//0.003
double roll_kd = 2.05;//2.05

double pitch_kp = 3.55;//3.55
double pitch_ki = 0.003;//0.003
double pitch_kd = 2.05;//2.05

double pid_p_gain_yaw = 4.0;
double pid_i_gain_yaw = 0.02;
double pid_d_gain_yaw = 0.0;

float roll_pid_i = 0;
float pitch_pid_i = 0; 
float yaw_pid_i = 0;

float roll_pid_d = 0;
float pitch_pid_d = 0;
float yaw_pid_d = 0;

float roll_PID = 0;
float pitch_PID = 0;
float yaw_PID = 0;

//PWM motoren
float pwm_Front = 0;
float pwm_Back = 0;
float pwm_Left = 0;
float pwm_Right = 0;

float speed_Front = 0;
float speed_Back = 0;
float speed_Right = 0;
float speed_Left = 0;


//PPM
//Volatile gebruiken --> Global variables modified by an interrupt service routine
volatile uint32 ms_value = 0;
volatile uint32 ms_values[4] = {0,0,0,0};
volatile uint32 rf_values[4] = {0,0,0,0};
volatile uint32 THROTTLE, PITCH, ROLL, YAW = 0;
volatile uint8 i = 0;
volatile uint8 last_i = 5;
volatile uint8 check = 0;
float averageThrottle = 0;
volatile uint32 throttleSum = 0;
volatile uint32 throttle_moving_average[5] = {0,0,0,0,0};
float averageYaw = 0;
volatile uint32 yawSum = 0;
volatile uint32 yaw_moving_average[5] = {0,0,0,0,0};
uint8_t activateMotorSequence = 0;
bool activateMotor = false;


//Compass

uint8_t X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB = 0;
double HX, HZ, HY = 0;
double compassValue = 0;
float Xm,Ym,Zm;
float temperature;

/* Functies */
long map(long x, long in_min, long in_max, long out_min, long out_max);
int checkI2C_Communication();
void startBlocks();
void updateMillis();
void setSensorAdress();
void calibrateGyro();
void calibrateAccel();
void readGyro();
void readAccel();
void readCompass();
void readTemp();
void mapAngles();
void calculatePID();
void adjustPID();
void calculateMotorPWM();
void adjustMotorPWM();
void mapPWM();
void setLedPWM(char color, uint8 period, uint8 compare);
void showError();
void setMotorSpeeds();
void printDebugs();

CY_ISR( pin_Handler )
{
        //MS_VALUES[0] | PIN 2.4 | Channel 1 | ROLL     | Rechtste Joystick | Links -   | Rechts +
        //MS_VALUES[1] | PIN 2.5 | Channel 2 | PITCH    | Rechtste Joystick | Omlaag -  | Omhoog + 
        //MS_VALUES[2] | PIN 2.6 | Channel 3 | THROTTLE | Linkse Joystick   | Omlaag -  | Omhoog + 
        //MS_VALUES[3] | PIN 2.7 | Channel 4 | YAW      | Linkse Joystick   | Links -   | Rechts +    
    
        uint32 raw_ch_val = 0; 
      
        float32 PPM_perc, temp = 0;
    
        raw_ch_val = Timer_ReadCapture();
     
        PPM_perc = (float32)raw_ch_val / 504000;
        
        ms_value = ( 1 - PPM_perc ) * 21 *1000; //timer telt af dus de overblijvende waarde is de overblijvende tijd en niet de tijd die voorbij is gegaan
 
        
        check = 200;
        if (i == 3)
        {
            ms_values[i] = ms_value;
            
            //Als er geen foute waarden zijn --> update alle values van ms_values naar rf_values
            if(!(    ms_values[0] > 2000  || ms_values[0] < 1000 
                || ms_values[1] > 2000  || ms_values[1] < 1000
                || ms_values[2] > 2000  || ms_values[2] < 1000
                || ms_values[3] > 2000  || ms_values[3] < 1000 ))
            {
                            averageThrottle = 0;
            averageYaw = 0;
            throttleSum = 0;
            yawSum = 0;
                
                for (int x = 0 ; x < 4 ; x++)
                {
                    rf_values[x] = ms_values[x];
                }
                
                ROLL = rf_values[0];
                PITCH = rf_values[1];
                THROTTLE = rf_values[2];
                YAW = rf_values[3];
                         
                
                for (int x = 4; x > 0 ; x--)
                {
                    throttle_moving_average[x] = throttle_moving_average[x - 1];
                    yaw_moving_average[x] = yaw_moving_average[x - 1];
                }
                throttle_moving_average[0] = THROTTLE;
                yaw_moving_average[0] = YAW;
                
                for (int x = 0; x < 5 ; x++)
                {
                    throttleSum += throttle_moving_average[x];
                    yawSum += yaw_moving_average[x];
                }
                
                averageThrottle = (float)throttleSum / 5;
                averageYaw = (float)yawSum / 5;
                
                //Throttle naar onder
                if( averageThrottle < 1250 && averageThrottle > 0 && millisCount > 2500)
                {
                    if(activateMotorSequence == 0)
                    {
                        //SERIAL_PutString("Activating motors 0 \n \r");
                        //Yaw naar links
                        if( averageYaw < 1250 && averageYaw > 700)
                        {
                           //Mogelijkheid om motor aan te zetten
                           activateMotorSequence = 1;
                           setLedPWM('g', 30, 15);
                        }
                    }
                    
                    //Yaw terug in het midden
                    if (activateMotorSequence == 1)
                    {
                        //SERIAL_PutString("Activating motors 1 \n \r");
                        //Zet de motoren aan
                        if( averageYaw > 1250 && averageYaw != 0)
                        {
                            activateMotorSequence = 2;
                            activateMotor = true;
                            setLedPWM('g', 10, 5);
                        }
                    }
                                     
                    if (activateMotorSequence == 2)
                    {
                        //SERIAL_PutString("Activating motors 2 \n \r");
                        //Zet de motoren terug uit (yaw rechts)
                        if( averageYaw > 1800 && averageYaw != 0)
                        {
                            activateMotorSequence = 0;
                            activateMotor = false;
                            setLedPWM('g', 200, 100);                      
                        }  
                    }
                }             
            }    
            else
            {
               //SERIAL_PutString("Skipping values \n \r");  
            }
            i = 0;
        }
        else
        {
            ms_values[i] = ms_value;
            i++;
        }
        
        rf_channels_ClearInterrupt();
        Timer_ReadStatusRegister(); //interrupt clear
}

int main()
{
    /* Start alle blokken */
    startBlocks();
    CyGlobalIntEnable;
    channels_itr_StartEx( pin_Handler );
    setLedPWM('g', 255, 255);
    setLedPWM('r', 120, 60);
    updateMillis();
    setLedPWM('g', 150, 75);
    setLedPWM('r', 255, 200);
    shouldRun = false;
    int num = 0;
    while (1)
    {
        updateMillis(); 
        //1.5 sec wachten voordat er iets gedaan wordt
        if(millisCount > 1500)
        {
            if(firstRun)
            {
                setSensorAdress();
                checkI2C_Communication();
                SERIAL_PutString("OK! \n \r");
                shouldRun = true;
                setLedPWM('g', 150, 75);
                setLedPWM('r', 150, 75);
            }
        }
        if(shouldRun)
        {
            if(firstRun)
            {
                setSensorAdress();
                int shouldStop;
                shouldStop = checkI2C_Communication();
                if(shouldStop == -1)
                {
                    SERIAL_PutString("I2C Error! \n \r");
                    setLedPWM('r', 10, 5);
                    return -1;
                } 
                calibrateGyro();
                calibrateAccel();
                firstRun = false;
            }
            readGyro();
            readAccel(); 
            readCompass();
            readTemp();
            mapAngles();
            calculatePID();
            adjustPID();
            calculateMotorPWM();
            adjustMotorPWM();
            mapPWM();
            if (activateMotor)
            {           
                setMotorSpeeds();
            }
        }
        else
        {
            setLedPWM('g', 10, 5);
            if(!firstRun)
            {
                firstRun = true; 
            }
        }
    }
}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int checkI2C_Communication()
{
    //Als er iets niet klopt (I2C niet goed ingesteld)
    if(!(status[0] == 0 && status[1] == 16 && status[2] == 16))
    {        
        SERIAL_PutString("CONFIG ERROR: Setup failed \n \r");
        status[0] == 0 ? SERIAL_PutString("CONFIG Power OK \n \r") : SERIAL_PutString("CONFIG Power NOK \n \r");
        status[1] == 16 ? SERIAL_PutString("CONFIG Gyro OK \n \r") : SERIAL_PutString("CONFIG Gyro NOK \n \r");
        status[2] == 16 ? SERIAL_PutString("CONFIG Accel OK \n \r") : SERIAL_PutString("CONFIG Accel NOK \n \r");        
        showError();
        return -1;
    }
    else
    {
        SERIAL_PutString("I2C Configured Succesfully \n \r");
        return 1;
    }
}

void startBlocks()
{
    SERIAL_Start();
    PWM_GREEN_Start();
    PWM_RED_Start();
    SERIAL_PutString("UART STARTED \n \r");
    millis_Start();
    I2C_MPU6050_Start();
    Timer_Start();
}

void updateMillis()
{
    prevMillisCount = millisCount;
    millisCount = millis();
    elapsedTime = (millisCount - prevMillisCount) / 1000;  
    
    millisCountDebug = millis();
    if (millisCountDebug - prevMillisCountDebug > DEBUG_TICK_RATE)
    {
        prevMillisCountDebug = millisCount;
        printDebugs();
        //SERIAL_PutString("I should print debug \n \r");
    }
    if (motorCount - prevMotorCount > MOTOR_TICK_RATE)
    {
        prevMotorCount = millisCount;
        //setMotorSpeeds();   
    }
    
}

void setSensorAdress()
{
    //MPU6050
    
    SERIAL_PutString("Configuring MPU6050\n \r");
    I2CWriteByte(MPU6050_SENSOR_ADRESS, 0x6B, 0x00);
    I2CWriteByte(MPU6050_SENSOR_ADRESS, 0x1B, 0x10);
    I2CWriteByte(MPU6050_SENSOR_ADRESS, 0x1C, 0x10);  
    SERIAL_PutString("Checking MPU6050 config\n \r");
    I2CReadBytes(MPU6050_SENSOR_ADRESS, 0x6B, 1, databuffer);
    status[0] = databuffer[0];   
    I2CReadBytes(MPU6050_SENSOR_ADRESS, 0x1B, 1, databuffer);
    status[1] = databuffer[0];
    I2CReadBytes(MPU6050_SENSOR_ADRESS, 0x1C, 1, databuffer);
    status[2] = databuffer[0];
      
    SERIAL_PutString("Setting HMC5983 Adress \n \r");    
    //HMC5983
    
    //Set Gain Range
    I2CWriteByte(HMC5983_SENSOR_ADRESS, 0x01, 0b111 << 5);
    
    //Set Data Rate
    uint8_t temp;
    I2CReadByte(HMC5983_SENSOR_ADRESS, 0x00, temp);
    temp &= 0b11100011;
    temp |= (0b111 << 2);
    I2CWriteByte(HMC5983_SENSOR_ADRESS, 0x00, temp);
   
    //Set Sample Average
    temp = 0x00;
    I2CReadByte(HMC5983_SENSOR_ADRESS, 0x00, temp);
    temp &= 0b10011111;
    temp |= (0b01 << 5);
    
    I2CWriteByte(HMC5983_SENSOR_ADRESS, 0x00, temp);  
    //Set Measurement Mode
    temp = 0x00;
    I2CReadByte(HMC5983_SENSOR_ADRESS, 0x02, temp);
    temp &= 0b11111100;
    temp |= 0b00;
    I2CWriteByte(HMC5983_SENSOR_ADRESS, 0x02, temp);
    
    
    

}

void calibrateGyro()
{
    SERIAL_PutString("config gyro \n \r");
    if (!gyroError)
    {
        for (int i = 0; i < NUMBER_OF_TESTS; i ++)
        {
            I2CReadBytes(MPU6050_SENSOR_ADRESS, 0x43, 6, databuffer);

            fullRawGyroXData = (databuffer[0] << 8 | databuffer[1]);
            fullRawGyroYData = (databuffer[2] << 8 | databuffer[3]);
            fullRawGyroZData = (databuffer[4] << 8 | databuffer[5]);

            fullRawGyroX += (float)fullRawGyroXData / 32.8;
            fullRawGyroY += (float)fullRawGyroYData / 32.8;
            fullRawGyroZ += (float)fullRawGyroZData / 32.8;
            /*
            sprintf(buf, "Gyro OFFSET X:%d Y:%d Z:%D \n \r", fullRawGyroXData,fullRawGyroYData, fullRawGyroZData);
            SERIAL_PutString(buf);
            */
            /*
            sprintf(buf, "Gyro OFFSET REAL X:%f Y:%f \n \r", fullRawGyroX,fullRawGyroY);
            SERIAL_PutString(buf);    
            */
            gyroError = true;
        }
        gyroErrorX = fullRawGyroX / NUMBER_OF_TESTS;
        gyroErrorY = fullRawGyroY / NUMBER_OF_TESTS; 
        gyroErrorZ = fullRawGyroZ / NUMBER_OF_TESTS;
    }
}

void calibrateAccel()
{
    SERIAL_PutString("config accell \n \r");
    if (!accelError)
    {
        for (int i = 0; i < NUMBER_OF_TESTS; i ++)
        {
            I2CReadBytes(MPU6050_SENSOR_ADRESS, 0x3B, 6, databuffer);

            fullRawAccelXData = (databuffer[0] << 8 | databuffer[1]);
            fullRawAccelYData = (databuffer[2] << 8 | databuffer[3]);
            fullRawAccelZData = (databuffer[4] << 8 | databuffer[5]);

            accelRawAngleX = (float)fullRawAccelXData / 4096.0;
            accelRawAngleY = (float)fullRawAccelYData / 4096.0;
            accelRawAngleZ = (float)fullRawAccelZData / 4096.0;
            /*
            sprintf(buf, "Accel Raw X:%f Y:%f Z:%f \n \r", (float)accelRawAngleX, (float)accelRawAngleY, (float)accelRawAngleZ);
            SERIAL_PutString(buf);
            */
            rawPowX = pow( (accelRawAngleX), 2);
            rawPowY = pow( (accelRawAngleY), 2);
            rawPowZ = pow( (accelRawAngleZ), 2);
                        
            //Euler
            accelAngleErrorX = accelAngleErrorX + atan( ( accelRawAngleY / sqrt (rawPowX + rawPowZ ) ) ) * rad_to_deg; 
            accelAngleErrorY = accelAngleErrorY + atan( ( accelRawAngleX / sqrt (rawPowY + rawPowZ ) ) ) * rad_to_deg; 
            /*
            sprintf(buf, "Accel Error X:%f Y:%f\n \r", (float)accelAngleErrorX, (float)accelAngleErrorY);
            SERIAL_PutString(buf);
            */

        }
        accelError = true;
        accelAngleErrorX = accelAngleErrorX / NUMBER_OF_TESTS;
        accelAngleErrorY = accelAngleErrorY / NUMBER_OF_TESTS; 
    }
}

void readGyro()
{
    I2CReadBytes(MPU6050_SENSOR_ADRESS, 0x43, 6, databuffer);

    fullRawGyroXData = (databuffer[0] << 8 | databuffer[1]);
    fullRawGyroYData = (databuffer[2] << 8 | databuffer[3]);
    fullRawGyroZData = (databuffer[4] << 8 | databuffer[5]);

    fullRawGyroX = (float)(fullRawGyroXData / 32.8) - gyroErrorX;
    fullRawGyroY = (float)(fullRawGyroYData / 32.8) - gyroErrorY;
    fullRawGyroZ = (float)(fullRawGyroYData / 32.8) - gyroErrorZ;

    gyroAngleX = fullRawGyroX * elapsedTime;
    gyroAngleY = fullRawGyroY * elapsedTime;
    gyroAngleZ = fullRawGyroZ * elapsedTime;
}

void readAccel()
{
    I2CReadBytes(MPU6050_SENSOR_ADRESS, 0x3B, 6, databuffer);

    fullRawAccelXData = (databuffer[0] << 8 | databuffer[1]);
    fullRawAccelYData = (databuffer[2] << 8 | databuffer[3]);
    fullRawAccelZData = (databuffer[4] << 8 | databuffer[5]);

    accelRawAngleX = (float)fullRawAccelXData / 4096.0;
    accelRawAngleY = (float)fullRawAccelYData / 4096.0;
    accelRawAngleZ = (float)fullRawAccelZData / 4096.0;

    accelAngleX = ((atan((accelRawAngleY) / sqrt(pow( (accelRawAngleX), 2) + pow( (accelRawAngleZ), 2))) * rad_to_deg )) - accelAngleErrorX;
    accelAngleY = ((atan( -1 * (accelRawAngleX) / sqrt(pow( (accelRawAngleY), 2) + pow( (accelRawAngleZ), 2))) * rad_to_deg )) - accelAngleErrorY;

    totalAngleX = 0.98 * (totalAngleX + gyroAngleX) + 0.02 * accelAngleX;
    totalAngleY = 0.98 * (totalAngleY + gyroAngleY) + 0.02 * accelAngleY;
}

void readCompass()
{
    I2CReadBytes(HMC5983_SENSOR_ADRESS, 0x03, 6, databuffer);
    
    HX  = ((databuffer[0] << 8) + databuffer[1]);
    HY  = ((databuffer[2] << 8) + databuffer[3]);
    HZ  = ((databuffer[4] << 8) + databuffer[5]);
    
    if (HX > 0x07FF)
    {
       HX = 0xFFFF - HX; 
    }
    
    if (HY  > 0x07FF)
    {
       HY  = 0xFFFF - HY; 
    } 
    
    if (HZ  > 0x07FF)
    {
       HZ  = 0xFFFF - HZ; 
    }  
    
    double H = 0;
    
    if (HY > 0)
    {
        H = 90.0 - atan(HX / HY) * 180.0 / M_PI;
    }
    if (HY < 0)
    {
        H = 270.0 - atan(HX / HY) * 180.0 / M_PI;
    }
    if (HY == 0 && HX < 0)
    {
       H = 180; 
    }
    if (HY == 0 && HX > 0)
    {
       H = 0; 
    }
    
    compassValue = H;
}

void readTemp()
{
    uint8_t tempMSB, tempLSB = 0;
    I2CReadBytes(HMC5983_SENSOR_ADRESS, 0x31, 2, databuffer);

    tempMSB = databuffer[0];
    tempLSB = databuffer[1];
       
    temperature = (tempMSB * 2^8 + tempLSB) / (2^4 * 8) + 25;

}

void mapAngles()
{
    roll_desired_angle = map(rf_values[0], 1000, 2000, -10, 10);
    pitch_desired_angle = map(rf_values[1], 1000, 2000, -10, 10); 
    yaw_desired_rotation = map(rf_values[3], 1000, 2000, -10, 10);
}

void calculatePID()
{
    roll_error = totalAngleX - roll_desired_angle;
    pitch_error = totalAngleY - pitch_desired_angle;

    roll_pid_p = roll_kp * roll_error;
    pitch_pid_p = pitch_kp * pitch_error;
    
    /*
    sprintf(buf, "roll error is %f pitch error is %f\n \r", roll_error, pitch_error);
    SERIAL_PutString(buf); 
    */
    /*
    sprintf(buf, "roll_pid_p is %f pitch_pid_p is %f\n \r", roll_pid_p, pitch_pid_p);
    SERIAL_PutString(buf);  
    */
    
    roll_pid_i = roll_pid_i + ( roll_ki * roll_error);  
    pitch_pid_i = pitch_pid_i + (pitch_ki * pitch_error);  

    //Derivative
    roll_pid_d = roll_kd * ((roll_error - roll_previous_error) / elapsedTime);
    pitch_pid_d = pitch_kd * ((pitch_error - pitch_previous_error) / elapsedTime);
    /*
    sprintf(buf, "elapsedTime is %f (roll_error - roll_previous_error) is %f\n \r", elapsedTime, (roll_error - roll_previous_error));
    SERIAL_PutString(buf);   
    */
    //Total = Proportional + Integral + Derivative
    roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
    pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d; 
    
    /*
    sprintf(buf, "roll_PID is %f pitch_PID is %f\n \r", roll_PID, pitch_PID);
    SERIAL_PutString(buf);   
    */
    
    roll_previous_error = roll_error;
    pitch_previous_error = pitch_error;   
}

void adjustPID()
{
    if (roll_PID < -400)
    {
        roll_PID=-400;
    }
    if (roll_PID > 400)
    {
        roll_PID=400; 
    }
    if (pitch_PID < -400)
    {
        pitch_PID=-400;
    }
    if (pitch_PID > 400) 
    {
        pitch_PID=400;
    }
}

void calculateMotorPWM()
{
    pwm_Front = rf_values[2] + pitch_PID;
    pwm_Back  = rf_values[2] - pitch_PID;
    pwm_Right = rf_values[2] + roll_PID;
    pwm_Left  = rf_values[2] - roll_PID;
}

void adjustMotorPWM()
{
    if (pwm_Front > 2000)
    {
        pwm_Front = 2000;
    }
    if (pwm_Front < 1100)
    {
        pwm_Front = 1100;
    }
    if (pwm_Back > 2000)
    {
        pwm_Back = 2000;
    }
    if (pwm_Back < 1100)
    {
        pwm_Back = 1100;
    }

    if (pwm_Right > 2000)
    {
        pwm_Right = 2000;
    }
    if (pwm_Right < 1100)
    {
        pwm_Right = 1100;
    }
    if (pwm_Left > 2000)
    {
        pwm_Left = 2000;
    }
    if (pwm_Left < 1100)
    {
        pwm_Left = 1100;
    }
}
void mapPWM()
{
    speed_Front = map(pwm_Front ,MIN_PWM, MAX_PWM, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED); 
    speed_Back = map(pwm_Back ,MIN_PWM, MAX_PWM, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    speed_Right = map(pwm_Right ,MIN_PWM, MAX_PWM, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    speed_Left = map(pwm_Left ,MIN_PWM, MAX_PWM, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);    
}

void setLedPWM(char color, uint8 period, uint8 compare)
{
    switch(color)
    {
        case 'r':
        { 
            PWM_RED_WritePeriod(period);
            PWM_RED_WriteCompare(compare);
        }
        break;
        case 'g':
        {
            PWM_GREEN_WritePeriod(period);
            PWM_GREEN_WriteCompare(compare);
        }
        break;
        default:
            SERIAL_PutString("How did you get here?");
        break;
    }
}

void showError()
{
    SERIAL_PutString("Error!!");
    setLedPWM('g', 10, 5);
    setLedPWM('r', 10, 5);
}

void setMotorSpeeds()
{
    //SERIAL_PutString("Setting motor speeds \n \r");
    
    I2CWriteByte(MOTOR_FRONT >> 1, 0x00, (uint8) speed_Front);
    I2CWriteByte(MOTOR_RIGHT >> 1, 0x00, (uint8) speed_Right); 
    I2CWriteByte(MOTOR_LEFT  >> 1, 0x00, (uint8) speed_Left); 
    I2CWriteByte(MOTOR_BACK  >> 1, 0x00, (uint8) speed_Back); 
    
}

void printDebugs()
{
 
    /*
    sprintf(buf, "pwm front is %f pwm back is %f pwn left is %f pwm right is %f \n \r", pwm_Front, pwm_Back, pwm_Left, pwm_Right);
    SERIAL_PutString(buf);  
    */
    
    /*
    sprintf(buf, "speed front is %f speed back is %f speed left is %f speed right is %f \n \r", speed_Front, speed_Back, speed_Left, speed_Right);
    SERIAL_PutString(buf);    
    */
    
    /*
    if(check == 200)
    {
        sprintf(buf, "ROLL: %lu || PITCH: %lu || THROTTLE: %lu || YAW: %lu\r\n", rf_values[0], rf_values[1], rf_values[2], rf_values[3]);
        SERIAL_PutString(buf);
        check = 0;
    }
    */
    
    /*
    if(check == 200)
    {
        sprintf(buf, "MAV: %lu || MAV: %lu || MAV: %lu || MAV: %lu || MAV : %lu \r\n", throttle_moving_average[0], throttle_moving_average[1], throttle_moving_average[2], throttle_moving_average[3], throttle_moving_average[4]);
        SERIAL_PutString(buf);
        check = 0;
    }
    */
    /*
    sprintf(buf, "roll error is %f pitch error is %f\n \r", roll_error, pitch_error);
    SERIAL_PutString(buf);    
    */  
    
    /*  
    sprintf(buf, "speed front %d speed back %d speed right %d speed left %d \n \r", (uint8) speed_Front, (uint8) speed_Back, (uint8) speed_Right, (uint8) speed_Left);
    SERIAL_PutString(buf); 
    */
    //totalAngleX = 0.98 * (totalAngleX + gyroAngleX) + 0.02 * accelAngleX;     
        
    
    sprintf(buf, "%lu, %lu,%f,%f,%d \n \r", THROTTLE, YAW, averageThrottle, averageYaw, activateMotor);
    SERIAL_PutString(buf);     
    
    /*
    sprintf(buf, "Angle X %f Angle Y %f\n \r", totalAngleX, totalAngleY);
    SERIAL_PutString(buf);  
    */
    
    /*
    sprintf(buf, "Is motor active? %d\n \r", activateMotor);
    SERIAL_PutString(buf);  
    */
    
    /*
    sprintf(buf, "temp %f\n \r", temperature);
    SERIAL_PutString(buf);  
    */
    
    
}