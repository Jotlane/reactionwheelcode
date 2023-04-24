#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>
#include <VescUart.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;//changed the delay here from 100 to 10 (10Hz to 100Hz), if anything goes wrong change back to 100ms delay

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

double Setpoint, Input, Output;
double Kp=3, Ki=0, Kd=0;//2,5,1

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
bool coas = false;
float timer = 0;
bool timer_start = false;
VescUart UART;

void setup(void)
{
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial1) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);
  
  //while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  //PID PART
    //initialize the variables we're linked to
  Input = 0;
  Setpoint = 180;
  Output = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-50, 50);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  delay(BNO055_SAMPLERATE_DELAY_MS);

  uint8_t bob, gyro, accel, mag = 0;
  bno.getCalibration(&bob, &gyro, &accel, &mag);
//  if (bob != 0)
//  {
//    Output = 0;
//    Serial.println("CALIBRATED,STOP ALL CURRENT");
//  }
//  else
//  {
    Input = orientationData.orientation.z;
    Input += 180;
    if (Input>=360)
    {
      Input -=360;
    }
    else if (Input<0)
    {
      Input += 360;
    }
    if (Input > 185 || Input < 175)
    {
      myPID.Compute();
    }
    else
    {
      Output = 0;
    }
//  }
  if ((Input > 185 || Input < 175) && coas == false && timer_start == false)
  {
    timer = millis();
    timer_start = true;
  }
  if (millis() > timer + 500)
  {
    coas = true;
  }
  if (coas == true)
  {
    Output = 0;
  }
  if (Input < 185 && Input > 175)
  {
    coas = false;
    timer = 0;
    timer_start = false;
  }
  Serial.println("Input:");
  Serial.println(Input);
  Serial.println("Output:");
  Serial.println(Output);
//  Serial.println("Timer: ");
//  Serial.println(timer);
//  Serial.println("Coas");
//  Serial.println(coas);
  UART.setCurrent(Output);
  Serial.println("System Calibration: ");
  Serial.println(bob);
  Serial.println("Accel X: ");
  Serial.println(accelerometerData.acceleration.x);
  Serial.println("Accel Y: ");
  Serial.println(accelerometerData.acceleration.y);
  Serial.println("Accel Z: ");
  Serial.println(accelerometerData.acceleration.z);
  Serial.println("LIN X: ");
  Serial.println(linearAccelData.acceleration.x);
  Serial.println("LIN Y: ");
  Serial.println(linearAccelData.acceleration.y);
  Serial.println("LIN Z: ");
  Serial.println(linearAccelData.acceleration.z);
  Serial.println("ROT X: ");
  Serial.println(angVelocityData.gyro.x);
  Serial.println("ROT Y: ");
  Serial.println(angVelocityData.gyro.y);
  Serial.println("ROT Z: ");
  Serial.println(angVelocityData.gyro.z);
}
