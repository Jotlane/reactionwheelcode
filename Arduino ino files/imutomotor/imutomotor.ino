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
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

double Setpoint, Input, Output;
double Kp=1, Ki=0, Kd=0;//2,5,1

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


VescUart UART;

void setup(void)
{
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial1) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);
  
  while (!Serial) delay(10);  // wait for serial port to open!

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
  myPID.SetOutputLimits(-3, 3);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//  Serial.print(orientationData.orientation.x);

//  int8_t boardTemp = bno.getTemp();
//  Serial.println();
//  Serial.print(F("temperature: "));
//  Serial.println(boardTemp);
//
//  uint8_t system, gyro, accel, mag = 0;
//  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.println();
//  Serial.print("Calibration: Sys=");
//  Serial.print(system);
//  Serial.print(" Gyro=");
//  Serial.print(gyro);
//  Serial.print(" Accel=");
//  Serial.print(accel);
//  Serial.print(" Mag=");
//  Serial.println(mag);
//
//  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);

  Input = orientationData.orientation.x;
  Input += 180;
  if (Input>=360)
  {
    Input -=360;
  }
  else if (Input<0)
  {
    Input += 360;
  }
  myPID.Compute();
  Serial.println("Input:");
  Serial.println(Input);
  Serial.println("Output:");
  Serial.println(Output);
  UART.setCurrent(Output);
}
