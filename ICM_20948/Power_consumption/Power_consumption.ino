/****************************************************************
 * Example2_Advanced.ino
 * ICM 20948 Arduino Library Demo
 * Shows how to use granular configuration of the ICM 20948
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/


#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <BMP388_DEV.h> 

#define SERIAL_PORT Serial

#define SPI_PORT SPI     // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 5000000 // You can override the default SPI frequency
#define CS_PIN 2         // Which pin you connect CS to. Used only when "USE_SPI" is defined

// Pines a añadir
#define SCL_PIN 6
#define SDA_PIN 7
#define REG_1V8_EN_PIN 8 //enable 1v8 register

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

void setup()
{

  SERIAL_PORT.begin(921600);
  while (!SERIAL_PORT)
  {
  };

  WIRE_PORT.setPins(SDA_PIN,SCL_PIN); 
  pinMode(REG_1V8_EN_PIN, OUTPUT);
  digitalWrite(REG_1V8_EN_PIN, 1);
  delay(100);
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  BMP388_DEV bmp388;   
  bmp388.begin(SLEEP_MODE, 0x76);

  // myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  
  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);

  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
  SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
  SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));
  
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("startupMagnetometer returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Configuration complete!"));
}

void loop()
{
  // High frequency: 400 Hz
  SERIAL_PORT.println(">>> Capturing data at ~400Hz");
  ICM_20948_smplrt_t mySmplrtHigh;
  mySmplrtHigh.g = 1;  
  mySmplrtHigh.a = 1;
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrtHigh);
  delay(1000);

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {  // 5 seconds
    if (myICM.dataReady()) {
      myICM.getAGMT();
      unsigned long actual = micros();
      printScaledAGMT(&myICM, actual);
    }
  }

  SERIAL_PORT.println(">>> Waiting 5 seconds before changing to ~20Hz");
  delay(5000);

  // Low frequency
  SERIAL_PORT.println(">>> Capturing data at ~20Hz");
  ICM_20948_smplrt_t mySmplrtLow;
  mySmplrtLow.g = 55;
  mySmplrtLow.a = 55;
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrtLow);
  delay(1000);

  startTime = millis();
  while (millis() - startTime < 5000) {  // 5 seconds
    if (myICM.dataReady()) {
      myICM.getAGMT();
      unsigned long actual = micros();
      printScaledAGMT(&myICM, actual);
    }
  }

  SERIAL_PORT.println(">>> Activating sleep mode");
  delay(100);
  myICM.resetMag();
  ICM_20948_Status_e stat = myICM.sleep(true);
  Serial.print(stat);
  delay(5000);
  myICM.sleep(false);
  myICM.startupMagnetometer();


  SERIAL_PORT.println(">>> Activating low power mode");
  myICM.lowPower(true);
  startTime = millis();
  while (millis() - startTime < 5000) {  // 5 seconds
    if (myICM.dataReady()) {
      myICM.getAGMT();
      unsigned long actual = micros();
      printScaledAGMT(&myICM, actual);
    }
  }
  myICM.lowPower(false);

  SERIAL_PORT.println(">>> Full cycle, restarting in 5 seconds...");
  delay(5000);
}

// Below here are some helper functions to print the data nicely!

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    break;
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor, unsigned long time)
{
  printFormattedFloat(time/1000000.0, 2, 6);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->accX(), 1, 5);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->accY(), 1, 5);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->accZ(), 1, 5);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->gyrX(), 3, 5);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->gyrY(), 3, 5);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->gyrZ(), 3, 5);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->magX(),3, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->magY(), 3, 2);
  SERIAL_PORT.print(",");
  printFormattedFloat(sensor->magZ(), 3, 2); 
  SERIAL_PORT.println();
} 
