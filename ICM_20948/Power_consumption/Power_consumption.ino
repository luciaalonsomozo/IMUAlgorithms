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

//#define USE_SPI       // Uncomment this to use SPI

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
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

void setup()
{

  SERIAL_PORT.begin(921600);
  while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else

  WIRE_PORT.setPins(SDA_PIN,SCL_PIN); 
  pinMode(REG_1V8_EN_PIN, OUTPUT);
  digitalWrite(REG_1V8_EN_PIN, 1);
  delay(100);
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT, SPI_FREQ); // Here we are using the user-defined SPI_FREQ as the clock speed of the SPI bus
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    if (myICM.status != ICM_20948_Stat_Ok)
    {
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  
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

  // Choose whether or not to use DLPF
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);

}

void loop()
{

  // Frecuencia alta: 400 Hz (real ≈ 562.5 Hz)
  
  // SERIAL_PORT.println(">>> Capturando datos a ~400Hz");
  ICM_20948_smplrt_t mySmplrtHigh;
  mySmplrtHigh.g = 1;  // 1125 / (1 + 1) = 562.5 Hz
  mySmplrtHigh.a = 1;
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrtHigh);
  delay(10000);

  // SERIAL_PORT.println(">>> Espera antes de cambiar a ~20Hz");
  // delay(5000);

  // Frecuencia baja: 20 Hz (real ≈ 20.4 Hz)
  //SERIAL_PORT.println(">>> Capturando datos a ~20Hz");
  ICM_20948_smplrt_t mySmplrtLow;
  mySmplrtLow.g = 55;  // 1125 / (55 + 1) = 20.1 Hz
  mySmplrtLow.a = 55;
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrtLow);
  delay(10000);
  
  
  // SERIAL_PORT.println(">>> Activando modo dormido");
  myICM.sleep(true);
  delay(10000); // En este tiempo aunque le pidieramos al sensor, no nos da nada.
  myICM.sleep(false);

  // SERIAL_PORT.println(">>> Activando modo bajo consumo");
  // Modo de bajo consumo
  myICM.lowPower(true);
  delay(10000);
  myICM.lowPower(false);

}

