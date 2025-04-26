/****************************************************************
 * Author: Lucía Alonso Mozo
 * Builds on Example2_Advanced.ino to set up interrupts when data is ready
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: June 5 2019
 ***************************************************************/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define INT_PIN 5 // Make sure to connect this pin on your uC to the "INT" pin on the ICM-20948 breakout
#define SCL_PIN 6 //6 in DAURIAN 
#define SDA_PIN 7 //7 in DAURIAN 
#define REG_1V8_EN_PIN 8 //enable 1v8 register

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

// Some vars to control or respond to interrupts
volatile bool isrFired = false;
volatile bool sensorSleep = false;
volatile bool canToggle = false;

void setup()
{
  pinMode(REG_1V8_EN_PIN, OUTPUT);
  digitalWrite(REG_1V8_EN_PIN, 1);
  pinMode(INT_PIN, INPUT_PULLUP);                                   // Using a pullup b/c ICM-20948 Breakout board has an onboard pullup as well and we don't want them to compete
  attachInterrupt(digitalPinToInterrupt(INT_PIN), icmISR, FALLING); // Set up a falling interrupt


  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT);
  WIRE_PORT.setPins(SDA_PIN, SCL_PIN); //SDA/SCL
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

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
  myICM.sleep(sensorSleep);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled);

  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 1;  
  mySmplrt.a = 1; 
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);


  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
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
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();

  // Now we're going to set up interrupts. There are a lot of options, but for this test we're just configuring the interrupt pin and enabling interrupts to tell us when new data is ready
  /*
    ICM_20948_Status_e  cfgIntActiveLow         ( bool active_low );
    ICM_20948_Status_e  cfgIntOpenDrain         ( bool open_drain );
    ICM_20948_Status_e  cfgIntLatch             ( bool latching );                          // If not latching then the interrupt is a 50 us pulse

    ICM_20948_Status_e  cfgIntAnyReadToClear    ( bool enabled );                           // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first

    ICM_20948_Status_e  cfgFsyncActiveLow       ( bool active_low );
    ICM_20948_Status_e  cfgFsyncIntMode         ( bool interrupt_mode );                    // Can ue FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

    ICM_20948_Status_e  intEnableI2C            ( bool enable );
    ICM_20948_Status_e  intEnableDMP            ( bool enable );
    ICM_20948_Status_e  intEnablePLL            ( bool enable );
    ICM_20948_Status_e  intEnableWOM            ( bool enable );
    ICM_20948_Status_e  intEnableWOF            ( bool enable );
    ICM_20948_Status_e  intEnableRawDataReady   ( bool enable );
    ICM_20948_Status_e  intEnableOverflowFIFO   ( uint8_t bm_enable );
    ICM_20948_Status_e  intEnableWatermarkFIFO  ( uint8_t bm_enable );
 */
  myICM.cfgIntActiveLow(true);  // Active low to be compatible with the breakout board's pullup resistor
  myICM.cfgIntOpenDrain(false); // Push-pull, though open-drain would also work thanks to the pull-up resistors on the breakout
  myICM.cfgIntLatch(true);      // Latch the interrupt until cleared

  myICM.intEnableRawDataReady(true); // enable interrupts on raw data ready


  //  // Note: weirdness with the Wake on Motion interrupt being always enabled.....
  //  uint8_t zero_0 = 0xFF;
  //  ICM_20948_execute_r( &myICM._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );
  //  SERIAL_PORT.print("INT_EN was: 0x"); SERIAL_PORT.println(zero_0, HEX);
  //  zero_0 = 0x00;
  //  ICM_20948_execute_w( &myICM._device, AGB0_REG_INT_ENABLE, (uint8_t*)&zero_0, sizeof(uint8_t) );
  // Frecuencia alta: 400 Hz (real ≈ 562.5 Hz)
  ICM_20948_smplrt_t mySmplrtHigh;
  mySmplrtHigh.g = 1;  // 1125 / (1 + 1) = 562.5 Hz
  mySmplrtHigh.a =1;
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrtHigh);
  delay(1000);

}
unsigned long now;

void loop()
{
  if (isrFired)
  { // If our isr flag is set then clear the interrupts on the ICM
    isrFired = false;
  }
}

void icmISR(void)
{
  now = micros();
  isrFired = true; // Can't use I2C within ISR on 328p, so just set a flag to know that data is available
}
