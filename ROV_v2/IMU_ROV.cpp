#include "IMU_ROV.h"
#include <Wire.h>

// Set initial input parameters
enum Ascale {  // ACC Full Scale
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_18G
};

enum Abw { // ACC Bandwidth
  ABW_7_81Hz = 0,
  ABW_15_63Hz,
  ABW_31_25Hz,
  ABW_62_5Hz,
  ABW_125Hz,
  ABW_250Hz,
  ABW_500Hz,
  ABW_1000Hz,    //0x07
};

enum APwrMode { // ACC Pwr Mode
  NormalA = 0,
  SuspendA,
  LowPower1A,
  StandbyA,
  LowPower2A,
  DeepSuspendA
};

enum Gscale {  // gyro full scale
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS    // 0x04
};

enum GPwrMode { // GYR Pwr Mode
  NormalG = 0,
  FastPowerUpG,
  DeepSuspendedG,
  SuspendG,
  AdvancedPowerSaveG
};

enum Gbw { // gyro bandwidth
  GBW_523Hz = 0,
  GBW_230Hz,
  GBW_116Hz,
  GBW_47Hz,
  GBW_23Hz,
  GBW_12Hz,
  GBW_64Hz,
  GBW_32Hz
};

enum OPRMode {  // BNO-55 operation modes
  CONFIGMODE = 0x00,
// Sensor Mode
  ACCONLY,
  MAGONLY,
  GYROONLY,
  ACCMAG,
  ACCGYRO,
  MAGGYRO,
  AMG,            // 0x07
// Fusion Mode
  IMU,
  COMPASS,
  M4G,
  NDOF_FMC_OFF,
  NDOF            // 0x0C
};

enum PWRMode {
  Normalpwr = 0,
  Lowpower,
  Suspendpwr
};

enum Modr {         // magnetometer output data rate
  MODR_2Hz = 0,
  MODR_6Hz,
  MODR_8Hz,
  MODR_10Hz,
  MODR_15Hz,
  MODR_20Hz,
  MODR_25Hz,
  MODR_30Hz
};

enum MOpMode { // MAG Op Mode
  LowPower = 0,
  Regular,
  EnhancedRegular,
  HighAccuracy
};

enum MPwrMode { // MAG power mode
  Normal = 0,
  Sleep,
  Suspend,
  ForceMode
};

static uint8_t PWRMode = Normal ;    // Select BNO055 power mode
static uint8_t OPRMode = NDOF;        // specify operation mode for sensors

static uint8_t status;               // BNO055 data status register
static int16_t EulCount[3];    // Stores the 16-bit signed Euler angle output
static float bPitch, bYaw, bRoll;
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
static float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

// I2C read/write functions for the BNO055 sensor

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
//	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom((int)address, 1);  // Read one byte from slave register address
	Wire.requestFrom((int)address, (size_t) 1);   // Read one byte from slave register address
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
//	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
        unsigned long timeout = 0;
	while ((i<count) && (timeout<10000)){
          timeout++;
          if (Wire.available()) {
            dest[i++] = Wire.read();
           }         // Put read results in the Rx buffer
        }
        if (i<count)
          Serial.println("DIAG:Timeout reading I2c;");
}



//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;      // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

int8_t readGyroTempData()
{
  return readByte(BNO055_ADDRESS, BNO055_TEMP);  // Read the two raw data registers sequentially into data array
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readQuatData(int16_t * destination)
{
  uint8_t rawData[8];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_QUA_DATA_W_LSB, 8, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;
}

void readEulData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_EUL_HEADING_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readLIAData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readGRVData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_GRV_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void initBNO055() {
  byte mode = readByte(BNO055_ADDRESS, BNO055_OPR_MODE);
  Serial.print("BNO055.Initial_Mode:"); Serial.print(mode); Serial.println(';');


  while (mode != CONFIGMODE) {
    // Select BNO055 config mode
     writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
     delay(100);
     mode = readByte(BNO055_ADDRESS, BNO055_OPR_MODE);
   }

   // Select page 0 to read sensors
   writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);

   //Select BNO055 orientation

     /*  Portait, wires forward, sensors up  */
   writeByte(BNO055_ADDRESS, BNO055_AXIS_MAP_CONFIG, 0x24 );//P1
   writeByte(BNO055_ADDRESS, BNO055_AXIS_MAP_SIGN, 0x00 );//P1
//   writeByte(BNO055_ADDRESS, BNO055_AXIS_MAP_CONFIG, 0x21 );//P0
//   writeByte(BNO055_ADDRESS, BNO055_AXIS_MAP_SIGN, 0x04 );//P0

   // Select BNO055 gyro temperature source
//   writeByte(BNO055_ADDRESS, BNO055_TEMP_SOURCE, 0x01 );

   // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
   writeByte(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01 );

   // Select BNO055 system power mode
   writeByte(BNO055_ADDRESS, BNO055_PWR_MODE, PWRMode );

   // Select BNO055 system operation mode
  while (mode != OPRMode) {
    // Select BNO055 config mode
     writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );
     delay(100);
     mode = readByte(BNO055_ADDRESS, BNO055_OPR_MODE);
   }

}

void IMU_setup()
{
  Wire.begin();
  Wire.beginTransmission(BNO055_ADDRESS);
  if (Wire.endTransmission() != 0)
    return; //Cannot find I2c device, abort setup
    
    initBNO055(); // Initialize the BNO055
}
