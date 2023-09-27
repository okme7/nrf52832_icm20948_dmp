#ifndef _ICM_20948_H_
#define _ICM_20948_H_

#include "ICM_20948_C.h" // The C backbone. ICM_20948_USE_DMP is defined in here.
#include "AK09916_REGISTERS.h"

//class ICM_20948
//private
//const uint8_t MAX_MAGNETOMETER_STARTS = 10; // This replaces maxTries

//protected
//ICM_20948_Device_t _device;

float ICM_20948_getTempC(int16_t val);
float ICM_20948_getGyrDPS(int16_t axis_val);
float ICM_20948_getAccMG(int16_t axis_val);
float ICM_20948_getMagUT(int16_t axis_val);


//public
extern ICM_20948_AGMT_t agmt;          // Acceleometer, Gyroscope, Magenetometer, and Temperature data
ICM_20948_AGMT_t ICM_20948_getAGMT(void); // Updates the agmt field in the object and also returns a copy directly

float ICM_20948_magX(void); // micro teslas
float ICM_20948_magY(void); // micro teslas
float ICM_20948_magZ(void); // micro teslas

float ICM_20948_accX(void); // milli g's
float ICM_20948_accY(void); // milli g's
float ICM_20948_accZ(void); // milli g's

float ICM_20948_gyrX(void); // degrees per second
float ICM_20948_gyrY(void); // degrees per second
float ICM_20948_gyrZ(void); // degrees per second

float ICM_20948_temp(void); // degrees celsius

extern ICM_20948_Status_e status;                                              // Status from latest operation
const char *ICM_20948_statusString(ICM_20948_Status_e stat); // Returns a human-readable status message. Defaults to status member, but prints string for supplied status if supplied

// Device Level
ICM_20948_Status_e ICM_20948_setBank(uint8_t bank);                                // Sets the bank
ICM_20948_Status_e ICM_20948_swReset(void);                                        // Performs a SW reset
ICM_20948_Status_e ICM_20948_sleep_(bool on);                               // Set sleep mode for the chip
ICM_20948_Status_e ICM_20948_lowPower(bool on);                             // Set low power mode for the chip
ICM_20948_Status_e ICM_20948_setClockSource(ICM_20948_PWR_MGMT_1_CLKSEL_e source); // Choose clock source
ICM_20948_Status_e ICM_20948_checkID(void);                                        // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI

bool ICM_20948_dataReady(void);    // Returns 'true' if data is ready
uint8_t ICM_20948_getWhoAmI(void); // Return whoami in out prarmeter
bool ICM_20948_isConnected(void);  // Returns true if communications with the device are sucessful

// Internal Sensor Options
ICM_20948_Status_e ICM_20948_setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
ICM_20948_Status_e ICM_20948_setFullScale(uint8_t sensor_id_bm, ICM_20948_fss_t fss);
ICM_20948_Status_e ICM_20948_setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg);
ICM_20948_Status_e ICM_20948_enableDLPF(uint8_t sensor_id_bm, bool enable);
ICM_20948_Status_e ICM_20948_setSampleRate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt);

// Interrupts on INT and FSYNC Pins
ICM_20948_Status_e ICM_20948_clearInterrupts(void);

ICM_20948_Status_e ICM_20948_cfgIntActiveLow(bool active_low);
ICM_20948_Status_e ICM_20948_cfgIntOpenDrain(bool open_drain);
ICM_20948_Status_e ICM_20948_cfgIntLatch(bool latching);         // If not latching then the interrupt is a 50 us pulse
ICM_20948_Status_e ICM_20948_cfgIntAnyReadToClear(bool enabled); // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first
ICM_20948_Status_e ICM_20948_cfgFsyncActiveLow(bool active_low);
ICM_20948_Status_e ICM_20948_cfgFsyncIntMode(bool interrupt_mode); // Can use FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

ICM_20948_Status_e ICM_20948_intEnableI2C(bool enable);
ICM_20948_Status_e ICM_20948_intEnableDMP(bool enable);
ICM_20948_Status_e ICM_20948_intEnablePLL(bool enable);
ICM_20948_Status_e ICM_20948_intEnableWOM(bool enable);
ICM_20948_Status_e ICM_20948_intEnableWOF(bool enable);
ICM_20948_Status_e ICM_20948_intEnableRawDataReady(bool enable);
ICM_20948_Status_e ICM_20948_intEnableOverflowFIFO(uint8_t bm_enable);
ICM_20948_Status_e ICM_20948_intEnableWatermarkFIFO(uint8_t bm_enable);

ICM_20948_Status_e ICM_20948_WOMLogic(uint8_t enable, uint8_t mode);
ICM_20948_Status_e ICM_20948_WOMThreshold(uint8_t threshold);

// Interface Options
ICM_20948_Status_e ICM_20948_i2cMasterPassthrough(bool passthrough);
ICM_20948_Status_e ICM_20948_i2cMasterEnable(bool enable);
ICM_20948_Status_e ICM_20948_i2cMasterReset();

//Used for configuring peripherals 0-3
ICM_20948_Status_e ICM_20948_i2cControllerConfigurePeripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut);
ICM_20948_Status_e ICM_20948_i2cControllerPeriph4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);

//Provided for backward-compatibility only. Please update to i2cControllerConfigurePeripheral and i2cControllerPeriph4Transaction.
//https://www.oshwa.org/2020/06/29/a-resolution-to-redefine-spi-pin-names/
ICM_20948_Status_e ICM_20948_i2cMasterConfigureSlave(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap);
ICM_20948_Status_e ICM_20948_i2cMasterSLV4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);

//Used for configuring the Magnetometer
ICM_20948_Status_e ICM_20948_i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t ICM_20948_i2cMasterSingleR(uint8_t addr, uint8_t reg);

// Default Setup
ICM_20948_Status_e ICM_20948_startupDefault(bool minimal); // If minimal is true, several startup steps are skipped. If ICM_20948_USE_DMP is defined, .begin will call startupDefault with minimal set to true.

// direct read/write
ICM_20948_Status_e ICM_20948_read(uint8_t reg, uint8_t *pdata, uint32_t len);
ICM_20948_Status_e ICM_20948_write(uint8_t reg, uint8_t *pdata, uint32_t len);

//Mag specific
ICM_20948_Status_e ICM_20948_startupMagnetometer(bool minimal); // If minimal is true, several startup steps are skipped. The mag then needs to be set up manually for the DMP.
ICM_20948_Status_e ICM_20948_magWhoIAm(void);
uint8_t ICM_20948_readMag(AK09916_Reg_Addr_e reg);
ICM_20948_Status_e ICM_20948_writeMag(AK09916_Reg_Addr_e reg, uint8_t *pdata);
ICM_20948_Status_e ICM_20948_resetMag();

//FIFO
ICM_20948_Status_e ICM_20948_enableFIFO(bool enable);
ICM_20948_Status_e ICM_20948_resetFIFO(void);
ICM_20948_Status_e ICM_20948_setFIFOmode(bool snapshot); // Default to Stream (non-Snapshot) mode
ICM_20948_Status_e ICM_20948_getFIFOcount(uint16_t *count);
ICM_20948_Status_e ICM_20948_readFIFO(uint8_t *data, uint8_t len);

//DMP

//Gyro Bias
ICM_20948_Status_e ICM_20948_setBiasGyroX(int32_t newValue);
ICM_20948_Status_e ICM_20948_setBiasGyroY(int32_t newValue);
ICM_20948_Status_e ICM_20948_setBiasGyroZ(int32_t newValue);
ICM_20948_Status_e ICM_20948_getBiasGyroX(int32_t* bias);
ICM_20948_Status_e ICM_20948_getBiasGyroY(int32_t* bias);
ICM_20948_Status_e ICM_20948_getBiasGyroZ(int32_t* bias);
//Accel Bias
ICM_20948_Status_e ICM_20948_setBiasAccelX(int32_t newValue);
ICM_20948_Status_e ICM_20948_setBiasAccelY(int32_t newValue);
ICM_20948_Status_e ICM_20948_setBiasAccelZ(int32_t newValue);
ICM_20948_Status_e ICM_20948_getBiasAccelX(int32_t* bias);
ICM_20948_Status_e ICM_20948_getBiasAccelY(int32_t* bias);
ICM_20948_Status_e ICM_20948_getBiasAccelZ(int32_t* bias);
//CPass Bias
ICM_20948_Status_e ICM_20948_setBiasCPassX(int32_t newValue);
ICM_20948_Status_e ICM_20948_setBiasCPassY(int32_t newValue);
ICM_20948_Status_e ICM_20948_setBiasCPassZ(int32_t newValue);
ICM_20948_Status_e ICM_20948_getBiasCPassX(int32_t* bias);
ICM_20948_Status_e ICM_20948_getBiasCPassY(int32_t* bias);
ICM_20948_Status_e ICM_20948_getBiasCPassZ(int32_t* bias);

// Done:
//  Configure DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
//  Load Firmware
//  Configure Accel scaling to DMP
//  Configure Compass mount matrix and scale to DMP
//  Reset FIFO
//  Reset DMP
//  Enable DMP interrupt
//  Configuring DMP to output data to FIFO: set DATA_OUT_CTL1, DATA_OUT_CTL2, DATA_INTR_CTL and MOTION_EVENT_CTL
//  Configuring DMP to output data at multiple ODRs
//  Configure DATA_RDY_STATUS
//  Configuring Accel calibration
//  Configuring Compass calibration
//  Configuring Gyro gain
//  Configuring Accel gain
//  Configure I2C_SLV0 and I2C_SLV1 to: request mag data from the hidden reserved AK09916 registers; trigger Single Measurements
//  Configure I2C Master ODR (default to 68.75Hz)

// To Do:
//  Additional FIFO output control: FIFO_WATERMARK, BM_BATCH_MASK, BM_BATCH_CNTR, BM_BATCH_THLD
//  Configuring DMP features: PED_STD_STEPCTR, PED_STD_TIMECTR
//  Enabling Activity Recognition (BAC) feature
//  Enabling Significant Motion Detect (SMD) feature
//  Enabling Tilt Detector feature
//  Enabling Pick Up Gesture feature
//  Enabling Fsync detection feature
//  Biases: add save and load methods

ICM_20948_Status_e ICM_20948_enableDMP(bool enable);
ICM_20948_Status_e ICM_20948_resetDMP(void);
ICM_20948_Status_e ICM_20948_loadDMPFirmware(void);
ICM_20948_Status_e ICM_20948_setDMPstartAddress(unsigned short address);
ICM_20948_Status_e ICM_20948_enableDMPSensor(enum inv_icm20948_sensor sensor, bool enable);
ICM_20948_Status_e ICM_20948_enableDMPSensorInt(enum inv_icm20948_sensor sensor, bool enable);
ICM_20948_Status_e ICM_20948_writeDMPmems(unsigned short reg, unsigned int length, const unsigned char *data);
ICM_20948_Status_e ICM_20948_readDMPmems(unsigned short reg, unsigned int length, unsigned char *data);
ICM_20948_Status_e ICM_20948_setDMPODRrate(enum DMP_ODR_Registers odr_reg, int interval);
ICM_20948_Status_e ICM_20948_readDMPdataFromFIFO(icm_20948_DMP_data_t *data);
ICM_20948_Status_e ICM_20948_setGyroSF(unsigned char div, int gyro_level);
ICM_20948_Status_e ICM_20948_initializeDMP(void); // Combine all of the DMP start-up code in one place. Can be overwritten if required

//I2C
//ICM_20948_Serif_t I2C_serif;
//ICM_20948_Serif_t SPI_serif;



ICM_20948_Status_e ICM_20948_I2C_begin();
ICM_20948_Status_e ICM_20948_SPI_begin();

#endif /* _ICM_20948_H_ */
