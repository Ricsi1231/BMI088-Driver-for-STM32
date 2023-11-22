#include "stm32h7xx_hal.h"

#ifndef BMI088_H
#define BMI088_H

//*ACCELOMETER REGISTERS
#define ACC_CHIP_ID (0x00)
#define ACC_ERR_REG (0x02)
#define ACC_STATUS (0x03)

#define ACC_X_LSB (0x12)
#define ACC_X_MSB (0x13)
#define ACC_Y_LSB (0x14)
#define ACC_Y_MSB (0x15)
#define ACC_Z_LSB (0x16)
#define ACC_Z_MSB (0x17)

#define SENSORTIME_0 (0x18)
#define SENSORTIME_1 (0x19)
#define SENSORTIME_2 (0x1A)

#define ACC_INT_STAT_1 (0x1D)
#define ACC_INT1_IO_CTRL (0x53)
#define ACC_INT2_IO_CTRL (0x54)
#define ACC_INT_MAP_DATA (0x58)

#define ACC_CONF (0x40)
#define ACC_RANGE (0x41)
#define ACC_SELF_TEST (0x6D)
#define ACC_PWR_CONF (0x7C)
#define ACC_PWR_CTRL (0x7D)
#define ACC_SOFTRESET (0x7E)
//*ACCELOMETER REGISTERS

//*ACCELOMETER CHIP ID VALUE
#define ACC_CHIP_ID_VALUE (0x0F)
//*ACCELOMETER CHIP ID VALUE

//*GYROSCOPE REGISTERS
#define GYRO_CHIP_ID (0x00)

#define GYRO_X_LSB (0x02)
#define GYRO_X_MSB (0x03)
#define GYRO_Y_LSB (0x04)
#define GYRO_Y_MSB (0x05)
#define GYRO_Z_LSB (0x06)
#define GYRO_Z_MSB (0x07)

#define GYRO_INT_STAT_1 (0x0A)
#define GYRO_INT_CTRL (0x15)
#define INT3_INT4_IO_CONF (0x16)
#define INT3_INT4_IO_MAP (0x18)

#define GYRO_RANGE (0x0A)
#define GYRO_BANDWIDTH (0x10)
#define GYRO_LPM1 (0x11)
#define GYRO_SOFTRESET (0x14)
#define GYRO_SELF_TEST (0x3C)
//*GYROSCOPE REGISTERS

//*GYROSCOPE CHIP ID VALUE
#define GYRO_CHIP_ID_VALUE (0x0D)
//*GYROSCOPE CHIP ID VALUE

//*GYROSCOPE INIT VALUES
#define GYRO_PWR_ON_INIT_VALUE (0x00) //! Turn on the sensor
#define GYRO_RANGE_INIT_VALUE (0x00) //! +-2000 (angular rate and resulotion)
#define GYRO_BANDWIDTH_INIT_VALUE (0x02) //! 1000 ODR (Hz), 116 Filter bandwidth (Hz)
#define GYRO_ENABLE_DATA_READY_INTERRUPT_VALUE (0x80) //! Enable data ready interrupt
#define GYRO_CONFIGURE_INTERRUPT (0x0F) //! active high, open drain
#define GYRO_CONFIGURE_INTERRUPT_DATA_READY (0x81) //! set 3-4 interrupt pin to be data ready interrupt pin
//*GYROSCOPE INIT VALUES

//*ACCELOMETER INIT VALUES
#define ACC_PWR_ON_INIT_VALUE (0x00) //! Turn on accelometer
#define ACC_ENABLE (0x04) //! Enable the accelometer
#define ACC_CONF_INIT_VALUE (0x0E) //! 800 ODR (Hz), Normal filter mode
#define ACC_RANGE_INIT_VALUE (0x03) //! +-24 G range
#define ACC_ENABLE_DATA_READY_INTERRUPT (0x0E) //! Enable data ready interrupt
#define ACC_CONFIGURE_INTERRUPT (0x22) //! set to data ready interrupt
//*ACCELOMETER INIT VALUES 

//*SENSOR MODES
#define GYRO_SUSPEND_VALUE (0x80)
#define GYRO_WAKEUP_VALUE (0x00)

#define ACC_SUSPEND_VALUE (0x03)
#define ACC_WAKEUP_VALUE (0x00)
//*SENSOR MODES

//* SPI settings
#define SPI_CLC_POLYNOIMAL_SIZE (10)
//* SPI settings

//*PART SELECT
#define GYROSCOPE (0)
#define ACCELOMETER (1)
//*PART SELECT

//*Logic levels
#define HIGH (GPIO_PIN_SET)
#define LOW (GPIO_PIN_RESET)
//*Logic levels

//* ERROR CODES
enum BMI088_errorCode 
{
    BMI088_spiInitError = 7,
    BMI088_bmiError = 6,
    BMI088_spiError = 5,
    BMI088_wrongValue = 4,
    BMI088_pinError = 3,
    BMI088_tempReadError = 2,
    BMI088_success = 1
};
//* ERROR CODES

class BMI088 
{
    public:
    //* Constructor for initializing the BMI088 sensor
    BMI088(SPI_HandleTypeDef *spi, SPI_TypeDef *spi_port, GPIO_TypeDef *acc_cs_port, uint8_t acc_cs_pin, GPIO_TypeDef *gyro_cs_port, uint8_t gyro_cs_pin, GPIO_TypeDef *int1_port, uint8_t int1_pin, IRQn_Type int1_irq_port, GPIO_TypeDef *int2_port, uint8_t int2_pin, IRQn_Type int2_irq_port);

    //* Set the configuration for the accelerometer
    //* Inputs: range - measurement range (g), bandwidth - output data rate (Hz)
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode setAccelometer(uint8_t range, uint16_t bandwidth);

    //* Read accelerometer data in standard mode
    //* Outputs: x, y, z - accelerometer readings in m/s^2
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode readAccelometer(int16_t &x, int16_t &y, int16_t &z);

    //* Read accelerometer data using DMA
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode readAccelometerDMA();

    //* Process completed DMA transfer and retrieve accelerometer data
    //* Outputs: x, y, z - accelerometer readings in m/s^2
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode readAccelometerDMAComplete(int16_t &x, int16_t &y, int16_t &z);

    //* Set the configuration for the gyroscope
    //* Inputs: range - angular rate range and resolution, bandwidth - output data rate (Hz)
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode setGyroscope(uint8_t range, uint16_t bandwidth);

    //* Read gyroscope data in standard mode
    //* Outputs: x, y, z - gyroscope readings in rad/s
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode readGyroscope(int16_t &x, int16_t &y, int16_t &z);

    //* Read gyroscope data using DMA
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode readGyroscopeDMA();

    //* Process completed DMA transfer and retrieve gyroscope data
    //* Outputs: x, y, z - gyroscope readings in rad/s
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode readGyroscopeDMAComplete(int16_t &x, int16_t &y, int16_t &z);

    //* Send the sensor to sleep mode
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode sleepSensor();

    //* Wake up the sensor from sleep mode
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode wakeupSensor();

    private:
    SPI_HandleTypeDef *SPI;
    SPI_TypeDef *SPI_PORT;

    GPIO_TypeDef *ACC_CS_PORT;
    GPIO_TypeDef *GYRO_CS_PORT;
    GPIO_TypeDef *INT1_PORT;
    GPIO_TypeDef *INT2_PORT;

    uint8_t ACC_CS_PIN;
    uint8_t GYRO_CS_PIN;
    uint8_t INT1_PIN;
    uint8_t INT2_PIN;

    uint8_t accDmaData[6];
    uint8_t gyroDmaData[6];

    uint8_t initStatus; 
    uint8_t bmiStatus;

    uint8_t accScaleFactor;
    uint8_t gyroScaleFactor;

    private:
    //* Write data to a register on the specified sensor part
    //* Inputs: part - GYROSCOPE or ACCELOMETER, address - register address, data - data to write
    //* Returns: Error code indicating success or error
    uint8_t writeRegister(uint8_t part, uint8_t address, uint8_t data);

    //* Read data from a register on the specified sensor part
    //* Inputs: part - GYROSCOPE or ACCELOMETER, address - register address
    //* Returns: Read data or error code indicating error
    uint8_t readRegister(uint8_t part, uint8_t address);

    //* Set the chip select pin for the specified sensor part
    //* Inputs: part - GYROSCOPE or ACCELOMETER, state - GPIO pin state (HIGH or LOW)
    void setCsPin(uint8_t part, GPIO_PinState state);

    private:
    //* Initialize the SPI interface for communication with the sensor
    //* Inputs: spi - SPI handle, spi_port - SPI peripheral
    //* Returns: BMI088_errorCode indicating success or error
    BMI088_errorCode spiInit(SPI_HandleTypeDef *spi, SPI_TypeDef *spi_port);

    //* Initialize GPIO pins for sensor control and interrupts
    //* Inputs: acc_cs_port - accelerometer chip select port, acc_cs_pin - accelerometer chip select pin, gyro_cs_pin - gyroscope chip select port
    //* Inputs: int1_port - accelometer int1 interrupt port, int2_pin - accelometer int1 interrupt pin, int2_port - gyroscope int2 interrupt port, int2_pin - gyroscope int1 interrupt pin  
    void gpioInit(GPIO_TypeDef *acc_cs_port, uint8_t acc_cs_pin, GPIO_TypeDef *gyro_cs_port, uint8_t gyro_cs_pin, GPIO_TypeDef *int1_port, uint8_t int1_pin, IRQn_Type int1_irq_port, GPIO_TypeDef *int2_port, uint8_t int2_pin, IRQn_Type int2_irq_port);
};

#endif
