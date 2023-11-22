#include "BMI088.h"
#include "bitManipulation.h"

void BMI088::setCsPin(uint8_t part, GPIO_PinState state) 
{
    if(part == GYROSCOPE) 
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, state);
    }
    else 
    {
        HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, state);
    }
}

uint8_t BMI088::writeRegister(uint8_t part, uint8_t address, uint8_t data) 
{
    uint8_t dataSize = 2;
    uint8_t bitPlace = 1;
    uint8_t errorState = 0;
    uint8_t txData[dataSize] = {address, data};

    BIT_CLEAR(txData[0], bitPlace);

    if(part == GYROSCOPE) 
    {
        setCsPin(GYROSCOPE, LOW);
        errorState += HAL_SPI_Transmit(SPI, txData, dataSize, HAL_MAX_DELAY);
        setCsPin(GYROSCOPE, HIGH);

        return errorState;
    }
    else 
    {
        setCsPin(ACCELOMETER, LOW);
        errorState += HAL_SPI_Transmit(SPI, txData, dataSize, HAL_MAX_DELAY);
        setCsPin(ACCELOMETER, HIGH);

        return errorState;
    }
}

uint8_t BMI088::readRegister(uint8_t part, uint8_t address) 
{
    uint8_t dataSize = 2;
    uint8_t bitPlace = 1;
    uint8_t errorState = 0;
    uint8_t txData[dataSize] = {address, 0x00};
    uint8_t rxData[dataSize];

    BIT_SET(txData[0], bitPlace);

    if(part == GYROSCOPE) 
    {
        setCsPin(GYROSCOPE, LOW);
        errorState += HAL_SPI_TransmitReceive(SPI, txData, rxData, dataSize, HAL_MAX_DELAY);
        setCsPin(GYROSCOPE, HIGH);

        if(errorState != 1) 
        {
            return rxData[0];
        }

        return errorState;
    }
    else 
    {
        setCsPin(ACCELOMETER, LOW);
        errorState += HAL_SPI_TransmitReceive(SPI, txData, rxData, dataSize, HAL_MAX_DELAY);
        setCsPin(ACCELOMETER, HIGH);

        if(errorState != 1) 
        {
            return rxData[0];
        }

        return errorState;
    }
}

BMI088_errorCode BMI088::spiInit(SPI_HandleTypeDef *spi, SPI_TypeDef *spi_port) 
{
    this->SPI = spi;
    this->SPI_PORT = spi_port;

    SPI->Instance = SPI_PORT;
    SPI->Init.Mode = SPI_MODE_MASTER;
    SPI->Init.Direction = SPI_DIRECTION_2LINES;
    SPI->Init.DataSize = SPI_DATASIZE_8BIT;
    SPI->Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI->Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI->Init.NSS = SPI_NSS_SOFT;
    SPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    SPI->Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI->Init.TIMode = SPI_TIMODE_DISABLE;
    SPI->Init.CRCPolynomial = SPI_CLC_POLYNOIMAL_SIZE;
    SPI->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    if(HAL_SPI_Init(SPI) != HAL_OK) 
    {
        return BMI088_errorCode::BMI088_spiError;
    }

    return BMI088_errorCode::BMI088_success;
}

void BMI088::gpioInit(GPIO_TypeDef *acc_cs_port, uint8_t acc_cs_pin, GPIO_TypeDef *gyro_cs_port, uint8_t gyro_cs_pin, GPIO_TypeDef *int1_port, uint8_t int1_pin, IRQn_Type int1_irq_port, GPIO_TypeDef *int2_port, uint8_t int2_pin, IRQn_Type int2_irq_port) 
{
    this->ACC_CS_PORT = acc_cs_port;
    this->GYRO_CS_PORT = gyro_cs_port;
    this->INT1_PORT = int1_port;
    this->INT2_PORT = int2_port;

    this->ACC_CS_PIN = acc_cs_pin;
    this->GYRO_CS_PIN = gyro_cs_pin;
    this->INT1_PIN = int1_pin;
    this->INT2_PIN = int2_pin;

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = ACC_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ACC_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GYRO_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GYRO_CS_PORT, &GPIO_InitStruct);

    GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = INT1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(INT1_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(int1_irq_port, 0, 0);
    HAL_NVIC_EnableIRQ(int1_irq_port);

    GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = INT2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(INT2_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(int2_irq_port, 0, 0);
    HAL_NVIC_EnableIRQ(int2_irq_port);
}

BMI088::BMI088(SPI_HandleTypeDef *spi, SPI_TypeDef *spi_port, GPIO_TypeDef *acc_cs_port, uint8_t acc_cs_pin, GPIO_TypeDef *gyro_cs_port, uint8_t gyro_cs_pin, GPIO_TypeDef *int1_port, uint8_t int1_pin, IRQn_Type int1_irq_port, GPIO_TypeDef *int2_port, uint8_t int2_pin, IRQn_Type int2_irq_port)
{
    uint8_t chipID = 0;

    initStatus = spiInit(spi, spi_port);
    gpioInit(acc_cs_port, acc_cs_pin, gyro_cs_port, gyro_cs_pin, int1_port, int1_pin, int1_irq_port, int2_port, int2_pin, int2_irq_port);

    if(initStatus == BMI088_errorCode::BMI088_success) 
    {
        chipID = readRegister(GYROSCOPE, GYRO_CHIP_ID);

        if(chipID == GYRO_CHIP_ID_VALUE) 
        {
            bmiStatus += writeRegister(GYROSCOPE, GYRO_LPM1, GYRO_PWR_ON_INIT_VALUE);
            bmiStatus += writeRegister(GYROSCOPE, GYRO_RANGE, GYRO_RANGE_INIT_VALUE);
            bmiStatus += writeRegister(GYROSCOPE, GYRO_BANDWIDTH, GYRO_BANDWIDTH_INIT_VALUE);
            bmiStatus += writeRegister(GYROSCOPE, GYRO_INT_CTRL, GYRO_ENABLE_DATA_READY_INTERRUPT_VALUE);
            bmiStatus += writeRegister(GYROSCOPE, INT3_INT4_IO_CONF, GYRO_CONFIGURE_INTERRUPT);
            bmiStatus += writeRegister(GYROSCOPE, INT3_INT4_IO_MAP, GYRO_CONFIGURE_INTERRUPT_DATA_READY);
        }
        else
        {
            bmiStatus = BMI088_errorCode::BMI088_bmiError;
        }

        chipID = readRegister(ACCELOMETER, ACC_CHIP_ID);

        if(chipID == ACC_CHIP_ID_VALUE) 
        {
            bmiStatus += writeRegister(ACCELOMETER, ACC_PWR_CONF, ACC_PWR_ON_INIT_VALUE);
            bmiStatus += writeRegister(ACCELOMETER, ACC_PWR_CTRL, ACC_ENABLE);
            bmiStatus += writeRegister(ACCELOMETER, ACC_CONF, ACC_CONF_INIT_VALUE);
            bmiStatus += writeRegister(ACCELOMETER, ACC_RANGE, ACC_RANGE_INIT_VALUE);
            bmiStatus += writeRegister(ACCELOMETER, ACC_INT1_IO_CTRL, ACC_ENABLE_DATA_READY_INTERRUPT);
            bmiStatus += writeRegister(ACCELOMETER, ACC_INT2_IO_CTRL, ACC_ENABLE_DATA_READY_INTERRUPT);
            bmiStatus += writeRegister(ACCELOMETER, ACC_INT_MAP_DATA, ACC_CONFIGURE_INTERRUPT);
        }
        else 
        {
            bmiStatus = BMI088_errorCode::BMI088_bmiError;
        }

        if(bmiStatus != HAL_OK) 
        {
            bmiStatus = BMI088_errorCode::BMI088_bmiError;
        }
        else 
        {
            bmiStatus = BMI088_errorCode::BMI088_success;
        }
    }
}

BMI088_errorCode BMI088::sleepSensor() 
{
    uint8_t errorState = 0;

    if(bmiStatus == BMI088_errorCode::BMI088_success)
    {
    	errorState += writeRegister(ACCELOMETER, ACC_PWR_CTRL, ACC_SUSPEND_VALUE);
    	errorState += writeRegister(GYROSCOPE, GYRO_LPM1, GYRO_SUSPEND_VALUE);

    	if (errorState == HAL_OK)
    	{
    		bmiStatus = BMI088_errorCode::BMI088_success;
    	    return BMI088_errorCode::BMI088_success;
    	}
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;
    return BMI088_errorCode::BMI088_bmiError;
}

BMI088_errorCode BMI088::wakeupSensor() 
{
    uint8_t errorState = 0;

    if(bmiStatus == BMI088_errorCode::BMI088_success)
    {
    	errorState += writeRegister(ACCELOMETER, ACC_PWR_CTRL, ACC_WAKEUP_VALUE);
    	errorState += writeRegister(GYROSCOPE, GYRO_LPM1, GYRO_WAKEUP_VALUE);

    	if (errorState == HAL_OK)
    	{
    		bmiStatus = BMI088_errorCode::BMI088_success;
    	    return BMI088_errorCode::BMI088_success;
    	}
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;
    return BMI088_errorCode::BMI088_bmiError;
}

BMI088_errorCode BMI088::setAccelometer(uint8_t range, uint16_t bandwidth) 
{
    if(bmiStatus == BMI088_errorCode::BMI088_success) 
    {
        uint8_t rangeValuesLenght = 4;
    	uint8_t rangeValues[rangeValuesLenght] = {3, 6, 12, 24};
        uint8_t rangeHexValues[rangeValuesLenght] = {0x00, 0x01, 0x02, 0x03};

        uint8_t bandWidhtValuesLenght = 8;
        uint16_t bandwidthValues[bandWidhtValuesLenght] = {12, 25, 50, 100, 200, 400, 800, 1600};
        uint8_t bandwidthHexValues[bandWidhtValuesLenght] = {0x05, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};
        
        uint8_t errorState = 0;
        bool accRangeGoodValue = false;
        bool accbandwidthGoodValue = false;

        for(uint8_t i = 0; i < rangeValuesLenght; i++)
        {
            if(range == rangeValues[i]) 
            {
                errorState += writeRegister(ACCELOMETER, ACC_RANGE, rangeHexValues[i]);
                accRangeGoodValue = true;
            }
        }

        for(uint8_t i = 0; i < bandwidthHexValues; i++)
        {
        	if(bandwidth == bandwidthValues[i])
        	{
        	    errorState += writeRegister(ACCELOMETER, ACC_CONF, bandwidthHexValues[i]);
        	    accbandwidthGoodValue = true;
        	}
        }

        if(!accRangeGoodValue || !accbandwidthGoodValue) 
        {
        	bmiStatus = BMI088_errorCode::BMI088_wrongValue;
            return BMI088_errorCode::BMI088_wrongValue;
        }

        if(errorState == 1) 
        {
        	bmiStatus = BMI088_errorCode::BMI088_spiError;
            return BMI088_errorCode::BMI088_spiError;
        }

        bmiStatus = BMI088_errorCode::BMI088_success;
        return BMI088_errorCode::BMI088_success;
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;
    return BMI088_errorCode::BMI088_bmiError;
}

BMI088_errorCode BMI088::readAccelometer(int16_t &x, int16_t &y, int16_t &z) 
{
    if(bmiStatus == BMI088_errorCode::BMI088_success) 
    {
        uint8_t dataSize = 6;
        uint8_t hexValues[dataSize] = {ACC_X_LSB, ACC_X_MSB, ACC_Y_LSB, ACC_Y_MSB, ACC_Z_LSB, ACC_Z_MSB};
        uint8_t dataValues[dataSize];

        for(uint8_t i = 0; i < dataSize; i++) 
        {
            dataValues[i] = readRegister(ACCELOMETER, hexValues[i]);
        }

        x = (int16_t)((dataValues[0] << 8) | dataValues[1]) * accScaleFactor;
        y = (int16_t)((dataValues[2] << 8) | dataValues[3]) * accScaleFactor;
        z = (int16_t)((dataValues[4] << 8) | dataValues[5]) * accScaleFactor;

        bmiStatus = BMI088_errorCode::BMI088_success;
        return BMI088_errorCode::BMI088_success;
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;
    return BMI088_errorCode::BMI088_bmiError;
}

BMI088_errorCode BMI088::readAccelometerDMA() 
{
    if(bmiStatus == BMI088_errorCode::BMI088_success) 
    {
        uint8_t errorState = 0;
        uint8_t dataSize = 6;
        uint8_t txData[dataSize];

        txData[0] = ACC_X_LSB | 0x80;
        
        setCsPin(ACCELOMETER, LOW);
        errorState += HAL_SPI_TransmitReceive_DMA(SPI, txData, accDmaData, dataSize);
        setCsPin(ACCELOMETER, HIGH);

        if(errorState != HAL_OK)
        {
        	bmiStatus = BMI088_errorCode::BMI088_spiError;
            return BMI088_errorCode::BMI088_spiError;
        }

        bmiStatus = BMI088_errorCode::BMI088_success;
        return BMI088_errorCode::BMI088_success;
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;
    return BMI088_errorCode::BMI088_bmiError;
}

BMI088_errorCode BMI088::readAccelometerDMAComplete(int16_t &x, int16_t &y, int16_t &z) 
{
    if(bmiStatus == BMI088_errorCode::BMI088_success)
    {
    	x = (int16_t)((accDmaData[0] << 8) | accDmaData[1]) * accScaleFactor;
    	y = (int16_t)((accDmaData[2] << 8) | accDmaData[3]) * accScaleFactor;
    	z = (int16_t)((accDmaData[4] << 8) | accDmaData[5]) * accScaleFactor;

    	bmiStatus = BMI088_errorCode::BMI088_success;
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;
    return BMI088_errorCode::BMI088_success;
}

BMI088_errorCode BMI088::setGyroscope(uint8_t range, uint16_t bandwidth) 
{
    if(bmiStatus == BMI088_errorCode::BMI088_success) 
    {
    	uint8_t rangeValuesLenght = 5;
        uint16_t rangeValues[rangeValuesLenght] = {125, 250, 500, 1000, 2000};
        uint8_t rangeHexValues[rangeValuesLenght] = {0x00, 0x01, 0x02, 0x03, 0x04};

        uint8_t bandWidhtValuesLenght = 8;
        uint16_t bandwidthValues[bandWidhtValuesLenght] = {100, 200, 100, 200, 400, 1000, 2000, 2000};
        uint8_t bandwidthHexValues[bandWidhtValuesLenght] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
        
        uint8_t errorState = 0;
        bool gyroRangeGoodValue = false;
        bool gyrobandwidthGoodValue = false;

        for(int i = 0; i < rangeValuesLenght; i++)
        {
            if(range == rangeValues[i]) 
            {
                errorState += writeRegister(GYROSCOPE, GYRO_RANGE, rangeHexValues[i]);
                gyroRangeGoodValue = true;

                break;
            }
        }
        for(int i = 0; i < bandWidhtValuesLenght; i++)
        {
            if(bandwidth == bandwidthValues[i]) 
            {
                errorState += writeRegister(GYROSCOPE, GYRO_BANDWIDTH, bandwidthHexValues[i]);
                gyrobandwidthGoodValue = true;

                break;
            }
        }

        if(!gyroRangeGoodValue || !gyrobandwidthGoodValue) 
        {
        	bmiStatus = BMI088_errorCode::BMI088_wrongValue;
            return BMI088_errorCode::BMI088_wrongValue;
        }

        if(errorState == 1) 
        {
        	bmiStatus = BMI088_errorCode::BMI088_spiError;
            return BMI088_errorCode::BMI088_spiError;
        }

        bmiStatus = BMI088_errorCode::BMI088_success;
        return BMI088_errorCode::BMI088_success;
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;
    return BMI088_errorCode::BMI088_bmiError;
}

BMI088_errorCode BMI088::readGyroscope(int16_t &x, int16_t &y, int16_t &z) 
{
    if(bmiStatus == BMI088_errorCode::BMI088_success) 
    {
        uint8_t dataSize = 6;
        uint8_t hexValues[dataSize] = {GYRO_X_LSB, GYRO_X_MSB, GYRO_Y_LSB, GYRO_Y_MSB, GYRO_Z_LSB, GYRO_Z_MSB};
        uint8_t dataValues[dataSize];

        for(uint8_t i = 0; i < dataSize; i++) 
        {
            dataValues[i] = readRegister(GYROSCOPE, hexValues[i]);
        }

        x = (int16_t)((dataValues[0] << 8) | dataValues[1]) * gyroScaleFactor;
        y = (int16_t)((dataValues[2] << 8) | dataValues[3]) * gyroScaleFactor;
        z = (int16_t)((dataValues[4] << 8) | dataValues[5]) * gyroScaleFactor;

        bmiStatus = BMI088_errorCode::BMI088_success;
        return BMI088_errorCode::BMI088_success;
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;
    return BMI088_errorCode::BMI088_bmiError;
}

BMI088_errorCode BMI088::readGyroscopeDMA() 
{
    if(bmiStatus == BMI088_errorCode::BMI088_success) 
    {
        uint8_t errorState = 0;
        uint8_t dataSize = 6;
        uint8_t txData[dataSize];

        txData[0] = GYRO_X_LSB | 0x80;
        
        setCsPin(GYROSCOPE, LOW);
        errorState += HAL_SPI_TransmitReceive_DMA(SPI, txData, gyroDmaData, dataSize);
        setCsPin(GYROSCOPE, HIGH);

        if(errorState != HAL_OK)
        {
        	bmiStatus = BMI088_errorCode::BMI088_spiError;
            return BMI088_errorCode::BMI088_spiError;
        }

        bmiStatus = BMI088_errorCode::BMI088_success;
        return BMI088_errorCode::BMI088_success;
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;

    return BMI088_errorCode::BMI088_bmiError;
}

BMI088_errorCode BMI088::readGyroscopeDMAComplete(int16_t &x, int16_t &y, int16_t &z) 
{
    if(bmiErrorCode == BMI088_errorCode::BMI088_success)
    {
    	x = (int16_t)((gyroDmaData[0] << 8) | gyroDmaData[1]) * gyroScaleFactor;
    	y = (int16_t)((gyroDmaData[2] << 8) | gyroDmaData[3]) * gyroScaleFactor;
    	z = (int16_t)((gyroDmaData[4] << 8) | gyroDmaData[5]) * gyroScaleFactor;

    	bmiStatus = BMI088_errorCode::BMI088_success;
    	return BMI088_errorCode::BMI088_success;
    }

    bmiStatus = BMI088_errorCode::BMI088_bmiError;

    return BMI088_errorCode::BMI088_bmiError;
}
