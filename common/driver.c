#include <string.h>
#include "driver.h"
#include "debug.h"
#include "ch32f20x.h"
#include "FreeRTOS.h"
#include "task.h"

/* I2C Mode Definition */
#define HOST_MODE   0
#define SLAVE_MODE   1

/* I2C Communication Mode Selection */
#define I2C_MODE   HOST_MODE

/*********************************************************************
 * @fn      i2c_init
 *
 * @brief   Initializes the IIC peripheral.
 *
 * @return  none
 */
void i2c_init(void *fd, uint32_t bound, uint16_t address)
{
	I2C_TypeDef *i2c_fd = (I2C_TypeDef *)fd;
	GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitTSturcture = {0};
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
	if (i2c_fd == I2C1)
	{
		printf("Init i2c bus 1\r\n");
		// SCL
		RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE );
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		// SDA
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
	else if (i2c_fd == I2C2)
	{
		printf("Init i2c bus 2\r\n");
		// SCL
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		// SDA
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}
	
	I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(i2c_fd, &I2C_InitTSturcture);

    I2C_Cmd(i2c_fd, ENABLE);

    I2C_AcknowledgeConfig(i2c_fd, ENABLE);
}

// close the Linux device
void i2c_close(void *fd)
{
	(void) fd;
    //return ;
}

/*********************************************************************
 * @fn      AT24CXX_ReadOneByte
 *
 * @brief   Read one data from EEPROM.
 *
 * @param   ReadAddr - Read frist address.
 *
 * @return  temp - Read data.
 */
uint8_t I2C_ReadOneByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint16_t ReadAddr)
{
    uint8_t temp = 0;

    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) != RESET);
    I2C_GenerateSTART(I2Cx, ENABLE);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

#if (Address_Lenth  == Address_8bit)
    I2C_SendData(I2Cx, (uint8_t)(ReadAddr & 0x00FF));
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

#elif (Address_Lenth  == Address_16bit)
    I2C_SendData(I2C2, (uint8_t)(ReadAddr >> 8));
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_SendData(I2C2, (uint8_t)(ReadAddr & 0x00FF));
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

#endif

    I2C_GenerateSTART(I2Cx, ENABLE);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Receiver);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE) == RESET)
    {
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
    }

    temp = I2C_ReceiveData(I2Cx);
    I2C_GenerateSTOP(I2Cx, ENABLE);

    return temp;
}


/*********************************************************************
 * @fn      I2C_WriteOneByte
 *
 * @brief   Write one data to I2C device.
 *
 * @param   WriteAddr - Write frist address.
 *
 * @return  DataToWrite - Write data.
 */
void I2C_WriteOneByte(I2C_TypeDef *I2Cx, uint8_t slaveAddr, uint16_t WriteAddr, uint8_t DataToWrite)
{
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY) != RESET);
    I2C_GenerateSTART(I2Cx, ENABLE);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

#if (Address_Lenth == Address_8bit)
    I2C_SendData(I2Cx, (uint8_t)(WriteAddr & 0x00FF));
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

#elif (Address_Lenth == Address_16bit)
    I2C_SendData(I2C2, (uint8_t)(WriteAddr >> 8));
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_SendData(I2C2, (uint8_t)(WriteAddr & 0x00FF));
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

#endif

    if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) != RESET)
    {
        I2C_SendData(I2Cx, DataToWrite);
    }

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(I2Cx, ENABLE);
}

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                     uint32_t len, void *intf_ptr, void *fd)
{
    int8_t rslt = BME68X_OK;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint8_t i;
    int *fd_ptr = (int*)fd;
    I2C_TypeDef *i2c_fd = (I2C_TypeDef *)fd_ptr;
	(void) intf_ptr;

    while(I2C_GetFlagStatus(i2c_fd, I2C_FLAG_BUSY ) != RESET);
    I2C_GenerateSTART(i2c_fd, ENABLE);

	// Set slave addr
    while(!I2C_CheckEvent(i2c_fd, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(i2c_fd, dev_addr, I2C_Direction_Transmitter);

    while(!I2C_CheckEvent(i2c_fd, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	// Send register addr
	I2C_SendData(i2c_fd, reg_addr);
	while(!I2C_CheckEvent(i2c_fd, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){};
	
	// Generate another start signal
    I2C_GenerateSTART(i2c_fd, ENABLE);
	
	// Set slave addr
    while(!I2C_CheckEvent(i2c_fd, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(i2c_fd, dev_addr, I2C_Direction_Receiver);

	while(!I2C_CheckEvent(i2c_fd, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){}
	I2C_GenerateSTOP(i2c_fd, DISABLE);
	
    for (i = 0; i < len; i++)
	{
		if (i == len -1 ) {
            I2C_AcknowledgeConfig(i2c_fd, DISABLE);
        }
		while(!I2C_GetFlagStatus(i2c_fd, I2C_FLAG_RXNE));
		reg_data[i] = I2C_ReceiveData(i2c_fd);
	}
	I2C_GenerateSTOP(i2c_fd, ENABLE);
	I2C_AcknowledgeConfig(i2c_fd, ENABLE);
    return rslt;
}

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                      uint32_t len, void *intf_ptr, void *fd)
{
	// Variables
    int8_t rslt = BME68X_OK;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    int *fd_ptr = (int*)fd;
    I2C_TypeDef *i2c_fd = (I2C_TypeDef *)fd_ptr;
	uint8_t i;
	(void) intf_ptr;

    while(I2C_GetFlagStatus(i2c_fd, I2C_FLAG_BUSY) != RESET);
    I2C_GenerateSTART(i2c_fd, ENABLE);

	// Set slave addr
    while(!I2C_CheckEvent(i2c_fd, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(i2c_fd, dev_addr, I2C_Direction_Transmitter);

    while( !I2C_CheckEvent(i2c_fd, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // Send register addr
	I2C_SendData(i2c_fd, reg_addr);
	while(!I2C_CheckEvent(i2c_fd, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){}
	
    for (i = 0; i < len; i++)
	{
		I2C_SendData(i2c_fd, reg_data[i]);
		while(!I2C_CheckEvent(i2c_fd, I2C_EVENT_MASTER_BYTE_TRANSMITTED));			
	}
    
	I2C_GenerateSTOP(i2c_fd, ENABLE);
    return rslt;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms     Time in milliseconds
 * @param[in]       intf_ptr Pointer to the interface descriptor
 * 
 * @return          none
 */
void bme68x_delay_ms(uint32_t t_ms, void *intf_ptr)
{
    (void)intf_ptr;
#ifdef ENABLE_FREERTOS
	vTaskDelay(pdMS_TO_TICKS(t_ms));
#else
	Delay_Ms(t_ms);
#endif
}

void get_timestamp_ms(uint32_t *tickcount)
{
#ifdef ENABLE_FREERTOS
    *tickcount = xTaskGetTickCount();
#else
	//Delay_Ms(t_us/1000);
	SysTick->VAL;
#endif
  
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:
            printf("API name [%s]  Info [%d] : Success\r\n", api_name, rslt);
            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf, void *intf_ptr, void *fd)
{
    int8_t rslt = BME68X_OK;

    if (bme != NULL)
    {
        Delay_Us(100);

        /* Bus configuration : I2C */
        if (intf == BME68X_I2C_INTF)
        {
            printf("I2C Interface\n");
            bme->read = bme68x_i2c_read;
            bme->write = bme68x_i2c_write;
            bme->intf = BME68X_I2C_INTF;
        }

        bme->delay_ms = bme68x_delay_ms;
        bme->intf_ptr = intf_ptr;
        bme->fd = fd;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    }
    else
    {
        rslt = BME68X_E_NULL_PTR;
    }

    return rslt;
}
