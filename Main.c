/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/08/08
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "driver.h"
#include "bsec_integration.h"
#include "bsec_iaq.h"


/* Global define */
#define TASK1_TASK_PRIO     5
#define TASK1_STK_SIZE      256
#define TASK2_TASK_PRIO     5
#define TASK2_STK_SIZE      256

/* Global Variable */
TaskHandle_t Task1Task_Handler;
TaskHandle_t Task2Task_Handler;

char *get_version(void)
{
    char *buffer = (char *)malloc(sizeof(char) * 16);
    bsec_version_t bsec_version;
    bsec_get_version(&bsec_version);
    snprintf(buffer, 16, "%d.%d.%d.%d", bsec_version.major, bsec_version.minor,
             bsec_version.major_bugfix, bsec_version.minor_bugfix);
    return buffer;
}

/*
 * Handling of the ready outputs
 *
 * param[in]       timestamp       time in microseconds
 * param[in]       iaq             IAQ signal
 * param[in]       iaq_accuracy    accuracy of IAQ signal
 * param[in]       temperature     temperature signal
 * param[in]       humidity        humidity signal
 * param[in]       pressure        pressure signal
 * param[in]       raw_temperature raw temperature signal
 * param[in]       raw_humidity    raw humidity signal
 * param[in]       gas             raw gas sensor signal
 * param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * return          none
 */
void output_ready(uint32_t timestamp, float iaq, uint8_t iaq_accuracy, float static_iaq, float co2_equivalent, float breath_voc_equivalent,
                  float raw_pressure, float raw_temp, float temp, float raw_humidity, float humidity, float raw_gas, float gas_percentage,
                  float stabilization_status, float run_in_status, bsec_library_return_t bsec_status)
{
    uint32_t timestamp_s = timestamp / 1000;

    // time_t t = timestamp_s;
    /*
     * timestamp for localtime only makes sense if get_timestamp_us() uses
     * CLOCK_REALTIME
     */

    printf("{\"IAQ Accuracy\": \"%d\", \"IAQ\":\"%.2f\", \"Static IAQ\": \"%.2f\"", iaq_accuracy, iaq, static_iaq);
    printf(", \"CO2 equivalent\": \"%.2f\", \"Breath VOC equivalent\": \"%.2f\"", co2_equivalent, breath_voc_equivalent);
    printf(", \"Raw Temperature\": \"%.2f\", \"Temperature\": \"%.2f\"", raw_temp, temp);
    printf(", \"Raw Humidity\": \"%.2f\", \"Humidity\": \"%.2f\",\"Pressure\": \"%.2f\"", raw_humidity, humidity, raw_pressure);
    printf(", \"Raw Gas\": \"%.0f\", \"Gas Percentage\":\"%.2f\"", raw_gas, gas_percentage);
    printf(", \"Stabilization status\": %.0f,\"Run in status\": %.0f,\"Status\": \"%d\"", stabilization_status, run_in_status, bsec_status);
    printf(", \"timestamp\": \"%d\"}", timestamp_s);
    printf("\r\n");
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
	(void)state_buffer;
	(void)n_buffer;
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
	(void)state_buffer;
	(void)length;
}

/*
 * Load library config from non-volatile memory
 *
 * param[in,out]   config_buffer    buffer to hold the loaded state string
 * param[in]       n_buffer         size of the allocated state buffer
 *
 * return          number of bytes copied to config_buffer or zero on failure
 */
static uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    //int32_t rslt = 0;
    /*
     * Provided config file is 4 bytes larger than buffer.
     * Apparently skipping the first 4 bytes works fine.
     *
     */
	uint32_t config_len = sizeof(bsec_config_iaq)/sizeof(uint8_t);
	if (config_len > n_buffer)
	{
		printf("%s: %d > %d\r\n", "binary data bigger than buffer", config_len, n_buffer);
		return 0;
	}
    memcpy(config_buffer, bsec_config_iaq, config_len);
    return 0;
}

/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0/1
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure={0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/*********************************************************************
 * @fn      task1_task
 *
 * @brief   task1 program.
 *
 * @param  *pvParameters - Parameters point of task1
 *
 * @return  none
 */
void task1_task(void *pvParameters)
{
		UBaseType_t msticks=0;
		msticks = pdMS_TO_TICKS(500);
    while(1)
    {
        printf("task1 entry\r\n");
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        vTaskDelay(msticks);
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        vTaskDelay(msticks);
    }
}

/*********************************************************************
 * @fn      task2_task
 *
 * @brief   task2 program.
 *
 * @param  *pvParameters - Parameters point of task2
 *
 * @return  none
 */
void task2_task(void *pvParameters)
{
	UBaseType_t msticks=0;
	msticks = pdMS_TO_TICKS(1000);
    while(1)
    {
        printf("task2 entry\r\n");
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
        vTaskDelay(msticks);
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
        vTaskDelay(msticks);
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    return_values_init ret;
	struct bme68x_dev bme_dev;
	I2C_TypeDef *bme680_i2c_fd = I2C2;
    uint8_t bme680_addr = BME68X_I2C_ADDR_LOW << 1;
	bsec_virtual_sensor_t sensor_list[13] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_RUN_IN_STATUS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_GAS_PERCENTAGE
    };
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("FreeRTOS Kernel Version:%s\r\n", tskKERNEL_VERSION_NUMBER);
	printf("bsec lib version:%s\r\n", get_version());
	memset(&bme_dev, 0, sizeof(bme_dev));
	
	// hardware init
	i2c_init(bme680_i2c_fd, 400000, 0xA6);
    GPIO_Toggle_INIT();
	
	/* Interface preference is updated as a parameter
     * For I2C : BME68X_I2C_INTF
     * For SPI : BME68X_SPI_INTF
     */
    ret.bme68x_status = bme68x_interface_init(&bme_dev, BME68X_I2C_INTF, &bme680_addr, bme680_i2c_fd);
    bme68x_check_rslt("bme68x_interface_init", ret.bme68x_status);
	
	// init bme680
	// ret.bme68x_status = bme68x_init(&bme_dev);
	// bme68x_check_rslt("bme68x_init", ret.bme68x_status);
	ret = bsec_iot_init(sensor_list, 13, BSEC_SAMPLE_RATE_LP, 0.0f, state_load, config_load, bme_dev);
	if (ret.bme68x_status)
    {
        /* Could not intialize BME68x */
        bme68x_check_rslt("bme68x_init", ret.bme68x_status);
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        printf("Can't not intialize BSEC library!\r\n");
		printf("BSEC warning code: %d\r\n", ret.bsec_status);
    }
    /* create two task */
    xTaskCreate((TaskFunction_t )task2_task,
                        (const char*    )"task2",
                        (uint16_t       )TASK2_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )TASK2_TASK_PRIO,
                        (TaskHandle_t*  )&Task2Task_Handler);

    xTaskCreate((TaskFunction_t )task1_task,
                    (const char*    )"task1",
                    (uint16_t       )TASK1_STK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )TASK1_TASK_PRIO,
                    (TaskHandle_t*  )&Task1Task_Handler);
    vTaskStartScheduler();

    while(1)
    {
        printf("shouldn't run at here!!\n");
    }
}



