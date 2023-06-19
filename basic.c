#include <stdio.h>
#include "basic.h"
#include "driver.h"
#include "bsec_integration.h"

char *filename_state = "./bme680_iaq_33v_3s_4d/bsec_iaq.state";
char *filename_config = "./bme680_iaq_33v_3s_4d/bsec_iaq.config";

char *get_version(void)
{
    char *buffer = (char *)malloc(sizeof(char) * 16);
    bsec_version_t bsec_version;
    bsec_get_version(&bsec_version);
    snprintf(buffer, 16, "%d.%d.%d.%d", bsec_version.major, bsec_version.minor,
             bsec_version.major_bugfix, bsec_version.minor_bugfix);
    return buffer;
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp               time in milliseconds
 * @param[in]       gas_estimate_1          gas estimate 1
 * @param[in]       gas_estimate_2          gas estimate 2
 * @param[in]       gas_estimate_3          gas estimate 3
 * @param[in]       gas_estimate_4          gas estimate 4 
 * @param[in]       raw_pressure            raw pressure
 * @param[in]       raw_temp                raw temperature
 * @param[in]       raw_humidity            raw humidity
 * @param[in]       raw_gas                 raw gas
 * @param[in]       raw_gas_index           raw gas index
 * @param[in]       bsec_status             value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float gas_estimate_1, float gas_estimate_2, float gas_estimate_3, float gas_estimate_4,
                    float raw_pressure, float raw_temp, float raw_humidity, float raw_gas, uint8_t raw_gas_index, bsec_library_return_t bsec_status)
{
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);

    printf("{\"Gas_estimate_1\": \"%.2f\",\"Gas_estimate_2\":\"%.2f\"", gas_estimate_1, gas_estimate_2);
    printf(",\"Gas_estimate_3\": \"%.2f\",\"Gas_estimate_4\":\"%.2f\"", gas_estimate_3, gas_estimate_4);
    printf(",\"Temperature\": \"%.2f\",\"Humidity\": \"%.2f\",\"Pressure\": \"%.2f\"", raw_temp, raw_humidity, raw_pressure / 100);
    printf(",\"Gas\": \"%.0f\"", raw_gas);
    printf(",\"Status\": \"%d\"}", bsec_status);
    // printf(",%" PRId64, timestamp);
    // printf(",%" PRId64, timestamp_ms);
    printf("\r\n");
    fflush(stdout);
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
// void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy,
//                   float temperature, float humidity, float pressure,
//                   float raw_temperature, float raw_humidity, float gas,
//                   bsec_library_return_t bsec_status)
// {
//   //int64_t timestamp_s = timestamp / 1000000000;
//   ////int64_t timestamp_ms = timestamp / 1000;

//   //time_t t = timestamp_s;
//   /*
//    * timestamp for localtime only makes sense if get_timestamp_us() uses
//    * CLOCK_REALTIME
//    */
//   time_t t = time(NULL);
//   struct tm tm = *localtime(&t);

//   printf("{\"IAQ_Accuracy\": \"%d\",\"IAQ\":\"%.2f\"", iaq_accuracy, iaq);
//   printf(",\"Temperature\": \"%.2f\",\"Humidity\": \"%.2f\",\"Pressure\": \"%.2f\"", temperature, humidity,pressure / 100);
//   printf(",\"Gas\": \"%.0f\"", gas);
//   printf(",\"Status\": \"%d\"}", bsec_status);
//   //printf(",%" PRId64, timestamp);
//   //printf(",%" PRId64, timestamp_ms);
//   printf("\r\n");
//   fflush(stdout);
// }

/*!
 * @brief Load binary file from non-volatile memory into buffer. Utility function
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded data
 * @param[in]       n_buffer        size of the allocated buffer
 * @param[in]       filename        name of the file on the NVM
 * @param[in]       offset          offset in bytes from where to start copying
 *                                  to buffer
 * @return          number of bytes copied to buffer or zero on failure
 */
static uint32_t binary_load(uint8_t *b_buffer, uint32_t n_buffer,
                            char *filename, uint32_t offset)
{
    int32_t copied_bytes = 0;
    int8_t rslt = 0;

    struct stat fileinfo;
    rslt = stat(filename, &fileinfo);
    if (rslt != 0)
    {
        perror("stating binary file");
        return 0;
    }

    uint32_t filesize = fileinfo.st_size - offset;

    if (filesize > n_buffer)
    {
        fprintf(stderr, "%s: %d > %d\n", "binary data bigger than buffer", filesize,
                n_buffer);
        return 0;
    }
    else
    {
        FILE *file_ptr;
        file_ptr = fopen(filename, "rb");
        if (!file_ptr)
        {
            perror("fopen");
            return 0;
        }
        fseek(file_ptr, offset, SEEK_SET);
        copied_bytes = fread(b_buffer, sizeof(char), filesize, file_ptr);
        if (copied_bytes == 0)
        {
            fprintf(stderr, "%s\n", "binary_load");
        }
        fclose(file_ptr);
        return copied_bytes;
    }
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
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available, 
    // otherwise return length of loaded state string.
    // ...
    int32_t rslt = 0;
    rslt = binary_load(state_buffer, n_buffer, filename_state, 0);
    return rslt;
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
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
    FILE *state_w_ptr;
    state_w_ptr = fopen(filename_state, "wb");
    fwrite(state_buffer, length, 1, state_w_ptr);
    fclose(state_w_ptr);
}

/*
 * Load library config from non-volatile memory
 *
 * param[in,out]   config_buffer    buffer to hold the loaded state string
 * param[in]       n_buffer         size of the allocated state buffer
 *
 * return          number of bytes copied to config_buffer or zero on failure
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    int32_t rslt = 0;
    /*
     * Provided config file is 4 bytes larger than buffer.
     * Apparently skipping the first 4 bytes works fine.
     *
     */
    rslt = binary_load(config_buffer, n_buffer, filename_config, 4);
    return rslt;
}

int main(void)
{
    printf("bsec lib ver%s\n", get_version());
    int8_t rslt;
    return_values_init ret;
    int fd = i2c_open("/dev/i2c-3");
    struct bme68x_dev bme_dev;
    uint8_t bme680_addr = BME68X_I2C_ADDR_LOW;

    memset(&bme_dev, 0, sizeof(bme_dev));
    /* Interface preference is updated as a parameter
     * For I2C : BME68X_I2C_INTF
     * For SPI : BME68X_SPI_INTF
     */
    rslt = bme68x_interface_init(&bme_dev, BME68X_I2C_INTF, &bme680_addr, &fd);
    bme68x_check_rslt("bme68x_interface_init", rslt);
    /* Call to the function which initializes the BSEC library
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_SCAN, 0.0f, bme68x_delay_us, state_load, config_load, bme_dev);
    if (ret.bme68x_status)
    {
        /* Could not intialize BME68x */
        bme68x_check_rslt("bme68x_init", ret.bme68x_status);
        return (int)ret.bme68x_status;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        printf("Could not intialize BSEC library!\n");
        return (int)ret.bsec_status;
    }
    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every 10.000 samples, which means every 10.000 * 3 secs = 500 minutes  */
    bsec_iot_loop(bme68x_delay_us, get_timestamp_us, output_ready, state_save, 10000);
    return 0;
}
