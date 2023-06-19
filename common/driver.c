#include "driver.h"
#include <string.h>

// open the Linux device
int i2c_open(const char *file)
{
    int fd = open(file, O_RDWR);
    if (fd < 0)
    {
        perror("i2c_open\n");
        exit(1);
    }
    fprintf(stdout, "fd: %d\n", fd);
    return fd;
}

// close the Linux device
void i2c_close(int fd)
{
    close(fd);
}

// set the I2C slave address for all subsequent I2C device transfers
void i2c_set_addr(int fd, int addr)
{
    if (ioctl(fd, I2C_SLAVE, addr) < 0)
    {
        perror("i2c set address\n");
        exit(1);
    }
}

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                     uint32_t len, void *intf_ptr, void *fd)
{
    int8_t rslt = BME68X_OK;
    (void) intf_ptr;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    int *fd_ptr = (int*)fd;
    int i2c_fd = *fd_ptr;

    i2c_set_addr(i2c_fd, dev_addr);

    if (i2c_smbus_read_i2c_block_data(i2c_fd, reg_addr, len, reg_data) < 0)
    {
        fprintf(stderr, "user_i2c_read_data\n");
        rslt = BME68X_E_COM_FAIL;
    }

    return rslt;
}

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                      uint32_t len, void *intf_ptr, void *fd)
{
    int8_t rslt = BME68X_OK;
    (void) intf_ptr;
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    int *fd_ptr = (int*)fd;
    int i2c_fd = *fd_ptr;

    i2c_set_addr(i2c_fd, dev_addr);

    if (i2c_smbus_write_i2c_block_data(i2c_fd, reg_addr, len, reg_data) < 0)
    {
        fprintf(stderr, "bme68x_i2c_write\n");
        rslt = BME68X_E_COM_FAIL;
    }
    
    return rslt;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_us     Time in microseconds
 * @param[in]       intf_ptr Pointer to the interface descriptor
 * 
 * @return          none
 */
void bme68x_delay_us(uint32_t t_us, void *intf_ptr)
{
    (void)intf_ptr;
    struct timespec ts;
    ts.tv_sec = t_us / 1000000;
    ts.tv_nsec = (t_us % 1000000) * 1000L;
    nanosleep(&ts, NULL);
}

int64_t get_timestamp_us()
{
  struct timespec spec;
  //clock_gettime(CLOCK_REALTIME, &spec);
  /* MONOTONIC in favor of REALTIME to avoid interference by time sync. */
  clock_gettime(CLOCK_MONOTONIC, &spec);

  int64_t system_current_time_ns = (int64_t)(spec.tv_sec) * (int64_t)1000000000
                                   + (int64_t)(spec.tv_nsec);
  int64_t system_current_time_us = system_current_time_ns / 1000;

  return system_current_time_us;
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
        bme68x_delay_us(100, NULL);

        /* Bus configuration : I2C */
        if (intf == BME68X_I2C_INTF)
        {
            printf("I2C Interface\n");
            bme->read = bme68x_i2c_read;
            bme->write = bme68x_i2c_write;
            bme->intf = BME68X_I2C_INTF;
        }

        bme->delay_us = bme68x_delay_us;
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
