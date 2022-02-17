#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "mlx90632_depends.h"

/**
 * @brief TWI master instance
 *
 * Instance of TWI master driver that would be used for communication with mlx90632.
 */
static const nrf_drv_twi_t m_mlx90632_twi = NRF_DRV_TWI_INSTANCE(MLX90632_INSTANCE_ID);
static volatile bool m_mlx90632_xfer_done = true;
static volatile nrf_drv_twi_evt_type_t m_mlx90632_xfer_event = NRF_DRV_TWI_EVT_DONE;

/**
 * @brief TWI events handler.
 */
void mlx90632_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    static uint16_t addr_nack_count = 0;
    static uint16_t data_nack_count = 0;
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            NRF_LOG_DEBUG("[mlx90632] NRF_DRV_TWI_EVT_DONE");
            m_mlx90632_xfer_event = p_event->type;
            m_mlx90632_xfer_done = true;
            break;

        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            addr_nack_count ++;
            NRF_LOG_ERROR("[mlx90632] NRF_DRV_TWI_EVT_ADDRESS_NACK (%0d)", addr_nack_count);
            m_mlx90632_xfer_event = p_event->type;
            m_mlx90632_xfer_done = true;
            break;

        case NRF_DRV_TWI_EVT_DATA_NACK:
            data_nack_count ++;
            NRF_LOG_ERROR("[mlx90632] NRF_DRV_TWI_EVT_DATA_NACK (%0d)", data_nack_count);
            m_mlx90632_xfer_event = p_event->type;
            m_mlx90632_xfer_done = true;
            break;

        default:
            NRF_LOG_ERROR("[mlx90632] Unexpected event");
            break;
    }
}

/**
 * @brief  Initialise platform comms.
 *
 * @param  comms_speed_khz - unsigned short containing the I2C speed in kHz
 *
 * @return status - status 0 = ok, 1 = error
 *
 */
int32_t mlx90632_comms_initialise(uint16_t comms_speed_khz)
{
    ret_code_t err_code;
    nrf_drv_twi_frequency_t nrf_speed;

    switch (comms_speed_khz)
    {
        case 400: nrf_speed = NRF_DRV_TWI_FREQ_400K; break;
        case 250: nrf_speed = NRF_DRV_TWI_FREQ_250K; break;
        case 100: nrf_speed = NRF_DRV_TWI_FREQ_100K; break;
        default:
        {
            NRF_LOG_ERROR("[mlx90632] Invalid TWI comms speed");
            return NRF_ERROR_INVALID_PARAM;
        }
    }

    nrf_drv_twi_config_t mlx90632_config =
    {
       .scl                = MLX90632_SCL,
       .sda                = MLX90632_SDA,
       .frequency          = nrf_speed,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_mlx90632_twi, &mlx90632_config, mlx90632_handler, NULL);
    if (NRF_SUCCESS == err_code)
    {
        nrf_drv_twi_enable(&m_mlx90632_twi);
    }
    else
    {
        NRF_LOG_ERROR("[mlx90632] TWI init failed");
    }

    return err_code;
};

/**
 * @brief  Close platform comms.
 *
 * @return status - status 0 = ok, 1 = error
 *
 */
int32_t mlx90632_comms_close(void){
    NRF_LOG_DEBUG("[mlx90632] Call nrf_drv_twi_disable");
    nrf_drv_twi_disable(&m_mlx90632_twi);
    nrf_drv_twi_uninit (&m_mlx90632_twi);
    return 0;
}

/** Read the register_address value from the mlx90632
 *
 * i2c read is processor specific and this function expects to have address of mlx90632 known, as it operates purely on
 * register addresses.
 *
 * @note Needs to be implemented externally
 * @param[in] register_address Address of the register to be read from
 * @param[out] *value pointer to where read data can be written

 * @retval 0 for success
 * @retval <0 for failure
 */
int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value)
{
    uint8_t buffer[2];
    uint8_t reg_add[2];
    ret_code_t err_code;

    reg_add[0] = (register_address >> 8) & 0xFF;
    reg_add[1] = (register_address) & 0xFF;
    nrf_drv_twi_xfer_desc_t m_rd_xfer = NRF_DRV_TWI_XFER_DESC_TXRX (
                    MLX90632_I2C_ADDR,
                    reg_add,
                    sizeof(reg_add),
                    (uint8_t*)&buffer,
                    sizeof(buffer)
                );

    if (m_mlx90632_xfer_done == true )
    {
        m_mlx90632_xfer_done = false;
    }
    else 
    {
        return -NRF_ERROR_BUSY;
    }
    err_code = nrf_drv_twi_xfer(&m_mlx90632_twi, &m_rd_xfer, 0);
    APP_ERROR_CHECK(err_code);
    if (NRF_SUCCESS != err_code) 
    {
        return -err_code;
    }
    while (m_mlx90632_xfer_done == false)
    {
        NRFX_DELAY_US(1); 
    }
    if (NRF_DRV_TWI_EVT_DONE != m_mlx90632_xfer_event)
    {
        return -m_mlx90632_xfer_event;
    }

    *value = (buffer[0] << 8) | (buffer[1] << 0);
    return 0;
}

/** Read the register_address value from the mlx90632
 *
 * i2c read is processor specific and this function expects to have address of mlx90632 known, as it operates purely on
 * register addresses.
 *
 * @note Needs to be implemented externally
 * @param[in] register_address Address of the register to be read from
 * @param[out] *value pointer to where read data can be written

 * @retval 0 for success
 * @retval <0 for failure
 */
int32_t mlx90632_i2c_read_dword(int16_t register_address, uint32_t *value)
{
    int32_t err = 0;
    uint16_t lo, hi;

    err |= mlx90632_i2c_read(register_address, &lo);
    err |= mlx90632_i2c_read(register_address + 1, &hi);

    *value = (hi << 16) + lo;
    return err;
}

/** Write value to register_address of the mlx90632
 *
 * i2c write is processor specific and this function expects to have address of mlx90632 known, as it operates purely
 * on register address and value to be written to.
 *
 * @note Needs to be implemented externally
 * @param[in] register_address Address of the register to be read from
 * @param[in] value value to be written to register address of mlx90632

 * @retval 0 for success
 * @retval <0 for failure
 */
int32_t mlx90632_i2c_write(int16_t register_address, uint16_t value)
{
    ret_code_t err_code;

    uint8_t buffer[4]; /* Addr + value */
    buffer[0] = (register_address >> 8) & 0xFF;
    buffer[1] = register_address & 0xFF;
    buffer[2] = (value >> 8) & 0xFF ;
    buffer[3] = value & 0xFF;

    if (m_mlx90632_xfer_done == true )
    {
        m_mlx90632_xfer_done = false;
    }
    else 
    {
        return NRF_ERROR_BUSY;
    }
    err_code = nrf_drv_twi_tx(&m_mlx90632_twi, MLX90632_I2C_ADDR, buffer, sizeof(buffer), false);
    APP_ERROR_CHECK(err_code);
    if (NRF_SUCCESS != err_code)
    {
        return err_code;
    }
    while (m_mlx90632_xfer_done == false)
    { 
        NRFX_DELAY_US(1);
    }

    return m_mlx90632_xfer_event;
}

/** Blocking function for sleeping in microseconds
 *
 * Range of microseconds which are allowed for the thread to sleep. This is to avoid constant pinging of sensor if the
 * data is ready.
 *
 * @note Needs to be implemented externally
 * @param[in] min_range Minimum amount of microseconds to sleep
 * @param[in] max_range Maximum amount of microseconds to sleep
 */
void usleep(int min_range, int max_range)
{
    nrf_delay_us((min_range + max_range)/2);
    return;
}

/** Blocking function for sleeping in milliseconds
 *
 * milliseconds which are allowed for the thread to sleep. This is to avoid constant pinging of sensor
 * while the measurement is ongoing in sleeping step mode.
 *
 * @note Needs to be implemented externally
 * @param[in] msecs Amount of milliseconds to sleep
 */
void msleep(int msecs)
{
    nrf_delay_ms(msecs);
    return;
}

///@}
