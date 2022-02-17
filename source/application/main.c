/*******************************************************************************
 *
 * Copyright (c) 2021
 * thangdh92@gmail.com
 * All Rights Reserved
 *
 *
 * Description:
 *
 * Author: 
 *
 * Last Changed By:  $ Author:  $
 * Revision:         $ Revision:  $
 * Last Changed:     $ Date:  $
 *
 ******************************************************************************/
 
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "nordic_common.h"

#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hts.h"
#include "ble_dis.h"
#include "ble_conn_params.h"

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf.h"
#include "nrf_delay.h"

#include "bsp.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
//#include "nrf_drv_rtc.h" // not used due to use app_timer
#include "drv_rtc.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_saadc.h"

#include "app_config.h"
#include "scommon.h"
#include "sensor_tempe.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define APP_BLE_OBSERVER_PRIO            3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG             1                                   /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_FAST_INTERVAL            40                                  /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_SLOW_INTERVAL            160                                 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 100ms). */
#define APP_ADV_DIRECTED_INTERVAL        160                                 /**< The advertising interval (in units of 0.625 ms. This value corresponds to 100ms). */
#define APP_ADV_FAST_DURATION            1000                                /**< The advertising duration (10 seconds) in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION            1500                                /**< The advertising duration (15 seconds) in units of 10 milliseconds. */
#define APP_ADV_DIRECTED_DURATION        1500                                /**< The advertising duration (15 seconds) in units of 10 milliseconds. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(2000)               /**< Battery level measurement interval (ticks). */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(330, UNIT_1_25_MS)    /**< Minimum acceptable connection interval (0.5 seconds) */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(660, UNIT_1_25_MS)    /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                    2                                   /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                /**< Time from initiating event (connect or start of indication) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000)               /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                    /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                    /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                    /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                    /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                    /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                 /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                    /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                    /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                   /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                           /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SENSORS_NOTIFICATION_INTERVAL    APP_TIMER_TICKS(1000)                /**< Sample at rate of 1000ms */
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
APP_TIMER_DEF(m_battery_timer_id);                                      /**< Battery timer. */
BLE_BAS_DEF(m_bas);                                                   /**< Structure used to identify the battery service. */
BLE_HTS_DEF (m_hts);                                                   /**< Structure used to identify the health thermometer service. */
NRF_BLE_GATT_DEF(m_gatt);                                                  /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                   /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                           /**< Advertising module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                         /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_PERIPHERAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

APP_TIMER_DEF(m_sensors_timer_id);

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                   /**< Handle of the current connection. */
static bool m_hts_meas_ind_conf_pending = false;                       /**< Flag to keep track of when an indication confirmation is pending. */

static ble_uuid_t m_adv_uuids[] =                                            /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE         },
    {BLE_UUID_BATTERY_SERVICE           , BLE_UUID_TYPE_BLE         },
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE         }
};

bool m_tnsg_connected = false;
bool m_tnsg_advertising = false;

static int32_t g_celciusx100;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void advertising_start(bool erase_bonds, bool fast_mode);
static void temperature_measurement_send(uint32_t celciusx100);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;
    bool is_indication_enabled;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    NRF_LOG_DEBUG("[pm_evt] code = %d", p_evt->evt_id);
    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            // Send a single temperature measurement if indication is enabled.
            // NOTE: For this to work, make sure ble_hts_on_ble_evt() is called before
            // pm_evt_handler() in ble_evt_dispatch().
            err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
            APP_ERROR_CHECK(err_code);
            if (is_indication_enabled)
            {
                temperature_measurement_send(g_celciusx100);
            }
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false, true);
            break;

        default:
            
            break;
    }
}


/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    static uint8_t battery_level = 0;

    battery_level++;
    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for implement thermal measurement
 *
 * @details This function will be called when Periodic counter timout or singleshot mode
 *
 * @param none
 */
static void measure_thermal(void) {
    ret_code_t err_code;
    bool is_indication_enabled;

    // PWR UP

    // pause to avoid excute time more than 1 sec
    app_timer_stop(m_sensors_timer_id);

    // Read sensor
    g_celciusx100 = sensor_tempe_read_celciusx100();
    // range_sample
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = ble_hts_is_indication_enabled(&m_hts, &is_indication_enabled);
        if (err_code == NRF_SUCCESS && is_indication_enabled)
        {
            temperature_measurement_send(g_celciusx100);
        }
    }

    // resume to avoid excute time more than 1 sec
    app_timer_start(m_sensors_timer_id, SENSORS_NOTIFICATION_INTERVAL, NULL);

    // PWR DOWN
}


/**@brief Function for handling the tnsg timer timeout.
 *
 * @details This function will be called each time the tnsg timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void sensors_handler(void * p_context)
{
    measure_thermal();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_handler);
    APP_ERROR_CHECK(err_code);

    // Create timers sensor.
    err_code = app_timer_create(&m_sensors_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensors_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                           strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for simulating and sending one Temperature Measurement.
 */
static void temperature_measurement_send(uint32_t celciusx100)
{
    ble_hts_meas_t tempe_meas;
    ble_hts_meas_t *p_meas = &tempe_meas;
    ret_code_t err_code;

    if (!m_hts_meas_ind_conf_pending)
    {
        static ble_date_time_t time_stamp = { 2020, 4, 30, 11, 30, 0 };
        p_meas->temp_in_fahr_units = false;
        p_meas->time_stamp_present = true;
        p_meas->temp_type_present  = (TEMP_TYPE_AS_CHARACTERISTIC ? false : true);

        p_meas->temp_in_celcius.exponent = -2;
        p_meas->temp_in_celcius.mantissa = celciusx100;
        p_meas->temp_in_fahr.exponent    = -2;
        p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusx100 * 9) / 5);
        p_meas->time_stamp               = time_stamp;
        p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;

        err_code = ble_hts_measurement_send(&m_hts, &tempe_meas);
        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
                m_hts_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
    }
}


/**@brief Function for handling the Health Thermometer Service events.
 *
 * @details This function will be called for all Health Thermometer Service events which are passed
 *          to the application.
 *
 * @param[in] p_hts  Health Thermometer Service structure.
 * @param[in] p_evt  Event received from the Health Thermometer Service.
 */
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HTS_EVT_INDICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
            //temperature_measurement_send();
            break;

        case BLE_HTS_EVT_INDICATION_CONFIRMED:
            m_hts_meas_ind_conf_pending = false;
            break;

        case BLE_HTS_EVT_INDICATION_DISABLED:
            // No implementation needed.
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Health Thermometer, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hts_init_t     hts_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dis_sys_id_t   sys_id;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Health Thermometer Service
    memset(&hts_init, 0, sizeof(hts_init));

    hts_init.evt_handler                 = on_hts_evt;
    hts_init.p_gatt_queue                = &m_ble_gatt_queue;
    hts_init.error_handler               = service_error_handler;
    hts_init.temp_type_as_characteristic = TEMP_TYPE_AS_CHARACTERISTIC;
    hts_init.temp_type                   = BLE_HTS_TEMP_TYPE_BODY;

    // Here the sec level for the Health Thermometer Service can be changed/increased.
//    hts_init.ht_meas_cccd_wr_sec = SEC_JUST_WORKS;
    hts_init.ht_meas_cccd_wr_sec = SEC_OPEN;
    hts_init.ht_type_rd_sec      = SEC_OPEN;

    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);

    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start battery timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL  , NULL);
    APP_ERROR_CHECK(err_code);

    // Start sensors timers
    err_code = app_timer_start(m_sensors_timer_id, SENSORS_NOTIFICATION_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("[adv_evt] ADV_FAST");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 0);
            APP_ERROR_CHECK(err_code);
            m_tnsg_advertising = true;
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("[adv_evt] ADV_SLOW");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, -8);
            APP_ERROR_CHECK(err_code);
            m_tnsg_advertising = true;
            break;

        case BLE_ADV_MODE_DIRECTED:
            NRF_LOG_INFO("[adv_evt] ADV_DIRECTED");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 0);
            APP_ERROR_CHECK(err_code);
            m_tnsg_advertising = true;
            break;

        case BLE_ADV_MODE_DIRECTED_HIGH_DUTY:
            NRF_LOG_INFO("[adv_evt] ADV_DIRECTED_HIGH_DUTY");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 0);
            APP_ERROR_CHECK(err_code);
            m_tnsg_advertising = true;
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("[adv_evt] ADV_IDLE");
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_tnsg_advertising = false;
//            NRF_LOG_INFO("[adv_evt] Enter sleep mode");
//            NRF_LOG_FINAL_FLUSH();
//            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;
    const    int8_t tx_power_array[9] = {-40, -20, -16, -12, -8, -4, 0, 3, 4}; //-40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm
            uint8_t channel ;
                
    static   int8_t     rssi         = -55;
             int8_t old_rssi        ;
    static  uint8_t     tx_power_idx = 3;
            uint8_t old_tx_power_idx;
    static   int8_t     tx_power     = -12;
             int8_t old_tx_power    ;
    static   bool       fast_rssi    = false;
             bool   old_fast_rssi   ;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("[ble_evt] GAP_CONNECTED");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_rssi_start(p_ble_evt->evt.gap_evt.conn_handle, 8, 3);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("[ble_evt] GAP_DISCONNECTED");
            m_conn_handle               = BLE_CONN_HANDLE_INVALID;
            m_hts_meas_ind_conf_pending = false;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            NRF_LOG_DEBUG("[ble_evt] GAP_PHY_UPDATE_REQUEST");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("[ble_evt] GATTC_TIMEOUT");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("[ble_evt] GATTS_TIMEOUT");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_RSSI_CHANGED:
            // save old value
            old_rssi         = rssi;
            old_tx_power     = tx_power;
            old_fast_rssi    = fast_rssi;
            old_tx_power_idx = tx_power_idx;
            err_code = sd_ble_gap_rssi_get(p_ble_evt->evt.gatts_evt.conn_handle, &rssi, &channel);
            APP_ERROR_CHECK(err_code);

            if        (rssi > -20) { tx_power_idx = 0;// -40
            } else if (rssi > -30) { tx_power_idx = 1;// -20
            } else if (rssi > -40) { tx_power_idx = 2;// -16
            } else if (rssi > -50) { tx_power_idx = 3;// -12
            } else if (rssi > -60) { tx_power_idx = 4;// -8
            } else if (rssi > -70) { tx_power_idx = 5;// -4
            } else if (rssi > -80) { tx_power_idx = 6;//  0
            } else if (rssi > -90) { tx_power_idx = 7;//  3
            } else                 { tx_power_idx = 8;//  4
            }
            if        (old_tx_power_idx < tx_power_idx) { // it means that rssi decrease
                // set immediate new big power
                tx_power = tx_power_array[tx_power_idx];
                err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, p_ble_evt->evt.gap_evt.conn_handle, tx_power);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO (    "[ble_evt] GAP_RSSI-- %3d=>%3d, TXpwr++:%3d=>%3d", old_rssi, rssi, old_tx_power, tx_power);
            } else if (old_tx_power_idx > tx_power_idx) { // it means that rssi increase
                if ((old_tx_power_idx - tx_power_idx) > 1) { // huge change power (at least 2 step)
                    tx_power = tx_power_array[tx_power_idx];
                    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, p_ble_evt->evt.gap_evt.conn_handle, tx_power);
                    APP_ERROR_CHECK(err_code);
                    NRF_LOG_INFO ("[ble_evt] GAP_RSSI++ %3d=>%3d, TXpwr--:%3d=>%3d", old_rssi, rssi, old_tx_power, tx_power);
                } else { // default dont decrease tx power if any smale change
                    tx_power_idx = old_tx_power_idx;
                    tx_power     = old_tx_power;
                    NRF_LOG_DEBUG("[ble_evt] GAP_RSSI++ %3d=>%3d, TXpwr keep   %3d", old_rssi, rssi, old_tx_power);
                }
            } else {
                tx_power_idx = old_tx_power_idx;
                tx_power     = old_tx_power;
                NRF_LOG_DEBUG(    "[ble_evt] GAP_RSSI   %3d~~%3d, TXpwr keep   %3d", old_rssi, rssi, old_tx_power);
            }
            
            if((rssi > -40) || (rssi < -80)) {
                fast_rssi = true;
            } else {
                fast_rssi = false;
            }
            if (old_fast_rssi != fast_rssi) {
                sd_ble_gap_rssi_stop(p_ble_evt->evt.gap_evt.conn_handle);
                APP_ERROR_CHECK(err_code);
                if (fast_rssi) {
                    err_code = sd_ble_gap_rssi_start(p_ble_evt->evt.gap_evt.conn_handle, 5, 3);
                } else {
                    err_code = sd_ble_gap_rssi_start(p_ble_evt->evt.gap_evt.conn_handle, 5, 5);
                }
            }  
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;


    switch (event)
    {
        case BSP_EVENT_SLEEP:
            NRF_LOG_DEBUG("[bsp_evt] BSP_SLEEP");
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            NRF_LOG_DEBUG("[bsp_evt] BSP_DISCONNECT");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            NRF_LOG_DEBUG("[bsp_evt] BSP_WHITELIST_OFF");
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_0:
            NRF_LOG_DEBUG("[bsp_evt] BSP_KEY_0");
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                temperature_measurement_send(g_celciusx100);
            }
            break;

        case BSP_EVENT_KEY_1:
            NRF_LOG_DEBUG("[bsp_evt] BSP_KEY_1");
            break;

        case BSP_EVENT_KEY_2:
            NRF_LOG_DEBUG("[bsp_evt] BSP_KEY_2");
            break;
        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
//    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.advdata.uuids_more_available.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_more_available.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_FAST_DURATION;

    init.config.ble_adv_slow_enabled  = true;
    init.config.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout  = APP_ADV_SLOW_DURATION;

    init.config.ble_adv_directed_enabled  = false;
    init.config.ble_adv_directed_interval = APP_ADV_DIRECTED_INTERVAL;
    init.config.ble_adv_directed_timeout  = APP_ADV_DIRECTED_DURATION;

    init.config.ble_adv_whitelist_enabled = false;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds, bool fast_mode)
{
    uint32_t err_code;

    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        if (fast_mode == true)
        {
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        }
        else 
        {
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_SLOW);
        }
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    log_init();
    timers_init();

    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();

    conn_params_init();
    peer_manager_init();

    NRF_LOG_INFO("MLX90632 demo started");

    #if ENABLE_TEMPE_SENSOR
    sensor_tempe_init();
    #endif

    application_timers_start();
    advertising_start(erase_bonds, true);

    // Enter main loop.
    while (1)
    {
        idle_state_handle();
    }
}
