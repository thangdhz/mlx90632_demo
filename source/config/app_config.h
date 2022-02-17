#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#define DEVICE_NAME                      "MLX90632"                         /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "MLX90632"                         /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                        "Upwork-001"                        /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                  0x1122334455                        /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                    0x667788                            /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define ENABLE_TEMPE_SENSOR              1
#define ENABLE_VBAT_SENSOR               0


#define MIN_BATTERY_LEVEL                81                                  /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL                100                                 /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT          1                                   /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define TEMP_TYPE_AS_CHARACTERISTIC      0                                   /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */

#define MIN_CELCIUS_DEGREES              3688                                /**< Minimum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define MAX_CELCIUS_DEGRESS              3972                                /**< Maximum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define CELCIUS_DEGREES_INCREMENT        36                                  /**< Value by which temperature is incremented/decremented for each call to the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */


// NRF_LOG_DEFAULT_LEVEL
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 
#define NRF_LOG_DEFAULT_LEVEL           3

#ifdef NRF52805_XXAA
// NRF_SDH_CLOCK_LF_SRC  - SoftDevice clock source.
// <0=> NRF_CLOCK_LF_SRC_RC 
// <1=> NRF_CLOCK_LF_SRC_XTAL 
// <2=> NRF_CLOCK_LF_SRC_SYNTH
#define NRF_SDH_CLOCK_LF_SRC 0

// NRF_SDH_CLOCK_LF_RC_CTIV - SoftDevice calibration timer interval. 
#define NRF_SDH_CLOCK_LF_RC_CTIV 16

// NRF_SDH_CLOCK_LF_RC_TEMP_CTIV - SoftDevice calibration timer interval under constant temperature. 
// <i> How often (in number of calibration intervals) the RC oscillator shall be calibrated
// <i>  if the temperature has not changed.
#define NRF_SDH_CLOCK_LF_RC_TEMP_CTIV 2

// NRF_SDH_CLOCK_LF_XTAL_ACCURACY  - External crystal clock accuracy used in the LL to compute timing windows.
// <0=> NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM 
// <1=> NRF_CLOCK_LF_XTAL_ACCURACY_500_PPM 
// <2=> NRF_CLOCK_LF_XTAL_ACCURACY_150_PPM 
// <3=> NRF_CLOCK_LF_XTAL_ACCURACY_100_PPM 
// <4=> NRF_CLOCK_LF_XTAL_ACCURACY_75_PPM 
// <5=> NRF_CLOCK_LF_XTAL_ACCURACY_50_PPM 
// <6=> NRF_CLOCK_LF_XTAL_ACCURACY_30_PPM 
// <7=> NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM 
#define NRF_SDH_CLOCK_LF_XTAL_ACCURACY 1

#endif

#endif // __APP_CONFIG_H
