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
#include "nrf_log.h"
#include "mlx90632.h"
#include "mlx90632_depends.h"
#include "sensor_tempe.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
int16_t ambient_new_raw;
int16_t ambient_old_raw;
int16_t object_new_raw;
int16_t object_old_raw;
int32_t PR = 0x00587f5b;
int32_t PG = 0x04a10289;
int32_t PT = 0xfff966f8;
int32_t PO = 0x00001e0f;
int32_t Ea = 4859535;
int32_t Eb = 5686508;
int32_t Fa = 53855361;
int32_t Fb = 42874149;
int32_t Ga = -14556410;
int16_t Ha = 16384;
int16_t Hb = 0;
int16_t Gb = 9728;
int16_t Ka = 10752;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

void sensor_tempe_init()
{
    int32_t err_code;
    err_code =  mlx90632_comms_initialise(100);
    require(err_code == 0, lb_exit);
    err_code = mlx90632_init();
    require(err_code == 0, lb_exit);
    err_code = mlx90632_read_calibration_parameters(&PR, &PG, &PO, &PT, &Ea, &Eb, &Fa, &Fb, &Ga, &Gb, &Ha, &Hb, &Ka);
    require(err_code == 0, lb_exit);
    NRF_LOG_INFO("[sensor] mlx90632 read calibration success");
    mlx90632_comms_close();
    return;

lb_exit:
    NRF_LOG_ERROR("[sensor] mlx90632 init fail %d", err_code);
    mlx90632_comms_close();
    return;
}

int32_t sensor_tempe_read_celciusx100() 
{
    int32_t err_code;
    int16_t ambient_new_raw;
    int16_t ambient_old_raw;
    int16_t object_new_raw;
    int16_t object_old_raw;
    double pre_ambient;
    double pre_object;
    double ambient;
    double object;

    err_code = mlx90632_comms_initialise(100);
    if (err_code != 0)
    {
        NRF_LOG_ERROR("[sensor] mlx90632_init fail");
    }
    /* Get raw data from MLX90632 */
    err_code = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw);
    if (err_code < 0)
    {
        NRF_LOG_ERROR("[sensor] mlx90632_read_temp_raw fail %d", err_code);
        goto lb_exit;
    }

    /* Pre-calculations for ambient and object temperature calculation */
    pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);
    pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw, ambient_new_raw, ambient_old_raw, Ka);
    /* Set emissivity = 1 */
    mlx90632_set_emissivity(1.0f);
    /* Calculate ambient and object temperature */
    ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw, PT, PR, PG, PO, Gb);
    object = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);

    NRF_LOG_INFO("====== Ta %d.%d To %d.%d [%d]", ambient, (int)(ambient * 100) % 100, object, (int)(object * 100) % 100, err_code)

lb_exit:
    mlx90632_comms_close();
    return (int32_t) (object * 100);
}