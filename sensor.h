/*
 ## Cypress FX3 Camera Kit header file (sensor.h)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file defines the parameters and the interface for the image sensor driver.
 *
 * Sensor selection:
 *   Define SENSOR_ATTO640D04 to build for the Lynred ATTO640D-04 thermal sensor.
 *   If not defined, the default EV76C541 (Python 480) sensor is used.
 *
 *   Example: add -DSENSOR_ATTO640D04 to CFLAGS in the makefile.
 */

#ifndef _INCLUDED_SENSOR_H_
#define _INCLUDED_SENSOR_H_

#include <cyu3types.h>

/* ---------------------------------------------------------------------------
 * Sensor Selection
 *
 * Uncomment the following line (or pass -DSENSOR_ATTO640D04 via CFLAGS) to
 * build for the Lynred ATTO640D-04 thermal sensor instead of the default
 * EV76C541 (Python 480).
 * -------------------------------------------------------------------------*/
/* #define SENSOR_ATTO640D04 */

#ifdef SENSOR_ATTO640D04

/* Include the ATTO640D-04 specific header for register definitions and
 * sensor-specific APIs. The common SensorInit/SensorStart/etc. wrappers
 * in sensor.c will delegate to the ATTO640D-04 implementations.
 */
#include "atto640d04.h"

/* I2C Slave address for the ATTO640D-04 sensor */
#define SENSOR_ADDR_WR ATTO_ADDR_WR
#define SENSOR_ADDR_RD ATTO_ADDR_RD

#else /* Default: EV76C541 (Python 480) */

/* The SADDR line allows EV76C541 image sensor to select between two different I2C slave address.
   If the SADDR line is high, enable this #define to allow access to the correct I2C address for the sensor.
 */

/* I2C Slave address for the image sensor. */
#define SENSOR_ADDR_WR 0xB8             /* Slave address used to write sensor registers. */
#define SENSOR_ADDR_RD 0xB9             /* Slave address used to read from sensor registers. */

/* GPIO 20 on FX3 is used as SYNC output -- high during FV */
#define SENSOR_SYNC_GPIO 20

#define SENSOR_REG_CHIP_ID             0
#define SENSOR_REG_SOFT_RESET_ANALOG  10
#define SENSOR_REG_SEQ_GEN_CONFIG    192
#define SENSOR_REG_MULT_TIMER0       199
#define SENSOR_REG_FR_LENGTH0        200
#define SENSOR_REG_EXPOSURE0         201
#define SENSOR_REG_ANALOG_GAIN       204
#define SENSOR_REG_SYNC_CONFIG       206
#define SENSOR_REG_ROI0_X_CONFIG     256
#define SENSOR_REG_ROI0_Y_CONFIG     257
#define SENSOR_REG_ROI1_X_CONFIG     258
#define SENSOR_REG_ROI1_Y_CONFIG     259
#define SENSOR_REG_ROI_CONFIG_LSB0   264
#define SENSOR_REG_ROI_CONFIG_LSB1   265
#define SENSOR_REG_ROI_SELECTION     195

#define SENSOR_A_GAIN_TRANSL_1X   1
#define SENSOR_A_GAIN_TRANSL_2X   2
#define SENSOR_A_GAIN_TRANSL_3_5X 3

#define SENSOR_ROI0 1
#define SENSOR_ROI1 2

extern CyU3PReturnStatus_t
SensorConfigureRoi1(
                      void);

extern CyU3PReturnStatus_t
SensorConfigureRoi2(
                      void);

#endif /* SENSOR_ATTO640D04 */

/* ---------------------------------------------------------------------------
 * Communication over saturation channel
 * These are used by uvc.c regardless of sensor type.
 * -------------------------------------------------------------------------*/
#define SATURATION_RECORD_START 0x01
#define SATURATION_RECORD_END 0x02
#define SATURATION_INIT     0x03
#define SATURATION_FPS5     0x11
#define SATURATION_FPS10    0x12
#define SATURATION_FPS15    0x13
#define SATURATION_FPS20    0x14
#define SATURATION_FPS30    0x15
#define SATURATION_FPS60    0x16
#define SATURATION_ROI0     0x17
#define SATURATION_ROI1     0x18

/* ---------------------------------------------------------------------------
 * Common Sensor API
 *
 * These functions are provided by sensor.c and delegate to the appropriate
 * sensor backend based on the compile-time SENSOR_ATTO640D04 define.
 * -------------------------------------------------------------------------*/

extern CyU3PReturnStatus_t
SensorInit (
            void);

extern CyU3PReturnStatus_t
SensorStart (
    void);

extern CyU3PReturnStatus_t
SensorStop (
    void);

/* Function    : SensorIsOn
   Description : Check to see if the sensor is on
   Parameters  : None
*/
extern CyU3PReturnStatus_t
SensorIsOn (
            CyBool_t *isOn);

extern CyU3PReturnStatus_t
SensorDisable (
        void);

/* Function    : SensorI2CBusTest
   Description : Test whether the sensor is connected on the I2C bus.
   Parameters  : None
 */
extern CyU3PReturnStatus_t
SensorI2CBusTest (
        CyBool_t *connected);

/* Function    : SensorGetGain
   Description : Get the current gain setting from the sensor.
   Parameters  : None
 */
extern CyU3PReturnStatus_t
SensorGetGain (
               uint8_t *translated_gain);

/* Function    : SensorSetGain
   Description : Set the desired gain setting on the sensor.
   Parameters  :
                 gain - Desired gain level.
 */
extern CyU3PReturnStatus_t
SensorSetGain (
        uint8_t new_translated_gain);

extern CyU3PReturnStatus_t
SensorGetRoi (
               uint8_t *translated_roi);

extern CyU3PReturnStatus_t
SensorSetRoi (
        uint8_t new_translated_roi);

#ifdef SENSOR_ATTO640D04
/* ---------------------------------------------------------------------------
 * ATTO640D-04 Additional Sensor API
 *
 * These functions expose ATTO640D-04 specific settings (integration time,
 * image flip) through a common interface so uvc.c can wire them to UVC
 * controls.
 * -------------------------------------------------------------------------*/

/* Function    : SensorGetIntegrationTime
   Description : Get the current integration time from the ATTO640D-04.
   Parameters  : uint16_t *time_val — receives the 16-bit integration time value.
 */
extern CyU3PReturnStatus_t
SensorGetIntegrationTime (
        uint16_t *time_val);

/* Function    : SensorSetIntegrationTime
   Description : Set the integration time on the ATTO640D-04.
   Parameters  : uint16_t time_val — 16-bit integration time value.
 */
extern CyU3PReturnStatus_t
SensorSetIntegrationTime (
        uint16_t time_val);

/* Function    : SensorGetFlip
   Description : Get the current horizontal and vertical flip state.
   Parameters  : CyBool_t *flipH, CyBool_t *flipV
 */
extern CyU3PReturnStatus_t
SensorGetFlip (
        CyBool_t *flipH,
        CyBool_t *flipV);

/* Function    : SensorSetFlip
   Description : Set horizontal and vertical flip.
   Parameters  : CyBool_t flipH, CyBool_t flipV
 */
extern CyU3PReturnStatus_t
SensorSetFlip (
        CyBool_t flipH,
        CyBool_t flipV);

#endif /* SENSOR_ATTO640D04 */

#ifndef SENSOR_ATTO640D04
/* Python 480 specific scaling functions */

/* Function     : SensorScaling_288_288_120fps
   Description  : Configure the EV76C541 sensor for 288x288 120 fps video stream.
   Parameters   : None
 */
extern CyU3PReturnStatus_t
SensorScaling_288_288_120fps (
        void);

/* Function     : SensorScaling_608_608_30fps
   Description  : Configure the EV76C541 sensor for 608x608 30 fps video stream.
   Parameters   : None
 */
extern CyU3PReturnStatus_t
SensorScaling_608_608_30fps (
        void);
#endif /* !SENSOR_ATTO640D04 */

#endif /* _INCLUDED_SENSOR_H_ */

/*[]*/
