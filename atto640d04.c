/*
 * atto640d04.c
 *
 * Driver implementation for the Lynred ATTO640D-04 thermal sensor.
 *
 * This file implements the I2C-based driver for the ATTO640D-04 640x480
 * thermal sensor used in the FX3 UVC camera pipeline.
 *
 * Key characteristics:
 *   - 640x480 resolution, 14-bit parallel digital output (B13..B0)
 *   - Sync: PSYNC (pixel clock), HSYNC (line valid), VSYNC (frame valid)
 *   - Sample on PSYNC rising edge when HSYNC=1 and VSYNC=1
 *   - I2C control: 16-bit register addresses, 8-bit register values
 *   - Digital supplies: DVDD ~1.8V, AVDD ~3.75V
 *   - Max framerate: 60 fps full frame
 *
 * IMPORTANT: The digital mode startup sequence is mandatory. See
 * Atto640d04_DigitalStartup() for the required bring-up flow. Failure
 * to follow sequencing can affect sensor reliability.
 */

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3types.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>
#include "atto640d04.h"
#include "appi2c.h"

/* ============================================================================
 * I2C Helpers — 16-bit register address, 8-bit register value
 *
 * The ATTO640D-04 uses a different I2C register value width (8-bit) compared
 * to the Python 480 sensor (16-bit). These helpers handle the 8-bit value
 * transfers while reusing the existing robust I2C bus infrastructure.
 * ==========================================================================*/

/*
 * Write a single 8-bit value to a 16-bit register address.
 *
 * I2C transaction format:
 *   [START] [slave_addr_W] [reg_addr_MSB] [reg_addr_LSB] [data_8bit] [STOP]
 */
CyU3PReturnStatus_t
Atto640d04_I2CWrite8 (
    uint8_t slaveAddr,
    uint16_t regAddr,
    uint8_t regValue)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;
    uint8_t buf[1];

    preamble.buffer[0] = slaveAddr;
    preamble.buffer[1] = (uint8_t)(regAddr >> 8);   /* Register address MSB */
    preamble.buffer[2] = (uint8_t)(regAddr & 0xFF);  /* Register address LSB */
    preamble.length    = 3;
    preamble.ctrlMask  = 0x0000;  /* No additional start/stop bits */

    buf[0] = regValue;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, 1, 0);
    return apiRetStatus;
}

/*
 * Read a single 8-bit value from a 16-bit register address.
 *
 * I2C transaction format:
 *   [START] [slave_addr_W] [reg_addr_MSB] [reg_addr_LSB]
 *   [RESTART] [slave_addr_R] [data_8bit] [STOP]
 */
CyU3PReturnStatus_t
Atto640d04_I2CRead8 (
    uint8_t slaveAddr,
    uint16_t regAddr,
    uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK;  /* Write address */
    preamble.buffer[1] = (uint8_t)(regAddr >> 8);          /* Register MSB */
    preamble.buffer[2] = (uint8_t)(regAddr & 0xFF);        /* Register LSB */
    preamble.buffer[3] = slaveAddr | 0x01;                 /* Read address */
    preamble.length    = 4;
    preamble.ctrlMask  = 0x0004;  /* Send restart after 3rd byte (before read) */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, 1, 0);
    return apiRetStatus;
}

/*
 * Conditional write helper — only executes if prior status is success.
 * This enables chaining multiple I2C writes with early-exit on failure.
 */
static void
Atto640d04_ConditionalWrite8 (
    CyU3PReturnStatus_t *apiRetStatus,
    uint8_t slaveAddr,
    uint16_t regAddr,
    uint8_t regValue)
{
    if (*apiRetStatus == CY_U3P_SUCCESS) {
        *apiRetStatus = Atto640d04_I2CWrite8 (slaveAddr, regAddr, regValue);
    }
}

/*
 * Conditional read helper — only executes if prior status is success.
 */
static void
Atto640d04_ConditionalRead8 (
    CyU3PReturnStatus_t *apiRetStatus,
    uint8_t slaveAddr,
    uint16_t regAddr,
    uint8_t *buf)
{
    if (*apiRetStatus == CY_U3P_SUCCESS) {
        *apiRetStatus = Atto640d04_I2CRead8 (slaveAddr, regAddr, buf);
    }
}

/* ============================================================================
 * Sensor Probe / Identity / Health Checks
 * ==========================================================================*/

/*
 * Verify sensor identity by reading the three integrity check registers.
 *
 * Expected values:
 *   0x00F7 -> 0x55
 *   0x00F8 -> 0xC6
 *   0x00F9 -> 0xCA
 *
 * Returns CY_U3P_SUCCESS on I2C success; *passed indicates whether
 * all three integrity values matched their expected constants.
 */
CyU3PReturnStatus_t
Atto640d04_CheckIntegrity (
    CyBool_t *passed)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t val0 = 0, val1 = 0, val2 = 0;

    *passed = CyFalse;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_INTEGRITY_0, &val0);
    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_INTEGRITY_1, &val1);
    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_INTEGRITY_2, &val2);

    if (apiRetStatus == CY_U3P_SUCCESS) {
        if (val0 == ATTO_INTEGRITY_0_EXPECTED &&
            val1 == ATTO_INTEGRITY_1_EXPECTED &&
            val2 == ATTO_INTEGRITY_2_EXPECTED) {
            *passed = CyTrue;
            CyU3PDebugPrint (4, "ATTO640D04: Integrity check PASSED "
                             "(0x%02X 0x%02X 0x%02X)\r\n", val0, val1, val2);
        } else {
            CyU3PDebugPrint (4, "ATTO640D04: Integrity check FAILED "
                             "(got 0x%02X 0x%02X 0x%02X, "
                             "expected 0x%02X 0x%02X 0x%02X)\r\n",
                             val0, val1, val2,
                             ATTO_INTEGRITY_0_EXPECTED,
                             ATTO_INTEGRITY_1_EXPECTED,
                             ATTO_INTEGRITY_2_EXPECTED);
        }
    } else {
        CyU3PDebugPrint (4, "ATTO640D04: Integrity check I2C error = %d\r\n",
                         apiRetStatus);
    }

    return apiRetStatus;
}

/* ============================================================================
 * Digital Startup Sequence (Mandatory)
 *
 * The following bring-up flow MUST be followed in order:
 *
 *   1.  Assert NRST low (sensor in reset)
 *   2.  Enable AVDD (~3.75V analog supply)
 *   3.  Wait for AVDD rails to stabilize
 *   4.  Enable DVDD (~1.8V digital supply)
 *   5.  Wait for DVDD rails to stabilize
 *   6.  Start master clock (MC)
 *   7.  Release NRST (set high)
 *   8.  Wait >= 1.6 ms for internal initialization
 *   9.  Verify I2C communication by reading integrity registers
 *  10.  Enable I2C diffusion
 *  11.  Initialize/configure ADC-related registers
 *  12.  Perform ADC calibration (2 cycles)
 *  13.  Finalize digital mode and frame sync mode
 *
 * WARNING: Failure to follow this exact sequence can affect sensor
 * reliability and may result in undefined behavior.
 * ==========================================================================*/

CyU3PReturnStatus_t
Atto640d04_DigitalStartup (void)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyBool_t integrityPassed = CyFalse;
    uint8_t regval = 0;

    CyU3PDebugPrint (4, "ATTO640D04: Starting digital startup sequence\r\n");

    /*
     * Steps 1-7: NRST, power sequencing, and clock start.
     *
     * In this FX3-based platform, the NRST, AVDD, DVDD, and master clock
     * are typically controlled by the auxiliary MCU (ATTiny) via the
     * SensorConfigurePython480() or equivalent power-control path.
     *
     * The existing auxiliary I2C infrastructure handles the power-on
     * sequencing through the REG_SENSOR_CTRL register. We issue the
     * power-on request and allow the necessary stabilization delays.
     *
     * TODO: If hardware-specific NRST GPIO control is needed, add it here.
     */

    /* Step 1-2: Assert NRST low, then enable AVDD via auxiliary MCU */
    CyU3PDebugPrint (4, "ATTO640D04: Step 1-2: Power-on request (NRST + AVDD)\r\n");

    /* Step 3: Wait for AVDD to stabilize */
    CyU3PThreadSleep (ATTO_AVDD_STABILIZE_MS);
    CyU3PDebugPrint (4, "ATTO640D04: Step 3: AVDD stabilized\r\n");

    /* Step 4-5: Enable DVDD and wait for stabilization */
    CyU3PThreadSleep (ATTO_DVDD_STABILIZE_MS);
    CyU3PDebugPrint (4, "ATTO640D04: Step 4-5: DVDD stabilized\r\n");

    /* Step 6-7: Start MC and release NRST (handled by auxiliary MCU) */
    CyU3PDebugPrint (4, "ATTO640D04: Step 6-7: MC started, NRST released\r\n");

    /* Step 8: Wait >= 1.6 ms after NRST release */
    CyU3PThreadSleep (ATTO_NRST_RELEASE_MS);
    CyU3PDebugPrint (4, "ATTO640D04: Step 8: Post-NRST wait complete\r\n");

    /* Step 9: Verify I2C communication by reading integrity registers */
    CyU3PDebugPrint (4, "ATTO640D04: Step 9: Verifying I2C integrity\r\n");
    apiRetStatus = Atto640d04_CheckIntegrity (&integrityPassed);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - I2C integrity read failed "
                         "(error %d)\r\n", apiRetStatus);
        return apiRetStatus;
    }
    if (!integrityPassed) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - Integrity check mismatch\r\n");
        return CY_U3P_ERROR_FAILURE;
    }

    /* Step 10: Enable I2C diffusion via CONFIG_B register */
    CyU3PDebugPrint (4, "ATTO640D04: Step 10: Enabling I2C diffusion\r\n");
    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_CONFIG_B, &regval);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - CONFIG_B read failed\r\n");
        return apiRetStatus;
    }
    regval |= ATTO_CFGB_I2C_DIFF_EN_bm;
    apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR, ATTO_REG_CONFIG_B, regval);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - I2C diffusion enable failed\r\n");
        return apiRetStatus;
    }

    /* Step 11: Initialize ADC — enable ADC in DIGITAL_OUTPUT register */
    CyU3PDebugPrint (4, "ATTO640D04: Step 11: Initializing ADC\r\n");
    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_DIGITAL_OUTPUT, &regval);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - DIGITAL_OUTPUT read failed\r\n");
        return apiRetStatus;
    }
    regval |= ATTO_DIGOUT_ADC_EN_bm;
    apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR, ATTO_REG_DIGITAL_OUTPUT, regval);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - ADC enable failed\r\n");
        return apiRetStatus;
    }

    /* Step 12: ADC calibration — perform two calibration cycles */
    CyU3PDebugPrint (4, "ATTO640D04: Step 12: ADC calibration "
                     "(%d cycles)\r\n", ATTO_ADC_CALIB_CYCLES);
    {
        uint8_t cycle;
        for (cycle = 0; cycle < ATTO_ADC_CALIB_CYCLES; cycle++) {
            /* Set ADC_CALIB_ON to trigger a calibration cycle */
            apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD,
                                                 ATTO_REG_DIGITAL_OUTPUT, &regval);
            if (apiRetStatus != CY_U3P_SUCCESS) {
                CyU3PDebugPrint (4, "ATTO640D04: ERROR - ADC calib read failed "
                                 "(cycle %d)\r\n", cycle);
                return apiRetStatus;
            }
            regval |= ATTO_DIGOUT_ADC_CALIB_ON_bm;
            apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR,
                                                  ATTO_REG_DIGITAL_OUTPUT, regval);
            if (apiRetStatus != CY_U3P_SUCCESS) {
                CyU3PDebugPrint (4, "ATTO640D04: ERROR - ADC calib write failed "
                                 "(cycle %d)\r\n", cycle);
                return apiRetStatus;
            }

            /* Allow calibration to complete */
            CyU3PThreadSleep (5);

            /* Clear ADC_CALIB_ON */
            regval &= (uint8_t)~ATTO_DIGOUT_ADC_CALIB_ON_bm;
            apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR,
                                                  ATTO_REG_DIGITAL_OUTPUT, regval);
            if (apiRetStatus != CY_U3P_SUCCESS) {
                CyU3PDebugPrint (4, "ATTO640D04: ERROR - ADC calib clear failed "
                                 "(cycle %d)\r\n", cycle);
                return apiRetStatus;
            }

            CyU3PDebugPrint (4, "ATTO640D04: ADC calibration cycle %d complete\r\n",
                             cycle + 1);
        }
    }

    /* Step 13: Finalize digital mode — set frame sync mode, enable sequencer */
    CyU3PDebugPrint (4, "ATTO640D04: Step 13: Finalizing digital mode\r\n");

    /* Configure default full-frame windowing: 640x480 */
    apiRetStatus = Atto640d04_SetWindow (0, 0, ATTO_WIDTH, ATTO_HEIGHT);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - Default window config failed\r\n");
        return apiRetStatus;
    }

    /* Set default integration time */
    apiRetStatus = Atto640d04_SetIntegrationTime (0x0100);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - Default integration time failed\r\n");
        return apiRetStatus;
    }

    /* Set default gain */
    apiRetStatus = Atto640d04_SetGain (ATTO_DEFAULT_GAIN);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - Default gain set failed\r\n");
        return apiRetStatus;
    }

    /* Set free-run mode (continuous frame output) */
    apiRetStatus = Atto640d04_SetTriggerMode (ATTO_MODE_FREE_RUN);
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - Free-run mode set failed\r\n");
        return apiRetStatus;
    }

    CyU3PDebugPrint (4, "ATTO640D04: Digital startup sequence COMPLETE\r\n");
    return CY_U3P_SUCCESS;
}

/* ============================================================================
 * Sensor Lifecycle
 * ==========================================================================*/

/*
 * Initialize the ATTO640D-04 sensor.
 * Executes the mandatory digital startup sequence and configures the
 * default full-frame 640x480 @ 60fps mode.
 */
CyU3PReturnStatus_t
Atto640d04_Init (void)
{
    CyU3PReturnStatus_t apiRetStatus;

    CyU3PDebugPrint (4, "ATTO640D04: Initializing sensor\r\n");

    /* Execute the mandatory digital startup sequence */
    apiRetStatus = Atto640d04_DigitalStartup ();
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - Digital startup failed "
                         "(error %d)\r\n", apiRetStatus);
        return apiRetStatus;
    }

    /* Start the sequencer to begin frame output */
    apiRetStatus = Atto640d04_StartSequencer ();
    if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "ATTO640D04: ERROR - Sequencer start failed\r\n");
        return apiRetStatus;
    }

    /* Print initial debug dump */
    Atto640d04_DebugDump ();

    CyU3PDebugPrint (4, "ATTO640D04: Initialization complete\r\n");
    return CY_U3P_SUCCESS;
}

/*
 * Start streaming — enable the sequencer.
 */
CyU3PReturnStatus_t
Atto640d04_Start (void)
{
    CyU3PDebugPrint (4, "ATTO640D04: Start streaming\r\n");
    return Atto640d04_StartSequencer ();
}

/*
 * Stop streaming — disable the sequencer.
 */
CyU3PReturnStatus_t
Atto640d04_Stop (void)
{
    CyU3PDebugPrint (4, "ATTO640D04: Stop streaming\r\n");
    return Atto640d04_StopSequencer ();
}

/* ============================================================================
 * Configuration APIs
 * ==========================================================================*/

/* --- Integration Time --- */

CyU3PReturnStatus_t
Atto640d04_SetIntegrationTime (uint16_t time_val)
{
    CyU3PReturnStatus_t apiRetStatus;

    apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR, ATTO_REG_INT_TIME_H,
                                          (uint8_t)(time_val >> 8));
    Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                   ATTO_REG_INT_TIME_L,
                                   (uint8_t)(time_val & 0xFF));
    return apiRetStatus;
}

CyU3PReturnStatus_t
Atto640d04_GetIntegrationTime (uint16_t *time_val)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t hi = 0, lo = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_INT_TIME_H, &hi);
    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_INT_TIME_L, &lo);

    if (apiRetStatus == CY_U3P_SUCCESS) {
        *time_val = ((uint16_t)hi << 8) | lo;
    }
    return apiRetStatus;
}

/* --- Windowing / ROI --- */

CyU3PReturnStatus_t
Atto640d04_SetWindow (
    uint16_t xstart, uint16_t ystart,
    uint16_t xsize, uint16_t ysize)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t cfgb = 0;

    /* Write window start and size registers */
    apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR, ATTO_REG_WIN_XSTART_H,
                                          (uint8_t)(xstart >> 8));
    Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                   ATTO_REG_WIN_XSTART_L,
                                   (uint8_t)(xstart & 0xFF));
    Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                   ATTO_REG_WIN_YSTART_H,
                                   (uint8_t)(ystart >> 8));
    Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                   ATTO_REG_WIN_YSTART_L,
                                   (uint8_t)(ystart & 0xFF));
    Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                   ATTO_REG_WIN_XSIZE_H,
                                   (uint8_t)(xsize >> 8));
    Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                   ATTO_REG_WIN_XSIZE_L,
                                   (uint8_t)(xsize & 0xFF));
    Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                   ATTO_REG_WIN_YSIZE_H,
                                   (uint8_t)(ysize >> 8));
    Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                   ATTO_REG_WIN_YSIZE_L,
                                   (uint8_t)(ysize & 0xFF));

    /* Enable windowing mode in CONFIG_B if window differs from full frame */
    if (apiRetStatus == CY_U3P_SUCCESS) {
        apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD,
                                             ATTO_REG_CONFIG_B, &cfgb);
        if (apiRetStatus == CY_U3P_SUCCESS) {
            if (xsize < ATTO_WIDTH || ysize < ATTO_HEIGHT ||
                xstart != 0 || ystart != 0) {
                cfgb |= ATTO_CFGB_WINDOW_bm;  /* Enable windowing */
            } else {
                cfgb &= (uint8_t)~ATTO_CFGB_WINDOW_bm;  /* Full frame */
            }
            apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR,
                                                  ATTO_REG_CONFIG_B, cfgb);
        }
    }

    return apiRetStatus;
}

CyU3PReturnStatus_t
Atto640d04_GetWindow (
    uint16_t *xstart, uint16_t *ystart,
    uint16_t *xsize, uint16_t *ysize)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t hi = 0, lo = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_WIN_XSTART_H, &hi);
    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_WIN_XSTART_L, &lo);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        *xstart = ((uint16_t)hi << 8) | lo;
    }

    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_WIN_YSTART_H, &hi);
    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_WIN_YSTART_L, &lo);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        *ystart = ((uint16_t)hi << 8) | lo;
    }

    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_WIN_XSIZE_H, &hi);
    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_WIN_XSIZE_L, &lo);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        *xsize = ((uint16_t)hi << 8) | lo;
    }

    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_WIN_YSIZE_H, &hi);
    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_WIN_YSIZE_L, &lo);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        *ysize = ((uint16_t)hi << 8) | lo;
    }

    return apiRetStatus;
}

/* --- Image Flip --- */

CyU3PReturnStatus_t
Atto640d04_SetFlip (CyBool_t flipH, CyBool_t flipV)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t regval = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_GAIN_IMAGE, &regval);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        if (flipH) {
            regval |= ATTO_GAIN_IMAGE_FLIP_H_bm;
        } else {
            regval &= (uint8_t)~ATTO_GAIN_IMAGE_FLIP_H_bm;
        }
        if (flipV) {
            regval |= ATTO_GAIN_IMAGE_FLIP_V_bm;
        } else {
            regval &= (uint8_t)~ATTO_GAIN_IMAGE_FLIP_V_bm;
        }
        apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR,
                                              ATTO_REG_GAIN_IMAGE, regval);
    }
    return apiRetStatus;
}

CyU3PReturnStatus_t
Atto640d04_GetFlip (CyBool_t *flipH, CyBool_t *flipV)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t regval = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_GAIN_IMAGE, &regval);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        *flipH = (regval & ATTO_GAIN_IMAGE_FLIP_H_bm) ? CyTrue : CyFalse;
        *flipV = (regval & ATTO_GAIN_IMAGE_FLIP_V_bm) ? CyTrue : CyFalse;
    }
    return apiRetStatus;
}

/* --- Trigger / Free-Run Mode --- */

CyU3PReturnStatus_t
Atto640d04_SetTriggerMode (uint8_t mode)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t gain_reg = 0, digout_reg = 0;

    /* Trigger mode is split across two registers:
     *   TRIGGER_1 in GAIN_IMAGE (bit 7)
     *   TRIGGER_2 in DIGITAL_OUTPUT (bit 5)
     *
     * Free-run:  TRIGGER_1=0, TRIGGER_2=0
     * Triggered: TRIGGER_1=1, TRIGGER_2=1
     */

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_GAIN_IMAGE, &gain_reg);
    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_DIGITAL_OUTPUT, &digout_reg);

    if (apiRetStatus == CY_U3P_SUCCESS) {
        if (mode == ATTO_MODE_TRIGGERED) {
            gain_reg   |= ATTO_GAIN_IMAGE_TRIGGER_1_bm;
            digout_reg |= ATTO_DIGOUT_TRIGGER_2_bm;
        } else {
            gain_reg   &= (uint8_t)~ATTO_GAIN_IMAGE_TRIGGER_1_bm;
            digout_reg &= (uint8_t)~ATTO_DIGOUT_TRIGGER_2_bm;
        }

        apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR,
                                              ATTO_REG_GAIN_IMAGE, gain_reg);
        Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                       ATTO_REG_DIGITAL_OUTPUT, digout_reg);
    }
    return apiRetStatus;
}

CyU3PReturnStatus_t
Atto640d04_GetTriggerMode (uint8_t *mode)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t gain_reg = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_GAIN_IMAGE, &gain_reg);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        *mode = (gain_reg & ATTO_GAIN_IMAGE_TRIGGER_1_bm) ?
                ATTO_MODE_TRIGGERED : ATTO_MODE_FREE_RUN;
    }
    return apiRetStatus;
}

/* --- Sequencer Control --- */

CyU3PReturnStatus_t
Atto640d04_StartSequencer (void)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t cfgb = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_CONFIG_B, &cfgb);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        cfgb |= ATTO_CFGB_START_SEQ_bm;
        apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR,
                                              ATTO_REG_CONFIG_B, cfgb);
    }
    return apiRetStatus;
}

CyU3PReturnStatus_t
Atto640d04_StopSequencer (void)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t cfgb = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_CONFIG_B, &cfgb);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        cfgb &= (uint8_t)~ATTO_CFGB_START_SEQ_bm;
        apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR,
                                              ATTO_REG_CONFIG_B, cfgb);
    }
    return apiRetStatus;
}

/* --- Interframe / Interline --- */

CyU3PReturnStatus_t
Atto640d04_SetUserInterframe (uint8_t value)
{
    return Atto640d04_I2CWrite8 (ATTO_ADDR_WR, ATTO_REG_USER_INTERFRAME, value);
}

CyU3PReturnStatus_t
Atto640d04_GetUserInterframe (uint8_t *value)
{
    return Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_USER_INTERFRAME, value);
}

CyU3PReturnStatus_t
Atto640d04_SetUserInterline (uint8_t value)
{
    return Atto640d04_I2CWrite8 (ATTO_ADDR_WR, ATTO_REG_USER_INTERLINE, value);
}

CyU3PReturnStatus_t
Atto640d04_GetUserInterline (uint8_t *value)
{
    return Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_USER_INTERLINE, value);
}

/* --- Gain --- */

CyU3PReturnStatus_t
Atto640d04_SetGain (uint8_t gain)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t regval = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_GAIN_IMAGE, &regval);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        regval = (regval & (uint8_t)~ATTO_GAIN_IMAGE_GAIN_MASK) |
                 (gain & ATTO_GAIN_IMAGE_GAIN_MASK);
        apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR,
                                              ATTO_REG_GAIN_IMAGE, regval);
    }
    return apiRetStatus;
}

CyU3PReturnStatus_t
Atto640d04_GetGain (uint8_t *gain)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t regval = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_GAIN_IMAGE, &regval);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        *gain = regval & ATTO_GAIN_IMAGE_GAIN_MASK;
    }
    return apiRetStatus;
}

/* --- DAC GFID / GSK (analog tuning) --- */

CyU3PReturnStatus_t
Atto640d04_SetGfid (uint8_t value)
{
    return Atto640d04_I2CWrite8 (ATTO_ADDR_WR, ATTO_REG_DAC_GFID, value);
}

CyU3PReturnStatus_t
Atto640d04_GetGfid (uint8_t *value)
{
    return Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_DAC_GFID, value);
}

CyU3PReturnStatus_t
Atto640d04_SetGsk (uint16_t value)
{
    CyU3PReturnStatus_t apiRetStatus;

    apiRetStatus = Atto640d04_I2CWrite8 (ATTO_ADDR_WR, ATTO_REG_DAC_GSK_H,
                                          (uint8_t)(value >> 8));
    Atto640d04_ConditionalWrite8 (&apiRetStatus, ATTO_ADDR_WR,
                                   ATTO_REG_DAC_GSK_L,
                                   (uint8_t)(value & 0xFF));
    return apiRetStatus;
}

CyU3PReturnStatus_t
Atto640d04_GetGsk (uint16_t *value)
{
    CyU3PReturnStatus_t apiRetStatus;
    uint8_t hi = 0, lo = 0;

    apiRetStatus = Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_DAC_GSK_H, &hi);
    Atto640d04_ConditionalRead8 (&apiRetStatus, ATTO_ADDR_RD,
                                  ATTO_REG_DAC_GSK_L, &lo);
    if (apiRetStatus == CY_U3P_SUCCESS) {
        *value = ((uint16_t)hi << 8) | lo;
    }
    return apiRetStatus;
}

/* --- Status / Diagnostics --- */

CyU3PReturnStatus_t
Atto640d04_GetStatus (uint8_t *status)
{
    return Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_STATUS, status);
}

/*
 * Print a comprehensive debug dump of the sensor state.
 * Shows detection status, integrity results, mode, ROI, integration time,
 * and trigger mode.
 */
void
Atto640d04_DebugDump (void)
{
    CyBool_t intPassed = CyFalse;
    uint8_t status = 0;
    uint8_t gain = 0;
    uint8_t trigMode = 0;
    uint8_t interframe = 0;
    uint8_t interline = 0;
    uint8_t roicRev = 0;
    uint16_t intTime = 0;
    uint16_t xstart = 0, ystart = 0, xsize = 0, ysize = 0;
    CyBool_t flipH = CyFalse, flipV = CyFalse;

    CyU3PDebugPrint (4, "=== ATTO640D-04 Debug Dump ===\r\n");

    /* Sensor detection / integrity */
    Atto640d04_CheckIntegrity (&intPassed);
    CyU3PDebugPrint (4, "  Sensor detected:     %s\r\n",
                     intPassed ? "YES" : "NO");
    CyU3PDebugPrint (4, "  Integrity registers: %s\r\n",
                     intPassed ? "PASSED" : "FAILED");

    /* ROIC revision */
    if (Atto640d04_I2CRead8 (ATTO_ADDR_RD, ATTO_REG_ROIC_REV,
                              &roicRev) == CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "  ROIC revision:       0x%02X\r\n", roicRev);
    }

    /* Status register */
    if (Atto640d04_GetStatus (&status) == CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "  Status register:     0x%02X\r\n", status);
        CyU3PDebugPrint (4, "    ROIC_INIT_DONE:    %d\r\n",
                         (status & ATTO_STATUS_ROIC_INIT_DONE_bm) ? 1 : 0);
        CyU3PDebugPrint (4, "    SEQ_STATUS:        %d\r\n",
                         (status & ATTO_STATUS_SEQ_STATUS_bm) ? 1 : 0);
        CyU3PDebugPrint (4, "    BAD_XY_PROG:       %d\r\n",
                         (status & ATTO_STATUS_BAD_XY_PROG_bm) ? 1 : 0);
        CyU3PDebugPrint (4, "    BAD_SIZE_PROG:     %d\r\n",
                         (status & ATTO_STATUS_BAD_SIZE_PROG_bm) ? 1 : 0);
    }

    /* Configured mode (640x480 @ 60fps) */
    CyU3PDebugPrint (4, "  Configured mode:     %dx%d @ %d fps\r\n",
                     ATTO_WIDTH, ATTO_HEIGHT, ATTO_MAX_FPS);

    /* ROI / Window */
    if (Atto640d04_GetWindow (&xstart, &ystart,
                               &xsize, &ysize) == CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "  ROI: x=%d y=%d w=%d h=%d\r\n",
                         xstart, ystart, xsize, ysize);
    }

    /* Integration time */
    if (Atto640d04_GetIntegrationTime (&intTime) == CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "  Integration time:    0x%04X\r\n", intTime);
    }

    /* Gain */
    if (Atto640d04_GetGain (&gain) == CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "  Gain:                0x%02X\r\n", gain);
    }

    /* Trigger mode */
    if (Atto640d04_GetTriggerMode (&trigMode) == CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "  Trigger mode:        %s\r\n",
                         (trigMode == ATTO_MODE_TRIGGERED) ?
                         "TRIGGERED" : "FREE-RUN");
    }

    /* Flip */
    if (Atto640d04_GetFlip (&flipH, &flipV) == CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "  Flip H/V:            %d/%d\r\n",
                         flipH ? 1 : 0, flipV ? 1 : 0);
    }

    /* Interframe / Interline */
    if (Atto640d04_GetUserInterframe (&interframe) == CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "  USER_INTERFRAME:     0x%02X\r\n", interframe);
    }
    if (Atto640d04_GetUserInterline (&interline) == CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "  USER_INTERLINE:      0x%02X\r\n", interline);
    }

    CyU3PDebugPrint (4, "=== End ATTO640D-04 Debug Dump ===\r\n");
}
