/*
 * atto640d04.h
 *
 * Driver header for the Lynred ATTO640D-04 thermal sensor.
 *
 * Resolution:   640x480
 * Output:       14-bit parallel digital video on B13..B0
 * Sync signals: PSYNC (pixel clock), HSYNC (line valid), VSYNC (frame valid)
 * Sample rule:  sample pixel data on PSYNC rising edge when HSYNC and VSYNC are high
 * Control bus:  I2C (16-bit register address, 8-bit register value)
 * Digital supplies: DVDD ~1.8V, AVDD ~3.75V
 * Max framerate: 60 fps full frame
 *
 * IMPORTANT: The digital mode startup sequence is mandatory and must be
 * followed exactly. Failure to follow sequencing can affect sensor reliability.
 * See Atto640d04DigitalStartup() for the required bring-up flow.
 */

#ifndef ATTO640D04_H_
#define ATTO640D04_H_

#include <cyu3types.h>

/* ---------------------------------------------------------------------------
 * I2C Addressing
 * The ATTO640D-04 I2C slave address (7-bit: 0x5C, 8-bit write/read format).
 * Adjust if hardware SADDR pin selects a different address.
 * -------------------------------------------------------------------------*/
#define ATTO_ADDR_WR        0xB8    /* I2C write address (matches existing wiring) */
#define ATTO_ADDR_RD        0xB9    /* I2C read address */

/* ---------------------------------------------------------------------------
 * Sensor Dimensions & Timing
 * -------------------------------------------------------------------------*/
#define ATTO_WIDTH          640
#define ATTO_HEIGHT         480
#define ATTO_BPP            14      /* Bits per pixel (parallel bus B13..B0) */
#define ATTO_MAX_FPS        60      /* Maximum frame rate at full resolution */

/*
 * Default full-frame mode: 640x480 @ 60 fps.
 *
 * Clocking assumptions:
 *   - Parallel data bus: B13..B0 (14 bits)
 *   - PSYNC: pixel clock
 *   - HSYNC: line valid
 *   - VSYNC: frame valid
 *   - Sampling: on PSYNC rising edge with HSYNC=1 and VSYNC=1
 */

/* ---------------------------------------------------------------------------
 * Register Map (subset used by this driver)
 *
 * All registers use 16-bit addresses and 8-bit values.
 * -------------------------------------------------------------------------*/

/* Gain / image control register (0x0040)
 * Bits:
 *   [7]   TRIGGER_1    - Trigger control bit 1
 *   [6]   FLIP_H       - Horizontal image flip
 *   [5]   FLIP_V       - Vertical image flip
 *   [4:0] GAIN_IMAGE   - Image gain value
 */
#define ATTO_REG_GAIN_IMAGE         0x0040

/* Digital output control register (0x0041)
 * Bits:
 *   [7]   ADC_CALIB_ON - ADC calibration enable
 *   [6]   ADC_EN       - ADC enable
 *   [5]   TRIGGER_2    - Trigger control bit 2
 *   [4:0] Reserved
 */
#define ATTO_REG_DIGITAL_OUTPUT     0x0041

/* Windowing / ROI registers (0x0043..0x004A) */
#define ATTO_REG_WIN_XSTART_H      0x0043   /* Window X start, high byte */
#define ATTO_REG_WIN_XSTART_L      0x0044   /* Window X start, low byte */
#define ATTO_REG_WIN_YSTART_H      0x0045   /* Window Y start, high byte */
#define ATTO_REG_WIN_YSTART_L      0x0046   /* Window Y start, low byte */
#define ATTO_REG_WIN_XSIZE_H       0x0047   /* Window X size, high byte */
#define ATTO_REG_WIN_XSIZE_L       0x0048   /* Window X size, low byte */
#define ATTO_REG_WIN_YSIZE_H       0x0049   /* Window Y size, high byte */
#define ATTO_REG_WIN_YSIZE_L       0x004A   /* Window Y size, low byte */

/* DAC registers for analog tuning */
#define ATTO_REG_DAC_GFID           0x004B   /* DAC GFID value */
#define ATTO_REG_DAC_GSK_H          0x004C   /* DAC GSK, high byte */
#define ATTO_REG_DAC_GSK_L          0x004D   /* DAC GSK, low byte */

/* Integration time registers (0x004F..0x0050) */
#define ATTO_REG_INT_TIME_H         0x004F   /* Integration time, high byte */
#define ATTO_REG_INT_TIME_L         0x0050   /* Integration time, low byte */

/* User interframe register (0x0056) */
#define ATTO_REG_USER_INTERFRAME    0x0056   /* User-defined interframe period */

/* Configuration B register (0x005C)
 * Bits:
 *   [7]   I2C_DIFF_EN  - Enable I2C diffusion
 *   [3]   WINDOW       - Enable windowing mode
 *   [0]   START_SEQ    - Start sequencer
 */
#define ATTO_REG_CONFIG_B           0x005C

/* User interline register (0x0062) */
#define ATTO_REG_USER_INTERLINE     0x0062   /* User-defined interline period */

/* Status register (0x0063)
 * Bits (read-only):
 *   [3] BAD_XY_PROG    - Bad X/Y programming error
 *   [2] BAD_SIZE_PROG  - Bad size programming error
 *   [1] ROIC_INIT_DONE - ROIC initialization complete
 *   [0] SEQ_STATUS     - Sequencer running status
 */
#define ATTO_REG_STATUS             0x0063

/* ROIC revision register */
#define ATTO_REG_ROIC_REV           0x00F6   /* ROIC silicon revision */

/* Integrity check registers (read-only) */
#define ATTO_REG_INTEGRITY_0        0x00F7   /* Expected value: 0x55 */
#define ATTO_REG_INTEGRITY_1        0x00F8   /* Expected value: 0xC6 */
#define ATTO_REG_INTEGRITY_2        0x00F9   /* Expected value: 0xCA */

/* Expected integrity check values */
#define ATTO_INTEGRITY_0_EXPECTED   0x55
#define ATTO_INTEGRITY_1_EXPECTED   0xC6
#define ATTO_INTEGRITY_2_EXPECTED   0xCA

/* ---------------------------------------------------------------------------
 * Bit Definitions for GAIN_IMAGE (0x0040)
 * -------------------------------------------------------------------------*/
#define ATTO_GAIN_IMAGE_TRIGGER_1_bp    7
#define ATTO_GAIN_IMAGE_TRIGGER_1_bm    (1 << ATTO_GAIN_IMAGE_TRIGGER_1_bp)
#define ATTO_GAIN_IMAGE_FLIP_H_bp       6
#define ATTO_GAIN_IMAGE_FLIP_H_bm       (1 << ATTO_GAIN_IMAGE_FLIP_H_bp)
#define ATTO_GAIN_IMAGE_FLIP_V_bp       5
#define ATTO_GAIN_IMAGE_FLIP_V_bm       (1 << ATTO_GAIN_IMAGE_FLIP_V_bp)
#define ATTO_GAIN_IMAGE_GAIN_MASK       0x1F

/* ---------------------------------------------------------------------------
 * Bit Definitions for DIGITAL_OUTPUT (0x0041)
 * -------------------------------------------------------------------------*/
#define ATTO_DIGOUT_ADC_CALIB_ON_bp     7
#define ATTO_DIGOUT_ADC_CALIB_ON_bm     (1 << ATTO_DIGOUT_ADC_CALIB_ON_bp)
#define ATTO_DIGOUT_ADC_EN_bp           6
#define ATTO_DIGOUT_ADC_EN_bm           (1 << ATTO_DIGOUT_ADC_EN_bp)
#define ATTO_DIGOUT_TRIGGER_2_bp        5
#define ATTO_DIGOUT_TRIGGER_2_bm        (1 << ATTO_DIGOUT_TRIGGER_2_bp)

/* ---------------------------------------------------------------------------
 * Bit Definitions for CONFIG_B (0x005C)
 * -------------------------------------------------------------------------*/
#define ATTO_CFGB_I2C_DIFF_EN_bp        7
#define ATTO_CFGB_I2C_DIFF_EN_bm        (1 << ATTO_CFGB_I2C_DIFF_EN_bp)
#define ATTO_CFGB_WINDOW_bp             3
#define ATTO_CFGB_WINDOW_bm             (1 << ATTO_CFGB_WINDOW_bp)
#define ATTO_CFGB_START_SEQ_bp          0
#define ATTO_CFGB_START_SEQ_bm          (1 << ATTO_CFGB_START_SEQ_bp)

/* ---------------------------------------------------------------------------
 * Bit Definitions for STATUS (0x0063)
 * -------------------------------------------------------------------------*/
#define ATTO_STATUS_BAD_XY_PROG_bp      3
#define ATTO_STATUS_BAD_XY_PROG_bm      (1 << ATTO_STATUS_BAD_XY_PROG_bp)
#define ATTO_STATUS_BAD_SIZE_PROG_bp    2
#define ATTO_STATUS_BAD_SIZE_PROG_bm    (1 << ATTO_STATUS_BAD_SIZE_PROG_bp)
#define ATTO_STATUS_ROIC_INIT_DONE_bp   1
#define ATTO_STATUS_ROIC_INIT_DONE_bm   (1 << ATTO_STATUS_ROIC_INIT_DONE_bp)
#define ATTO_STATUS_SEQ_STATUS_bp       0
#define ATTO_STATUS_SEQ_STATUS_bm       (1 << ATTO_STATUS_SEQ_STATUS_bp)

/* ---------------------------------------------------------------------------
 * Trigger / mode constants
 * -------------------------------------------------------------------------*/
#define ATTO_MODE_FREE_RUN              0   /* Free-running (continuous) mode */
#define ATTO_MODE_TRIGGERED             1   /* External trigger mode */

/* Default gain value */
#define ATTO_DEFAULT_GAIN               0x00

/* ADC calibration cycle count */
#define ATTO_ADC_CALIB_CYCLES           2

/* Power-on timing constants (milliseconds) */
#define ATTO_AVDD_STABILIZE_MS          10  /* Wait for AVDD rails to stabilize */
#define ATTO_DVDD_STABILIZE_MS          10  /* Wait for DVDD rails to stabilize */
#define ATTO_NRST_RELEASE_MS            2   /* 1.6 ms minimum, round up to 2 ms */

/* ---------------------------------------------------------------------------
 * 14-bit to YUY2 Packing
 *
 * The ATTO640D-04 outputs 14-bit monochrome pixels on bus B13..B0.
 * For UVC transport we pack into YUY2 (YUYV):
 *   - Each 14-bit pixel value is right-shifted by 6 to yield an 8-bit Y value.
 *   - U and V are set to 0x80 (neutral chrominance) for grayscale.
 *   - Two consecutive pixels produce one YUY2 macropixel: [Y0, U, Y1, V]
 *
 * This provides a simple, lossless-enough mapping for thermal preview.
 * -------------------------------------------------------------------------*/
#define ATTO_THERMAL_CHROMA_NEUTRAL     0x80

/* ---------------------------------------------------------------------------
 * Public API — ATTO640D-04 Sensor Driver
 * -------------------------------------------------------------------------*/

/*
 * I2C Helpers (16-bit register address, 8-bit register value)
 */
extern CyU3PReturnStatus_t
Atto640d04_I2CWrite8 (uint8_t slaveAddr, uint16_t regAddr, uint8_t regValue);

extern CyU3PReturnStatus_t
Atto640d04_I2CRead8 (uint8_t slaveAddr, uint16_t regAddr, uint8_t *buf);

/*
 * Sensor probe / identity / health checks
 */
extern CyU3PReturnStatus_t
Atto640d04_CheckIntegrity (CyBool_t *passed);

/*
 * Digital startup sequence (mandatory — see implementation for full flow)
 */
extern CyU3PReturnStatus_t
Atto640d04_DigitalStartup (void);

/*
 * Sensor lifecycle
 */
extern CyU3PReturnStatus_t
Atto640d04_Init (void);

extern CyU3PReturnStatus_t
Atto640d04_Start (void);

extern CyU3PReturnStatus_t
Atto640d04_Stop (void);

/*
 * Integration time
 */
extern CyU3PReturnStatus_t
Atto640d04_SetIntegrationTime (uint16_t time_val);

extern CyU3PReturnStatus_t
Atto640d04_GetIntegrationTime (uint16_t *time_val);

/*
 * Windowing / ROI (Region of Interest)
 */
extern CyU3PReturnStatus_t
Atto640d04_SetWindow (uint16_t xstart, uint16_t ystart,
                      uint16_t xsize, uint16_t ysize);

extern CyU3PReturnStatus_t
Atto640d04_GetWindow (uint16_t *xstart, uint16_t *ystart,
                      uint16_t *xsize, uint16_t *ysize);

/*
 * Image flip
 */
extern CyU3PReturnStatus_t
Atto640d04_SetFlip (CyBool_t flipH, CyBool_t flipV);

extern CyU3PReturnStatus_t
Atto640d04_GetFlip (CyBool_t *flipH, CyBool_t *flipV);

/*
 * Trigger / free-run mode
 */
extern CyU3PReturnStatus_t
Atto640d04_SetTriggerMode (uint8_t mode);

extern CyU3PReturnStatus_t
Atto640d04_GetTriggerMode (uint8_t *mode);

/*
 * Sequencer control
 */
extern CyU3PReturnStatus_t
Atto640d04_StartSequencer (void);

extern CyU3PReturnStatus_t
Atto640d04_StopSequencer (void);

/*
 * Interframe / Interline timing
 */
extern CyU3PReturnStatus_t
Atto640d04_SetUserInterframe (uint8_t value);

extern CyU3PReturnStatus_t
Atto640d04_GetUserInterframe (uint8_t *value);

extern CyU3PReturnStatus_t
Atto640d04_SetUserInterline (uint8_t value);

extern CyU3PReturnStatus_t
Atto640d04_GetUserInterline (uint8_t *value);

/*
 * Gain control
 */
extern CyU3PReturnStatus_t
Atto640d04_SetGain (uint8_t gain);

extern CyU3PReturnStatus_t
Atto640d04_GetGain (uint8_t *gain);

/*
 * DAC GFID / GSK (analog tuning)
 */
extern CyU3PReturnStatus_t
Atto640d04_SetGfid (uint8_t value);

extern CyU3PReturnStatus_t
Atto640d04_GetGfid (uint8_t *value);

extern CyU3PReturnStatus_t
Atto640d04_SetGsk (uint16_t value);

extern CyU3PReturnStatus_t
Atto640d04_GetGsk (uint16_t *value);

/*
 * Status / diagnostics
 */
extern CyU3PReturnStatus_t
Atto640d04_GetStatus (uint8_t *status);

extern void
Atto640d04_DebugDump (void);

#endif /* ATTO640D04_H_ */
