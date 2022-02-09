#ifndef __SX126X_H__
#define __SX126X_H__
#include <device-tree.h>
#include <drv_gpio.h>
#include <lora-radio.h>
#include <rtdevice.h>

typedef enum {
  RADIO_RAMP_10_US   = 0x00,
  RADIO_RAMP_20_US   = 0x01,
  RADIO_RAMP_40_US   = 0x02,
  RADIO_RAMP_80_US   = 0x03,
  RADIO_RAMP_200_US  = 0x04,
  RADIO_RAMP_800_US  = 0x05,
  RADIO_RAMP_1700_US = 0x06,
  RADIO_RAMP_3400_US = 0x07,
} sx126x_ramp_times_t;

typedef struct sx126x_s {
  sx126x_device_t*    device;
  rt_base_t           gpio_busy;
  rt_base_t           gpio_dio1;
  rt_base_t           gpio_reset;
  rt_bool_t           image_calibrated;
  sx126x_ramp_times_t ramp_times;
  rt_time_t           rx_timeout;
} sx126x_t;

typedef enum {
  RADIO_GET_STATUS                = 0xC0,
  RADIO_WRITE_REGISTER            = 0x0D,
  RADIO_READ_REGISTER             = 0x1D,
  RADIO_WRITE_BUFFER              = 0x0E,
  RADIO_READ_BUFFER               = 0x1E,
  RADIO_SET_SLEEP                 = 0x84,
  RADIO_SET_STANDBY               = 0x80,
  RADIO_SET_FS                    = 0xC1,
  RADIO_SET_TX                    = 0x83,
  RADIO_SET_RX                    = 0x82,
  RADIO_SET_RXDUTYCYCLE           = 0x94,
  RADIO_SET_CAD                   = 0xC5,
  RADIO_SET_TXCONTINUOUSWAVE      = 0xD1,
  RADIO_SET_TXCONTINUOUSPREAMBLE  = 0xD2,
  RADIO_SET_PACKETTYPE            = 0x8A,
  RADIO_GET_PACKETTYPE            = 0x11,
  RADIO_SET_RFFREQUENCY           = 0x86,
  RADIO_SET_TXPARAMS              = 0x8E,
  RADIO_SET_PACONFIG              = 0x95,
  RADIO_SET_CADPARAMS             = 0x88,
  RADIO_SET_BUFFERBASEADDRESS     = 0x8F,
  RADIO_SET_MODULATIONPARAMS      = 0x8B,
  RADIO_SET_PACKETPARAMS          = 0x8C,
  RADIO_GET_RXBUFFERSTATUS        = 0x13,
  RADIO_GET_PACKETSTATUS          = 0x14,
  RADIO_GET_RSSIINST              = 0x15,
  RADIO_GET_STATS                 = 0x10,
  RADIO_RESET_STATS               = 0x00,
  RADIO_CFG_DIOIRQ                = 0x08,
  RADIO_GET_IRQSTATUS             = 0x12,
  RADIO_CLR_IRQSTATUS             = 0x02,
  RADIO_CALIBRATE                 = 0x89,
  RADIO_CALIBRATEIMAGE            = 0x98,
  RADIO_SET_REGULATORMODE         = 0x96,
  RADIO_GET_ERROR                 = 0x17,
  RADIO_CLR_ERROR                 = 0x07,
  RADIO_SET_TCXOMODE              = 0x97,
  RADIO_SET_TXFALLBACKMODE        = 0x93,
  RADIO_SET_RFSWITCHMODE          = 0x9D,
  RADIO_SET_STOPRXTIMERONPREAMBLE = 0x9F,
  RADIO_SET_LORASYMBTIMEOUT       = 0xA0,
} sx126x_radio_commands_t;

typedef enum {
  MOD_SHAPING_OFF     = 0x00,
  MOD_SHAPING_G_BT_03 = 0x08,
  MOD_SHAPING_G_BT_05 = 0x09,
  MOD_SHAPING_G_BT_07 = 0x0A,
  MOD_SHAPING_G_BT_1  = 0x0B,
} sx126x_radio_mod_shapings_t;

typedef enum {
  RADIO_CRC_OFF          = 0x01,
  RADIO_CRC_1_BYTES      = 0x00,
  RADIO_CRC_2_BYTES      = 0x02,
  RADIO_CRC_1_BYTES_INV  = 0x04,
  RADIO_CRC_2_BYTES_INV  = 0x06,
  RADIO_CRC_2_BYTES_IBM  = 0xF1,
  RADIO_CRC_2_BYTES_CCIT = 0xF2,
} sx126x_radio_crc_types_t;

/*!
 * \brief The address of the register holding the first byte defining the CRC seed
 *
 */
#define REG_LR_CRCSEEDBASEADDR 0x06BC

/*!
 * \brief The address of the register holding the first byte defining the CRC polynomial
 */
#define REG_LR_CRCPOLYBASEADDR 0x06BE

/*!
 * \brief The address of the register holding the first byte defining the whitening seed
 */
#define REG_LR_WHITSEEDBASEADDR_MSB 0x06B8
#define REG_LR_WHITSEEDBASEADDR_LSB 0x06B9

/*!
 * \brief The address of the register holding the packet configuration
 */
#define REG_LR_PACKETPARAMS 0x0704

/*!
 * \brief The address of the register holding the payload size
 */
#define REG_LR_PAYLOADLENGTH 0x0702

/*!
 * \brief The address of the register holding the re-calculated number of symbols
 */
#define REG_LR_SYNCH_TIMEOUT 0x0706

/*!
 * \brief The addresses of the registers holding SyncWords values
 */
#define REG_LR_SYNCWORDBASEADDRESS 0x06C0

/*!
 * \brief The addresses of the register holding LoRa Modem SyncWord value
 */
#define REG_LR_SYNCWORD 0x0740

/*!
 * The address of the register giving a 32-bit random number
 */
#define RANDOM_NUMBER_GENERATORBASEADDR 0x0819

/*!
 * The address of the register used to disable the LNA
 */
#define REG_ANA_LNA 0x08E2

/*!
 * The address of the register used to disable the mixer
 */
#define REG_ANA_MIXER 0x08E5

/*!
 * The address of the register holding RX Gain value (0x94: power saving, 0x96: rx boosted)
 */
#define REG_RX_GAIN 0x08AC

/*!
 * Change the value on the device internal trimming capacitor
 */
#define REG_XTA_TRIM 0x0911

/*!
 * Set the current max value in the over current protection
 */
#define REG_OCP 0x08E7

#define REG_TX_CLAMP_CONFIG 0x8D8

#endif  // __SX126x_H__
