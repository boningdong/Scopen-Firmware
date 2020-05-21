#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "main.h"

#define IQS266_ADDRESS 0x44

// NOTE Platform specific defines
#define GPIO_RDY_GROUP GPIOA
#define GPIO_RDY_PIN GPIO_PIN_10
#define GPIO_RDY_PIN_NUM 10

typedef union {
  uint8_t flagByte;
	struct{
		uint8_t lpActive:1;
		uint8_t ignoreGH:1;
		uint8_t inAti:1;
		uint8_t inNPSeg:1;
		uint8_t reserved:1;
		uint8_t atiError:1;
		uint8_t ltaBlocked:1;
		uint8_t shReset:1;
	};
} system_flags_t;

typedef union{
	uint8_t flagByte;
	struct{
		uint8_t ch0:1;
		uint8_t ch1:1;
		uint8_t ch2:1;
		uint8_t ch3:1;
		uint8_t ch4:1;
		uint8_t ch5:1;
		uint8_t ch6:1;
		uint8_t reserved:1;
	}touch;
	struct {
		uint8_t reserved:7;
		uint8_t ch0:1;
	} prox;
} channel_status_t;

typedef union{
	uint8_t  flagByte;
	struct {
		uint8_t prox:1;
		uint8_t touch:1;
		uint8_t trackpad:1;
		uint8_t ati:1;
		uint8_t tap:1;
		uint8_t swipe:1;
		uint8_t reserved:1;
		uint8_t lowPower:1;
	};
} event_flags_t;

typedef union{
	uint8_t  flagByte;
	struct {
		uint8_t active:1;
		uint8_t tap:1;
		uint8_t swipeUp:1;
		uint8_t swipeDown:1;
		uint8_t swipeLeft:1;
		uint8_t swipeRight:1;
		uint8_t segment0:1;
		uint8_t segment1:1;
	};
} trackpad_flags_t;

typedef enum {
	CH0, CH1, CH2, CH3, CH4, CH5, CH6, ALL
} channel_t;

void user_button_init();


/*******************************************************************************
                      			IQS266 - API Functions
*******************************************************************************/
void iqs266_init();
bool iqs266_request_communication();
bool iqs266_wait_ready();
void iqs266_clear_reset(bool restart);
void iqs266_auto_tune(bool restart);
void iqs266_event_mode(bool restart);
void iqs266_enable_events(uint8_t event_bits, bool restart);
void iqs266_enable_all_events(bool restart);
void iqs266_disable_events(uint8_t event_bits, bool restart);
void iqs266_disable_all_events(bool restart);
void iqs266_enable_channel(channel_t channel, bool restart);
void iqs266_disable_channel(channel_t channel, bool restart);
void iqs266_set_touch_sensitivity(channel_t channel, uint8_t sensitivity, bool restart);
void iqs266_set_proximate_sensitivity(uint8_t sensitivity, bool restart);

uint8_t iqs266_read_system_flags(bool restart);
uint8_t iqs266_read_trackpad_flags(bool restart);
uint8_t iqs266_read_events(bool restart);
uint8_t iqs266_read_gestures(bool restart);
uint8_t iqs266_read_approximate(bool restart);
uint8_t iqs266_read_touch(bool restart);
uint8_t iqs266_read_channels(bool restart);

/*******************************************************************************
                      IQS266 - Registers & Memory Map
*******************************************************************************/

// Device Information
#define DEVICE_INFO         0x00    // Byte 0: PRODUCT_NUM    Byte 1: VERSION_NUM
// Flags
#define SYS_FLAGS           0x01    // Byte 0: SYSFLAGS       Byte 1: EVENTS
// Trackpad Data
#define TP_FLAGS      			0x02    // Byte 0: TP_FLAGS
#define COORDINATES         0x03    // Byte 0: X_CURR         Byte 1: Y_CURR
// Channel Prox and Touch Flags
#define CHANNEL_BYTES       0x04    // Byte 0: PROX_CHANNEL   Byte 1: TOUCH_CHANNELS
// Channel Counts Bytes
#define ACF_CH0             0x05    // Byte 0: ACF_CH0_LOW    Byte 1: ACF_CH0_HIGH
#define ACF_CH1             0x06    // Byte 0: ACF_CH1_LOW    Byte 1: ACF_CH1_HIGH
#define ACF_CH2             0x07    // Byte 0: ACF_CH2_LOW    Byte 1: ACF_CH2_HIGH
#define ACF_CH3             0x08    // Byte 0: ACF_CH3_LOW    Byte 1: ACF_CH3_HIGH
#define ACF_CH4             0x09    // Byte 0: ACF_CH4_LOW    Byte 1: ACF_CH4_HIGH
#define ACF_CH5             0x0A    // Byte 0: ACF_CH5_LOW    Byte 1: ACF_CH5_HIGH
#define ACF_CH6             0x0B    // Byte 0: ACF_CH6_LOW    Byte 1: ACF_CH6_HIGH
// Channel LTA Bytes
#define LTA_CH0             0x0C    // Byte 0: LTA_CH0_LOW    Byte 1: LTA_CH0_HIGH
#define LTA_CH1             0x0D    // Byte 0: LTA_CH1_LOW    Byte 1: LTA_CH1_HIGH
#define LTA_CH2             0x0E    // Byte 0: LTA_CH2_LOW    Byte 1: LTA_CH2_HIGH
#define LTA_CH3             0x0F    // Byte 0: LTA_CH3_LOW    Byte 1: LTA_CH3_HIGH
#define LTA_CH4             0x10    // Byte 0: LTA_CH4_LOW    Byte 1: LTA_CH4_HIGH
#define LTA_CH5             0x11    // Byte 0: LTA_CH5_LOW    Byte 1: LTA_CH5_HIGH
#define LTA_CH6             0x12    // Byte 0: LTA_CH6_LOW    Byte 1: LTA_CH6_HIGH
// Channel Delta Bytes
#define DELTA_CH0           0x13    // Byte 0: DELTA_CH0_LOW  Byte 1: DELTA_CH0_HIGH
#define DELTA_CH1           0x14    // Byte 0: DELTA_CH1_LOW  Byte 1: DELTA_CH1_HIGH
#define DELTA_CH2           0x15    // Byte 0: DELTA_CH2_LOW  Byte 1: DELTA_CH2_HIGH
#define DELTA_CH3           0x16    // Byte 0: DELTA_CH3_LOW  Byte 1: DELTA_CH3_HIGH
#define DELTA_CH4           0x17    // Byte 0: DELTA_CH4_LOW  Byte 1: DELTA_CH4_HIGH
#define DELTA_CH5           0x18    // Byte 0: DELTA_CH5_LOW  Byte 1: DELTA_CH5_HIGH
#define DELTA_CH6           0x19    // Byte 0: DELTA_CH6_LOW  Byte 1: DELTA_CH6_HIGH
// Prox Settings Bytes
#define PROXSETTINGS_01     0x80    // Byte 0: PROX_SETTINGS_0  Byte 1: PROX_SETTINGS_1
#define PROXSETTINGS_23     0x81    // Byte 0: PROX_SETTINGS_2  Byte 1: PROX_SETTINGS_3
// Event Mask and Timeout Bytes
#define EVENT_MASK          0x82    // Byte 0: EVENT_MASK     Byte 1: ZOOM_TIMEOUT
#define TIMEOUT_PERIODS     0x83    // Byte 0: HALT_TIMEOUT   Byte 1: RDY_TIMEOUT
// Report Rate Bytes
#define REPORT_RATES        0x84    // Byte 0: NM_PERIOD      Byte 1: LP_PERIOD
// Threshold Bytes
#define THRESHOLDS_CH0      0x85    // Byte 0: PROX_THR_CH0   Byte 1: TOUCH_THR_CH0
#define TOUCH_THR_CH1_CH2   0x86    // Byte 0: TOUCH_THR_CH1  Byte 1: TOUCH_THR_CH2
#define TOUCH_THR_CH3_CH4   0x87    // Byte 0: TOUCH_THR_CH3  Byte 1: TOUCH_THR_CH4 
#define TOUCH_THR_CH5_CH6   0x88    // Byte 0: TOUCH_THR_CH5  Byte 1: TOUCH_THR_CH6
// Channel Settings Bytes
#define ATI_TARGETS         0x89    // Byte 0: ATI_TARGET_CH1_CH6 Byte 1: ATI_TARGET_CH0
#define CHANNEL_SETTINGS     0x8A    // Byte 0: BASE_VALUES        Byte 1: ACTIVE_CHANNELS
// Tap Gesture Settings
#define TAP_SETTINGS        0x8B    // Byte 0: TAP_TIMER    Byte 1: TAP_THRESHOLD
// Swipe Gesture Settings
#define SWIPE_SETTINGS      0x8C    // Byte 0 : SWIPE_TIMER   Byte 1: SWIPE_THRESHOLD


/*******************************************************************************
                      IQS266 - Registers Bits
*******************************************************************************/
// Event Byte Bits. The EVENTS byte and the EVENT_MASK byte use the same bit positions.
#define PROX_EVENT_BIT      0x01
#define TOUCH_EVENT_BIT     0x02
#define TP_EVENT_BIT        0x04
#define ATI_EVENT_BIT       0x08
#define TAP_EVENT_BIT       0x10
#define SWIPE_EVENT_BIT     0x20
#define LP_EVENT_BIT		0x80
// Gesture Byte Bits
#define TP_BIT              0x01
#define TAP_BIT             0x02
#define SWIPE_UP_BIT        0x04
#define SWIPE_DOWN_BIT      0x08
#define SWIPE_LEFT_BIT      0x10
#define SWIPE_RIGHT_BIT     0x20
// Prox Bits
#define CH0_PROX_BIT		0x01
// Touch Bits
#define CH0_TOUCH_BIT		0x01
#define CH1_TOUCH_BIT		0x02
#define CH2_TOUCH_BIT		0x04
#define CH3_TOUCH_BIT		0x08
#define CH4_TOUCH_BIT		0x10
#define CH5_TOUCH_BIT		0x20
#define CH6_TOUCH_BIT		0x40
// Utility Bits
#define SHOW_RESET_BIT		0x80
#define ACK_RESET_BIT		0x80
#define REDO_ATI_BIT        0x10
#define EVENT_MODE_BIT      0x40

#endif