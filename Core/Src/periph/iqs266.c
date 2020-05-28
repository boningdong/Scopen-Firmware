#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "periph/iqs266.h"
#include "periph/i2c.h"
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_exti.h>

system_flags_t system_flags;
trackpad_flags_t trackpad_flags;
event_flags_t events;
channel_status_t channels;

GPIO_TypeDef *ready_ports = GPIO_RDY_GROUP;
EXTI_TypeDef *exti = EXTI;

// Static function declarations
static void _set_ready_output();
static void _set_ready_input();
static void _toggle_ready();
static void _read_random_bytes(uint16_t address, uint8_t buffer[], uint16_t size, bool restart);
static void _write_random_bytes(uint16_t address, uint8_t buffer[], uint16_t size, bool restart);
static void _set_event_mask_bits(uint8_t bit_mask, bool restart);
static void _clear_event_mask_bits(uint8_t mask_bit, bool restart);


/*******************************************************************************
                      			IQS266 - API Functions
*******************************************************************************/

/**
 * @brief Initialize the RDY pin interrupt for receiving events.
 * 
 */
void iqs266_init_ready_interrupt() {
  LL_EXTI_InitTypeDef exti_config = {0};
  exti_config.Line_0_31 = GPIO_RDY_PIN;
  exti_config.LineCommand = ENABLE;
  exti_config.Mode = LL_EXTI_MODE_IT;
  exti_config.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&exti_config);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 8, 8);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  // NVIC_SetPriority(EXTI15_10_IRQn, 0);
  // NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Enable the event interrupt.
 * @note  This function is platform specific.
 * 
 */
void iqs266_disable_rdy_interrupt() {
  LL_EXTI_DisableFallingTrig_0_31(GPIO_RDY_PIN);
}

/**
 * @brief Disable the event interrupt.
 * @note  This function is platform specific.
 */
void iqs266_enable_rdy_interrupt() {
  LL_EXTI_EnableFallingTrig_0_31(GPIO_RDY_PIN);
}

/**
 * @brief Clear RESET bit by writing it to 0.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done.
 * @notes If a reset has occurred the device settings should be reloaded using the begin function.
 * 		    After new device settings have been reloaded this method should be used to clear the
 *  		  reset bit.
 */
void iqs266_clear_reset(bool restart) {
  uint8_t buffer[2] = {0};
  _read_random_bytes(PROXSETTINGS_23, buffer, 2, true);
  buffer[1] |= ACK_RESET_BIT;
  _write_random_bytes(PROXSETTINGS_23, buffer, 2, restart);
}

/**
 * @brief A function turns off the communication watch dog timer. This is useful if the chip restarts due
 *        to the long communication time.
 * 
 * @param restart 
 */
void iqs266_disable_wdt(bool restart) {
  uint8_t buffer[2] = {0};
  _read_random_bytes(PROXSETTINGS_01, buffer, 2, true);
  buffer[1] |= WDT_DISABLE_BIT;
  _write_random_bytes(PROXSETTINGS_01, buffer, 2, restart);
}

/**
 * @brief A method which sets the REDO_ATI_BIT in order to force the IQS266 device to run the 
 *        Automatic Tuning Implementation (ATI) routine.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done. 
 */
void iqs266_auto_tune(bool restart) {
    uint8_t buffer[1];
  // Read the current PROX_SETTINGS_0 value in order not to alter any existing settings.
  _read_random_bytes(PROXSETTINGS_01, buffer, 1, true);
  // Mask the settings with the REDO_ATI_BIT.
  buffer[0] |= REDO_ATI_BIT;  // This is the bit required to start an ATI routine.
  // Write the new byte to the required device.
  _write_random_bytes(PROXSETTINGS_01, buffer, 1, restart);
}

/**
 * @brief Set the iqs266 into event mode.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done.  
 */
void iqs266_event_mode(bool restart) {
  uint8_t buffer[2];
  // First read the bytes at the memory address so that they can be preserved.
  _read_random_bytes(PROXSETTINGS_01, buffer, 2, true);
  // Clear the EVENT_MODE_BIT in PROX_SETTINGS_1
  buffer[1] |= EVENT_MODE_BIT;
  // Write the bytes back to the device
  _write_random_bytes(PROXSETTINGS_01, buffer, 2, restart);
}

/**
 * @brief A function the reads the device info.
 * 
 * @param restart 
 * @return uint8_t An unsigned char containing the product number. Should be 0x4a (74).
 */
uint8_t iqs266_read_device_info(bool restart) {
  uint8_t buffer[2];
  _read_random_bytes(DEVICE_INFO, buffer, 2, true);
  return buffer[0];
}

/**
 * @brief A method which reads and returns the IQS266 system flags.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done. 
 * @return uint8_t an unsigned char containing the system flags byte.
 */
uint8_t iqs266_read_system_flags(bool restart) {
  uint8_t buffer[1];
	_read_random_bytes(SYS_FLAGS, buffer, 1, restart);
	return buffer[0];
}

/**
 * @brief A method which reads and returns the byte which holds the trackpad flags.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done. 
 * @return uint8_t Returns the byte which contains the trackpad flags.
 */
uint8_t iqs266_read_trackpad_flags(bool restart) {
  uint8_t buffer[1]; 
  _read_random_bytes(TP_FLAGS, buffer, 1, restart);
  return buffer[0];
}

/**
 * @brief A methods which reads and returns the byte which holds the event flags.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done.  
 * @return uint8_t Returns the byte which contains the event flags.
 */
uint8_t iqs266_read_events(bool restart) {
  uint8_t buffer[2];
  // Events flags are in the second byte.
  _read_random_bytes(SYS_FLAGS, buffer, 2, restart);
  return buffer[1];
}

/**
 * @brief An alias of iqs266_read_trackpad_flags, which reads and returns the byte which holds the gesture flags.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done.  
 * @return uint8_t Returns the byte which contains the gesture flags.
 */
uint8_t iqs266_read_gestures(bool restart) {
  uint8_t buffer[1]; 
  _read_random_bytes(TP_FLAGS, buffer, 1, restart);
  return buffer[0];
}

/**
 * @brief A method which reads and returns the PROX CHANNEL byte which holds the prox flag for channel 0, this is a bit uselss as it only contains
			    one bit which is set when the prox threshold on channel 0 is breached. It would probably be better to use the Prox Event.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done.   
 * @return uint8_t Returns the byte which contains the prox flag for channel 0.
 */
uint8_t iqs266_read_approximate(bool restart) {
  uint8_t buffer[1];
  _read_random_bytes(CHANNEL_BYTES, buffer, 1, restart);
  return buffer[0];
}

/**
 * @brief A method which reads and returns the TOUCH CHANNELS byte which holds the touch flag for all the channels.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done. 
 * @return uint8_t Returns the byte which contains the touch flags for all the channels.
 */
uint8_t iqs266_read_touch(bool restart) {
  uint8_t buffer[2];
  _read_random_bytes(CHANNEL_BYTES, buffer, 2, restart);
  // The touch info is in the second byte.
  return buffer[1];
}

/**
 * @brief A methodd which reads and returns the byte which holds the X-Coordinates of the touch taking place on the trackpad.
 * 
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done. 
 * @return uint8_t  Returns the byte which contains the X-Cordinates flags.
 */
uint8_t iqs266_read_x(bool restart) {
  uint8_t buffer[1];
  // Read the byte using the readRandomByte method.
  _read_random_bytes(COORDINATES, buffer, 1, restart);
  //  Return the byte, the X_CURR byte is the first byte at the COORDINATES address.
  return buffer[0];
}

/**
 * @brief A methodd which reads and returns the byte which holds the Y-Coordinates of the touch taking place on the trackpad.
 * 
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done. 
 * @return uint8_t  Returns the byte which contains the Y-Cordinates flags.
 */
uint8_t iqs266_read_y(bool restart) {
  uint8_t buffer[2]; // The temporary address which will hold the bytes at the COORDINATES address.
  // Read the bytes using the readRandomByte method.
  _read_random_bytes(COORDINATES, buffer, 2, restart);
  // Return the byte, the Y_CURR byte is the second byte at the COORDINATES address.
  return buffer[1];
}

/**
 * @brief A method which read both the touch byte and the prox byte and assigns the channel status to the channels union.
 * 
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done. 
 * @return uint8_t returns the info of all channels. It's the combination of touch and approximate.
 */
uint8_t iqs266_read_channels(bool restart) {
  uint8_t buffer[2];
  _read_random_bytes(CHANNEL_BYTES, buffer, 2, restart);
  // Combine approximate and touch into the same byte.
  buffer[1] |= (buffer[0]<<7);
  return buffer[1];
}

/**
 * @brief A method which reads and returns the counts value of a specific channel
 * 
 * @param channel     The enumerator which specifes which channel's counts to return.
 * @param restart     Determines if the RESART or STOP bit is sent after the communication is done.
 * @return uint16_t   Returns a 16 bit unsigned integer containing the counts value of the channel.
 */
uint16_t iqs266_read_channel_counts(channel_t channel, bool restart) {
  uint8_t buffer[2];      
  uint8_t start_address = ACF_CH0;  // The address of the first channel's counts bytes, this address is adjusted according to the enum passed.
  uint8_t count_low = 0;            // Temporary storage for the counts low byte.
  uint8_t count_high = 0;           // Temporary storage for the counts high byte.
  uint16_t count_value = 0;         // The 16bit return value.

  // Adjust the address according to the channel enum which was passed.
  start_address += channel;
  // Read the bytes with the readRandomBytes method
  _read_random_bytes(start_address, buffer, 2, restart);
  // Construct the 16bit return value.
  count_low = buffer[0];
  count_high = buffer[1];
  count_value = (uint16_t)(count_low);
  count_value |= (uint16_t)(count_high<<8);
  // Return the counts value.
  return count_value;
}

/**
 * @brief A method which reads and returns the Delta value of a specific channel.
 *        For ch0         delta = LTA - ACF
 *        For ch1 - ch6:  delta = ACF - REF
 * @param channel     The enumerator which specifes which channel's Delta value to return.
 * @param restart     Determines if the RESART or STOP bit is sent after the communication is done.
 * @return uint16_t   Returns a 16 bit unsigned integer containing the Delta value of the channel.
 */
uint16_t iqs266_read_channel_delta(channel_t channel, bool restart) {
  uint8_t buffer[2];         
  uint8_t start_address = DELTA_CH0;  // The address of the first channel's Delta bytes, this address is adjusted according to the enum passed.
  uint8_t delta_low = 0;                // Temporary storage for the Delta low byte.
  uint8_t delta_high = 0;               // Temporary storage for the Delta high byte.
  uint16_t delta_value = 0;             // The 16bit return value.

  // Adjust the address according to the channel enum which was passed.
  start_address += channel;
  // Read the bytes with the readRandomBytes method
  _read_random_bytes(start_address, buffer, 2, restart);
  // Construct the 16bit return value.
  delta_low = buffer[0];
  delta_high = buffer[1];
  delta_value = (uint16_t)(delta_low);
  delta_value |= (uint16_t)(delta_high<<8);
  // Return the Delta value.
  return delta_value;  
}

/**
 * @brief A method which reads and returns the Delta value of a specific channel.
 * 
 * @param channel     The enumerator which specifes which channel's Delta value to return.
 * @param restart     Determines if the RESART or STOP bit is sent after the communication is done.
 * @return uint16_t   Returns a 16 bit unsigned integer containing the Delta value of the channel.
 */
uint16_t iqs266_read_channel_lta(channel_t channel, bool restart) {
  uint8_t buffer[2];       // A temporary array which will hold all the channel LTA low and high bytes.
  uint8_t read_address = LTA_CH0;  // The address of the first channel's LTA bytes, this address is adjusted according to the enum passed.
  uint8_t lta_low = 0;             // Temporary storage for the LTA low byte.
  uint8_t lta_high = 0;            // Temporary storage for the LTA high byte.
  uint16_t lta_value = 0;         // The 16bit return value.

  // Adjust the address according to the channel enum which was passed.
  read_address += channel;
  // Read the bytes with the readRandomBytes method
  _read_random_bytes(read_address, buffer, 2, restart);
  // Construct the 16bit return value.
  lta_low = buffer[0];
  lta_high = buffer[1];
  lta_value = (uint16_t)(lta_low);
  lta_value |= (uint16_t)(lta_high<<8);
  // Return the LTA value.
  return lta_value;
}


/**
 * @brief A function to set the Halt Timeout Time in seconds. This is done by setting.
 * 
 * @param timeout   The time in seconds to which the timeout must be set, ranging from 1 to 127, 0 disables halt timeout.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_halt_timeout(uint8_t timeout, bool restart) {
  // Map and constrain the haltTime argument.
  if(timeout > 127)
    timeout = 254;
  else if(timeout < 1)
    timeout = 1;
  else
    timeout += timeout;

  // Now write the new halt timeout time value to the HALT_TIMEOUT register which is byte 0 at the TIMEOUT_PERIODS address.
  _write_random_bytes(TIMEOUT_PERIODS, &timeout, 1, restart);
}

/**
 * @brief Function to disable halt timeout. 
 * 
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_disable_halt_timeout(bool restart) {
  // A value of 255 disables halt timeout.
  uint8_t timeout = 255;

  // Now write timeout value (255) to the HALT_TIMEOUT register which is byte 0 at the TIMEOUT_PERIODS address.
  _write_random_bytes(TIMEOUT_PERIODS, &timeout, 1, restart);
}

/**
 * @brief A method which sets the low power mode report rate in increments of 16 milliseconds. 0 disables LP mode.
 * 
 * @param value     A value between 0 and 7. Normal power segment rate = 2^(segmentRate).
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_np_segment_rate(uint8_t value, bool restart) {
  uint8_t buffer[1];

	// Constrain the segment rate to a maximum value of 7..
	if (value > 7)
		value = 7;

	// First read the PROXSETTINGS2 register in order not to change any settings.
	_read_random_bytes(PROXSETTINGS_23, buffer, 1, true);
	// First clear the lower 3 bytes of PROXSETTINGS2
	buffer[0] &= 0xf8; // Preserve all bytes except lower 3.
	// Set the lower 3 bytes to the segment rate value.
	buffer[0] |= value;
	// Write the new value to the PROXSETTINGS2 register.
	_write_random_bytes(PROXSETTINGS_23, buffer, 1, restart);
}

/**
 * @brief A method which sets the low power mode report rate in increments of 16 milliseconds. 0 disables LP mode.
 * 
 * @param value     A value between 0 and 255 which sets the low power report rate in milliseconds.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_report_rate_lp(uint8_t value, bool restart) {
  uint8_t buffer[2];

	// First read the 1st byte at the REPORT_RATES address in order not to change the report rate for normal mode.
	_read_random_bytes(REPORT_RATES, buffer, 1,  true);
	// Set the report rate for the low power mode.
	buffer[1] = value;
	// Now write all the bytes back to the REPORT_RATES address.
	_write_random_bytes(REPORT_RATES, buffer, 2, restart);
}

/**
 * @brief A method which sets the normal mode report rate in milliseconds.
 * 
 * @param value     A value between 0 and 255 which sets the normal mode report rate in milliseconds.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_report_rate_nm(uint8_t value, bool restart) {
	// No need to read any information since we are just over-writing a value into the first byte at the address.
	// Write the byte into the REPORT_RATES address.
	_write_random_bytes(REPORT_RATES, &value, 1, restart);
}

/**
 * @brief A method that will let the iqs266 clear the event flags after each communication window.
 * 
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_clear_tp_flags(bool restart) {
  uint8_t buffer[2];
  _read_random_bytes(PROXSETTINGS_23, buffer, 2, true);
  buffer[0] != TP_CLEAR_BIT;
  _write_random_bytes(PROXSETTINGS_23, buffer, 2, restart);
}

/**
 * @brief A method which sets the zoom timeout time. This is the time which the device will stay in normal power mode with no activity.
 * 			  When the timeout value is reached the device will enter low power mode.
 * 
 * @param timeout   A value between 0 and 255 which specifies the timeout time in 500ms increments.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_zoom_timeout(uint8_t timeout, bool restart) {
  uint8_t buffer[2];	// Temporary array which holds the bytes to be transferred.

	// First read the 1st byte at the EVENT_MASK address in order not to change the event mask.
	_read_random_bytes(EVENT_MASK, buffer, 1, true);
	// Set the ZOOM_TIMEOUT register, byte 1, to the required value.
	buffer[1] = timeout;
	// Now write all the bytes back to the EVENT_MASK address.
	_write_random_bytes(EVENT_MASK, buffer, 2, restart);
}

/**
 * @brief A method which sets the ATI target for the touch channels (CH_1 to CH_6). This value is multiplied by 8 for the actual counts target.
 * 
 * @param value     An 8 bit value which specifies the touch ATI target in increments of 8. Eg. Value = 80, TATI Target = 80 x 8 = 640.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_touch_ati_target(uint8_t value, bool restart) {
  // No need to read any information since we are just over-writing a value into the first byte at the address.
	// Write the byte into the ATI_TARGETS address.
	_write_random_bytes(ATI_TARGETS, &value, 1, restart);
}

/**
 * @brief A method which sets the base value for channels 1 to 6 which are the touch channels.
 * 
 * @param value     A value between 0 and 15, both inclusive, which sets the base value in increments of 16 ranging from 74 to 314.
 * 						      See the IQS266 datasheet to make the correct selection.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_touch_base_value(uint8_t value, bool restart) {
	uint8_t buffer[1];

	// Constrain the base value to a maximum value of 15.
	if (value > 15)
		value = 15;
	// Shift the base value to the upper 4 bytes.
	value = (value << 4);

	// First read the byte in order not to change the base value for the touch channels.
	_read_random_bytes(CHANNEL_SETTINGS, buffer, 1, true);
	// Set the base value for the prox channel.
	buffer[0] &= 0x0F; // Preserve the lower 4 bytes, clear the upper 4 bytes.
	buffer[0] |= value;
	// Now write the new value to the register address.
	_write_random_bytes(CHANNEL_SETTINGS, buffer, 1, restart);
}

/**
 * @brief A method which sets the touch threshold on a specified channel or all channels, thereby adjusting the 
 *        sensitivity of the channel. The Prox threshold for channel 0 is not altered at all.
 * 
 * @param channel       A channel having a type channel_t. Use ALL to set the sensitivity for all channels.
 * @param sensitivity   A value from 0 to 255 which specifies the touch threshold for the channels, a smaller value means more sensitive.
 * @param restart       Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_touch_sensitivity(channel_t channel, uint8_t sensitivity, bool restart) {
  uint8_t buffer[8];
  uint8_t address;
  switch (channel) {
    case CH0: case ALL:
      address = THRESHOLDS_CH0; break;
    case CH1: case CH2:
      address = TOUCH_THR_CH1_CH2; break;
    case CH3: case CH4:
      address = TOUCH_THR_CH3_CH4; break;
    case CH5: case CH6:
      address = TOUCH_THR_CH5_CH6; break;
    default:
      return;
  }

  // Read bytes into the buffer to maintain the other channels sensitivity
  _read_random_bytes(address, buffer, 2, true);
  switch (channel)
  {
  case CH0: case CH2: case CH4: case CH6:
    buffer[1] = sensitivity; break;
  case CH1: case CH3: case CH5:
    buffer[0] = sensitivity; break;
  case ALL:
    for (int i = 1; i < 8; i++) {
      buffer[i] = sensitivity;
    }
  }
  // Write new sensitivities to the registers.
  if (channel == ALL)
    _write_random_bytes(address, buffer, 8, restart);
  else
    _write_random_bytes(address, buffer, 2, restart);

}

/**
 * @brief A method which sets the proximity threshold, thereby adjusting the sensitivity of the proximity channel which is channel 0.
 * 
 * @param sensitivity   A value from 0 to 255 which specifies the touch threshold for the channels, a smaller value means more sensitive.
 * @param restart       Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_proximate_sensitivity(uint8_t sensitivity, bool restart) {
  uint8_t buffer[1];
  buffer[0] = sensitivity;
  _write_random_bytes(THRESHOLDS_CH0, buffer, 1, restart);
}

/**
 * @brief A method which sets the maximum time allowed for a Tap gesture. The gesture must occur in less time than this setting to be valid.
 * 
 * @param timeout   A value from 0 to 255 which specifies the Tap timer limit in 2 millsecond increments.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_tap_timeout(uint8_t timeout, bool restart) {
  uint8_t buffer[1];

  // Adjust the time value to a value between 0 and 255.
  if(timeout >= 255)
    timeout = 255;
  else if (timeout <=0)
    timeout = 0;

  // Store the time limit to the byte which will be transferred.
  buffer[0] = timeout;
  // Write the time limit to the correct address.
  _write_random_bytes(TAP_SETTINGS, buffer, 1, restart);
}

/**
 * @brief A method which sets the maximum coordinate length of a square area on the trackpad for Tap gesture. Exceeding this length will not recognise a Tap gesture.
 * 
 * @param threshold   A value from 0 to 255 which specifies the coordinate length of the maximum square area.
 * @param restart     Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_tap_threshold(uint8_t threshold, bool restart) {
  uint8_t buffer[2];

  // Clamp the value within 0 - 255
  if(threshold >= 255)
    threshold = 255;
  else if (threshold <=0)
    threshold = 0;

  // First read the Tap Timer Limit in order not to overwrite it, this will be stored in buffer[0].
  _read_random_bytes(TAP_SETTINGS, buffer, 1, true);
  // Store the the threshold to the second byte which will be transferred.
  buffer[1] = threshold;
  // Write the tap settings bytes to the correct address.
  _write_random_bytes(TAP_SETTINGS, buffer, 2, restart);
}

/**
 * @brief A method which sets the maximum time allowed for a Swipe gesture. The gesture must occur in less time than this setting to be valid.
 * 
 * @param timeout   A value from 0 to 510 which specifies the Swipe timer limit in 2 millsecond increments.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_swipe_timeout(uint8_t timeout, bool restart) {
  uint8_t buffer[1];

  // Adjust the time value to a value between 0 and 255.
  if(timeout >= 255)
    timeout = 255;
  else if (timeout <=0)
    timeout = 0;

  // Store the time limit to the byte which will be transferred.
  buffer[0] = timeout;
  // Write the time limit to the correct address.
  _write_random_bytes(SWIPE_SETTINGS, buffer, 1, restart);
}

/**
 * @brief A method which sets the minimum coordinate length on the trackpad for Swipe gesture. Swiping for less than the specified threshold will not recognise a Swipe gesture.
 * 
 * @param threshold   A value from 0 to 255 which specifies the minimum swipe coordinate length.
 * @param restart     Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_set_swipe_threshold(uint8_t threshold, bool restart) {
  uint8_t buffer[2];

  // Adjust the time value to a value between 0 and 255.
  if(threshold >= 255)
    threshold = 255;
  else if (threshold <=0)
    threshold = 0;

  // First read the Swipe Timer Limit in order not to overwrite it, this will be stored in buffer[0].
  _read_random_bytes(SWIPE_SETTINGS, buffer, 1, true);
  // Store the the threshold to the second byte which will be transferred.
  buffer[1] = threshold;
  // Write the Swipe settings bytes to the correct address.
  _write_random_bytes(SWIPE_SETTINGS, buffer, 2, restart);
}


/**
 * @brief This function enables corresponding events based on the input event bits.
 * 
 * @param event_bits  The event bits that determine which events are being enabled.
 * @param restart     Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_enable_events(uint8_t event_bits, bool restart) {
  _set_event_mask_bits(event_bits, restart);
}

/**
 * @brief This function enables all events.
 * 
 * @param restart     Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_enable_all_events(bool restart) {
  _set_event_mask_bits(0xff, restart);
}

/**
 * @brief This function disables corresponding events based on the input event bits.
 * 
 * @param event_bits  The event bits that determine which events are being disabled.
 * @param restart     Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_disable_events(uint8_t event_bits, bool restart) {
  _clear_event_mask_bits(event_bits, restart);
}

/**
 * @brief This function disables all events.
 * 
 * @param restart     Determines if the RESART or STOP bit is sent after the communication is done.
 */
void iqs266_disable_all_events(bool restart) {
  _clear_event_mask_bits(0xff, restart);
}

/**
 * @brief A method which enables a channel specified by the user. All current settings are preserved excepts for the channel specified.
 * 
 * @param channel   Use the CHANNEL enumerator to specify a channel to be enabled. Passing CH_LP does nothing.
 * @param restart   Specifies whether the communications window must be kept open or closed after this communication.
 *                  Use the STOP and RESTART definitions.
 */
void iqs266_enable_channel(channel_t channel, bool restart) {
  uint8_t buffer[2];  
  // Read both bytes at the CHANNEL_SETTINGS address, the second byte will be altered.
  _read_random_bytes(CHANNEL_SETTINGS, buffer, 2, true);
  // Set the bit in the second byte for the channel which must be enabled.
  buffer[1] |= (1<<channel);
  // Write the new value to the ATCTIVE_CHANNELS address
  _write_random_bytes(CHANNEL_SETTINGS, buffer, 2, restart);
}

/**
 * @brief A method which disables a channel specified by the user. All current settings are preserved excepts for the channel specified.
 * 
 * @param channel   Use the CHANNEL enumerator to specify a channel to be enabled. Passing CH_LP does nothing.
 * @param restart   Specifies whether the communications window must be kept open or closed after this communication.
 */
void iqs266_disable_channel(channel_t channel, bool restart) {
  uint8_t buffer[2];
  // Read both bytes at the CHANNEL_SETTINGS address, the second byte will be altered.
  _read_random_bytes(CHANNEL_SETTINGS, buffer, 2, true);
  // Clear the bit in the second byte for the channel which must be disabled.
  buffer[1] &= ~(1<<channel);
  // Write the new value to the ATCTIVE_CHANNELS address
  _write_random_bytes(CHANNEL_SETTINGS, buffer, 2, true);
}


/**
 * @brief Function to enable one or more events by setting the appropriate bits in the event mask of the IQS266 device.
 *        Several event bits can be ORed together to enable multiple bits.
 * 
 * @param bit_mask  A byte with the bits which should be set high. High event bits enable corresponding events.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
static void _set_event_mask_bits(uint8_t bit_mask, bool restart) {
  uint8_t buffer[1] = {0}; // Array which will hold the bytes read and written.
  
  // First read the previous settings into the array, keep communication open.
  _read_random_bytes(EVENT_MASK, buffer, 1, true);
  // set the required bits in the event mask.
  buffer[0] |= bit_mask;
  // Write the new byte to the EVENT_MASK address.
  _write_random_bytes(EVENT_MASK, buffer, 1, restart);
}

/**
 * @brief Function to disable one or more events by clearing the appropriate bits in the event mask of the IQS266 device.
 *        Several event bits can be ORed together to disable multiple events.
 * @param mask_bit  A byte with the bits which should be cleared high.
 * @param restart   Determines if the RESART or STOP bit is sent after the communication is done.
 */
static void _clear_event_mask_bits(uint8_t mask_bit, bool restart)
{
  uint8_t buffer[1]; // Array which will hold the bytes read and written.
  // First read the previous settings into the array, keep communication open.
  _read_random_bytes(EVENT_MASK, buffer, 1, true);
  // Clear the required bits from the event mask.
  buffer[0] &= ~(mask_bit);
  // Write the new byte to the EVENT_MASK address.
  _write_random_bytes(EVENT_MASK, buffer, 1, restart);
}


/*******************************************************************************
                      	NOTE Platform Specific Functions
*******************************************************************************/

/**
 * @brief Initialize the MCU interfaces connected to iqs266.
 * 				Also enable the interrupt of the MCU to accept iqs266 event.
 * @note  This function is platform specific.
 */
void iqs266_init() {
  // Initialize the RDY pin
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

  LL_GPIO_InitTypeDef io_config;             
  io_config.Pin = GPIO_RDY_PIN;
  io_config.Mode = LL_GPIO_MODE_INPUT;
  io_config.Pull = LL_GPIO_PULL_UP;
  io_config.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  io_config.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(GPIO_RDY_GROUP, &io_config);

  // Initialize I2C interface.
  i2c3_init();
  
  while (!iqs266_request_communication()) {
    iqs266_clear_reset(true);
    iqs266_read_events(false);
  }
}

/**
 * @brief Request a communication window from iqs266
 * 
 * @return true 
 * @return false 
 * @note   This function is platform specific.
 */
bool iqs266_request_communication() {
  bool response = false;
  uint16_t count = 0;
  // disable_ready_interrupt();
  // toggle ready
  _toggle_ready();
  while(HAL_GPIO_ReadPin(GPIO_RDY_GROUP, GPIO_RDY_PIN) == GPIO_PIN_SET) {
    count ++;
    // Delay 1ms 
    HAL_Delay(1);
    if (count % 1000 == 0)
      return response;
    if (count % 100 == 0)
      _toggle_ready();
  }
  response = true;
  // enable_ready_interrupt();
  return response;
}

/**
 * @brief Wait for the ready signal (RDY low) within timeout.
 * 
 * @return true The iqs266 is ready (RDY is low).
 * @return false The iqs266 is not ready.
 * 
 * @note This function is platfrom specific.
 */
bool iqs266_wait_ready() {
  bool ready = false;
  uint16_t count = 0;
  while(HAL_GPIO_ReadPin(GPIO_RDY_GROUP, GPIO_RDY_PIN) == GPIO_PIN_SET) {
    count ++;
    HAL_Delay(1);
    if(count % 1000 == 0)
      return ready;
  }
  ready = true;
  return ready;
}



/**
 * @brief Set the RDY pin to be output mode so that the MCU can request a communication window.
 * 
 */
static void _set_ready_output() {
  ready_ports->MODER |= 1UL << GPIO_RDY_PIN_NUM * 2 ;
}

/**
 * @brief Set the RDY pin to be input mode so that the MCU can monitor events.
 * 
 */
static void _set_ready_input() {
  ready_ports->MODER &= ~(3UL << GPIO_RDY_PIN_NUM * 2);
}


/**
 * @brief Generate a falling edge to request a communication window.
 * @note  This function is platform specific.
 */
static void _toggle_ready() {
  _set_ready_output();
  HAL_GPIO_WritePin(GPIO_RDY_GROUP, GPIO_RDY_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIO_RDY_GROUP, GPIO_RDY_PIN, GPIO_PIN_SET);
  _set_ready_input();
}

/**
 * @brief A wrapper function implements the I2C communication on specific platforms.
 * 
 * @param address Register address that is being read from.
 * @param buffer  A buffer used for accepting the read results.
 * @param size    Read size in bytes.
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done.
 *  
 * @note This function is platform specific.
 */
static void _read_random_bytes(uint16_t address, uint8_t buffer[], uint16_t size, bool restart) {
  // HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c3, IQS266_ADDRESS << 1, address, 1, buffer, size, 200);
  ErrorStatus status = i2c3_random_read(IQS266_ADDRESS, address, buffer, size, restart);
  if (status == ERROR)
    printf("read random bytes failed.\n");
}

/**
 * @brief A wrapper function implements the I2C communication on specific platforms.
 * 
 * @param address Register address that is being write to.
 * @param buffer  A buffer used for holdering the data being transmitted.
 * @param size    Write size in bytes.
 * @param restart Determines if the RESART or STOP bit is sent after the communication is done.
 *
 * @note This function is platfrom specific.
 */
static void _write_random_bytes(uint16_t address, uint8_t buffer[], uint16_t size, bool restart) {
  // HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c3, IQS266_ADDRESS << 1, address, 1, buffer, size, 200);
  ErrorStatus status = i2c3_random_write(IQS266_ADDRESS, address, buffer, size, restart);
  if (status == ERROR)
    printf("write random bytes failed.\n");
}



