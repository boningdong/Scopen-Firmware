/**
 * @file i2c.h
 * @author boning@ucsb.edu
 * @brief Includes the low level drivers for the i2c perpherals.
 * @version 0.1
 * @date 2020-05-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "main.h"

#define NBYTES_MAX 255
#define DEFAULT_TIMEOUT 100

#define CHECK_TIMEOUT(counter) (LL_SYSTICK_IsActiveCounterFlag() && counter-- == 0)
#define WAIT_CONDITION(condition, timeout, counter, on_timeout)   \
        do {                                                    \
            timer = timeout;                                    \
            while (!condition)                                  \
              if (CHECK_TIMEOUT(timer)) on_timeout;             \
        } while(0);

/**
 * @brief Initialize the i2c3 interface on the board.
 *        Enable the auto-end mode and the clock stretching mode.
 * 
 * @return true   Initialization is successful.
 * @return false  Initialization failed.
 */
ErrorStatus i2c3_init();
/**
 * @brief Read the continuos n registers values in the target device, starting from the register address specified.
 * 
 * @param device    7 bit target device address.
 * @param address   The starting register address.
 * @param buffer    Receiving buffer used for accepting the read result.
 * @param size      The byte numbers of values to be read.
 * @param restart   End the conversation with STOP or another START flag to restart.
 */
ErrorStatus i2c3_random_read(const uint16_t device, const uint8_t address, uint8_t* buffer, uint16_t size, bool restart);

/**
 * @brief Write continuous n bytes to the registers to the target device synchronically, starting form the register address specified.  
 * 
 * @param device        7 bit target device address.
 * @param address       The starting register address.
 * @param buffer        Transmit buffer that holds the data to be transferred.
 * @param size          Number of byte to be transferred.
 * @param restart       End the conversation with STOP or another START flag to restart.
 * @return ErrorStatus  Return SUCCESS if it successfully runs. Ruturn ERROR if it's not.
 */
ErrorStatus i2c3_random_write(const uint16_t device, const uint8_t address, const uint8_t* buffer, uint16_t size, bool restart);