#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#ifndef RFM95_SPI_TIMEOUT
#define RFM95_SPI_TIMEOUT 10
#endif

#ifndef RFM95_WAKEUP_TIMEOUT
#define RFM95_WAKEUP_TIMEOUT 10
#endif

#ifndef RFM95_SEND_TIMEOUT
#define RFM95_SEND_TIMEOUT 100
#endif

#ifndef RFM95_RECEIVE_TIMEOUT
#define RFM95_RECEIVE_TIMEOUT 1000
#endif

typedef void (*rfm95_on_after_interrupts_configured)();
typedef uint32_t (*rfm95_get_precision_tick)();
typedef void (*rfm95_precision_sleep_until)(uint32_t ticks_target);

typedef enum
{
	RFM95_INTERRUPT_DIO0,
	RFM95_INTERRUPT_DIO1,
	RFM95_INTERRUPT_DIO5
} rfm95_interrupt_t;

#define RFM95_INTERRUPT_COUNT 3

/**
 * Structure defining a handle describing an RFM95(W) transceiver.
 */
typedef struct {

	/**
	 * The handle to the SPI bus for the device.
	 */
	SPI_HandleTypeDef *spi_handle;

	/**
	 * The port of the NSS pin.
	 */
	GPIO_TypeDef *nss_port;

	/**
	 * The NSS pin.
	 */
	uint16_t nss_pin;

	/**
	 * The port of the RST pin.
	 */
	GPIO_TypeDef *nrst_port;

	/**
	 * The RST pin.
	 */
	uint16_t nrst_pin;

	/**
	 * The frequency of the precision tick in Hz.
	 */
	uint32_t precision_tick_frequency;

	/**
	 * The +/- timing drift per second in nanoseconds.
	 */
	uint32_t precision_tick_drift_ns_per_s;

	/**
	 * Function provided that returns a precise tick for timing critical operations.
	 */
	rfm95_get_precision_tick get_precision_tick;

	/**
	 * Function that provides a precise sleep until a given tick count is reached.
	 */
	rfm95_precision_sleep_until precision_sleep_until;

	/**
	 * Callback called after the interrupt functions have been properly configured;
	 */
	rfm95_on_after_interrupts_configured on_after_interrupts_configured;

	/**
	 * Tick values when each interrupt was called.
	 */
	volatile uint32_t interrupt_times[RFM95_INTERRUPT_COUNT];

} rfm95_handle_t;

bool rfm95_init(rfm95_handle_t *handle);

bool rfm95_set_power(rfm95_handle_t *handle, int8_t power);

bool rfm95_send_package(rfm95_handle_t *handle, const uint8_t *send_data, size_t send_data_length);

bool rfm95_receive_package(rfm95_handle_t *handle, uint8_t *receive_data, size_t *receive_data_length, int8_t *snr);

void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt);