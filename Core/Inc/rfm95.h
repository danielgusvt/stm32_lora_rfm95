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
#define RFM95_SEND_TIMEOUT 2000
#endif

#ifndef RFM95_RECEIVE_TIMEOUT
#define RFM95_RECEIVE_TIMEOUT 2000
#endif

#define RFM9x_VER 0x12

typedef enum
{
	RFM95_REGISTER_FIFO_ACCESS = 0x00,
	RFM95_REGISTER_OP_MODE = 0x01,
	RFM95_REGISTER_FR_MSB = 0x06,
	RFM95_REGISTER_FR_MID = 0x07,
	RFM95_REGISTER_FR_LSB = 0x08,
	RFM95_REGISTER_PA_CONFIG = 0x09,
	RFM95_REGISTER_LNA = 0x0C,
	RFM95_REGISTER_FIFO_ADDR_PTR = 0x0D,
	RFM95_REGISTER_FIFO_TX_BASE_ADDR = 0x0E,
	RFM95_REGISTER_FIFO_RX_BASE_ADDR = 0x0F,
	RFM95_REGISTER_IRQ_FLAGS = 0x12,
	RFM95_REGISTER_FIFO_RX_BYTES_NB = 0x13,
    RFM95_REGISTER_FIFO_RX_CURRENT_ADDR = 0x10,
	RFM95_REGISTER_PACKET_SNR = 0x19,
	RFM95_REGISTER_MODEM_CONFIG_1 = 0x1D,
	RFM95_REGISTER_MODEM_CONFIG_2 = 0x1E,
	RFM95_REGISTER_SYMB_TIMEOUT_LSB = 0x1F,
	RFM95_REGISTER_PREAMBLE_MSB = 0x20,
	RFM95_REGISTER_PREAMBLE_LSB = 0x21,
	RFM95_REGISTER_PAYLOAD_LENGTH = 0x22,
	RFM95_REGISTER_MAX_PAYLOAD_LENGTH = 0x23,
	RFM95_REGISTER_MODEM_CONFIG_3 = 0x26,
	RFM95_REGISTER_INVERT_IQ_1 = 0x33,
	RFM95_REGISTER_SYNC_WORD = 0x39,
	RFM95_REGISTER_INVERT_IQ_2 = 0x3B,
	RFM95_REGISTER_DIO_MAPPING_1 = 0x40,
	RFM95_REGISTER_VERSION = 0x42,
	RFM95_REGISTER_PA_DAC = 0x4D
} rfm95_register_t;

typedef struct
{
	union {
		struct {
			uint8_t output_power : 4;
			uint8_t max_power : 3;
			uint8_t pa_select : 1;
		};
		uint8_t buffer;
	};
} rfm95_register_pa_config_t;

#define RFM95_REGISTER_OP_MODE_SLEEP                            0x00
#define RFM95_REGISTER_OP_MODE_LORA_SLEEP                       0x80
#define RFM95_REGISTER_OP_MODE_LORA_STANDBY                     0x81
#define RFM95_REGISTER_OP_MODE_LORA_TX                          0x83
#define RFM95_REGISTER_OP_MODE_LORA_RX_CONT                     0x85
#define RFM95_REGISTER_OP_MODE_LORA_RX_SINGLE                   0x86

#define RFM95_REGISTER_PA_DAC_LOW_POWER                         0x84
#define RFM95_REGISTER_PA_DAC_HIGH_POWER                        0x87

#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE             0x40
#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE             0x00

#define RFM95_REGISTER_INVERT_IQ_1_TX                    		0x27
#define RFM95_REGISTER_INVERT_IQ_2_TX							0x1d

#define RFM95_REGISTER_INVERT_IQ_1_RX                    		0x67
#define RFM95_REGISTER_INVERT_IQ_2_RX							0x19

/*
 * RegModemConfig 1 (0x1D)
 */
// Signal bandwidth, must be shifted to the left by 4 bits
#define RFM95_BW_78  0
#define RFM95_BW_104  1
#define RFM95_BW_156  2
#define RFM95_BW_208 3
#define RFM95_BW_3125  4
#define RFM95_BW_417  5
#define RFM95_BW_625  6
#define RFM95_BW_125  7
#define RFM95_BW_250  8
#define RFM95_BW_500  9
// Coding rate, must be shifted to the left by 1 bit
#define RFM95_CR_45 1
#define RFM95_CR_46 2
#define RFM95_CR_47 3
#define RFM95_CR_48 4
// Implicit Header Mode On
#define RFM95_IMP_HEA 1

/*
 * RegModemConfig 2 (0x1E)
 */
// Spreading factor, must be shifted to the left by 4 bits
#define RFM95_SF_6  6
#define RFM95_SF_7  7
#define RFM95_SF_8  8
#define RFM95_SF_9  9
#define RFM95_SF_10 10
#define RFM95_SF_11  11
#define RFM95_SF_12  12
// TxContinuousMode, must be shifted to the left by 3 bits
#define RFM95_TX_CONT 1
// RxPayloadCrcOn, must be shifted to the left by 2 bits
#define RFM95_CRC_ON 1

/*
 * RegModemConfig 3 (0x26)
 */
// MobileNode, must be shifted to the left by 3 bits
#define RFM95_STATIC_ND 0
#define RFM95_MOBILE_ND 1
// AgcAutoOn, must be shifted to the left by 2 bits
#define RFM95_AGC_AUTO 1

typedef void (*rfm95_on_after_interrupts_configured)();
typedef uint32_t (*rfm95_get_precision_tick)();
typedef void (*rfm95_precision_sleep_until)(uint32_t ticks_target);

typedef enum
{
	RFM95_INTERRUPT_DIO0
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

bool rfm95_read_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t *value);

void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt);