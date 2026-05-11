#include "rfm95.h"

#include <assert.h>
#include <string.h>

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

static bool read_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t *buffer, size_t length)
{
	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

	uint8_t transmit_buffer = (uint8_t)reg & 0x7fu;

	if (HAL_SPI_Transmit(handle->spi_handle, &transmit_buffer, 1, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	if (HAL_SPI_Receive(handle->spi_handle, buffer, length, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

	return true;
}

static bool write_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t value)
{
	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);

	uint8_t transmit_buffer[2] = {((uint8_t)reg | 0x80u), value};

	if (HAL_SPI_Transmit(handle->spi_handle, transmit_buffer, 2, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

	return true;
}

static void reset(rfm95_handle_t *handle)
{
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_SET);
	HAL_Delay(5);
}

static bool configure_frequency(rfm95_handle_t *handle, uint32_t frequency)
{
	// FQ = (FRF * 32 Mhz) / (2 ^ 19)
	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	if (!write_register(handle, RFM95_REGISTER_FR_MSB, (uint8_t)(frf >> 16))) return false;
	if (!write_register(handle, RFM95_REGISTER_FR_MID, (uint8_t)(frf >> 8))) return false;
	if (!write_register(handle, RFM95_REGISTER_FR_LSB, (uint8_t)(frf >> 0))) return false;

	return true;
}

static bool wait_for_irq(rfm95_handle_t *handle, rfm95_interrupt_t interrupt, uint32_t timeout_ms)
{
	uint32_t timeout_tick = handle->get_precision_tick() + timeout_ms * handle->precision_tick_frequency / 1000;

	while (handle->interrupt_times[interrupt] == 0) {
		if (handle->get_precision_tick() >= timeout_tick) {
			return false;
		}
	}

	return true;
}

bool rfm95_set_power(rfm95_handle_t *handle, int8_t power)
{
	assert((power >= 2 && power <= 17) || power == 20);

	rfm95_register_pa_config_t pa_config = {0};
	uint8_t pa_dac_config = 0;

	if (power >= 2 && power <= 17) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = (power - 2);
		pa_dac_config = RFM95_REGISTER_PA_DAC_LOW_POWER;

	} else if (power == 20) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = 15;
		pa_dac_config = RFM95_REGISTER_PA_DAC_HIGH_POWER;
	}

	if (!write_register(handle, RFM95_REGISTER_PA_CONFIG, pa_config.buffer)) return false;
	if (!write_register(handle, RFM95_REGISTER_PA_DAC, pa_dac_config)) return false;

	return true;
}

bool rfm95_init(rfm95_handle_t *handle)
{
	assert(handle->spi_handle->Init.Mode == SPI_MODE_MASTER);
	assert(handle->spi_handle->Init.Direction == SPI_DIRECTION_2LINES);
	assert(handle->spi_handle->Init.DataSize == SPI_DATASIZE_8BIT);
	assert(handle->spi_handle->Init.CLKPolarity == SPI_POLARITY_LOW);
	assert(handle->spi_handle->Init.CLKPhase == SPI_PHASE_1EDGE);
	assert(handle->get_precision_tick != NULL);
	assert(handle->precision_sleep_until != NULL);
	assert(handle->precision_tick_frequency > 10000);

	reset(handle);

	// Check for correct version.
	uint8_t version;
	if (!read_register(handle, RFM95_REGISTER_VERSION, &version, 1)) return false;
	if (version != RFM9x_VER) return false;

	// Module must be placed in sleep mode before switching to lora.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_SLEEP)) return false;
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	// Set standard frequency 868.0 MHz
	configure_frequency(handle, 868000000);

	// Default interrupt configuration, must be done to prevent DIO5 clock interrupts at 1Mhz
	if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;

	if (handle->on_after_interrupts_configured != NULL) {
		handle->on_after_interrupts_configured();
	}

	// Set module power to 17dbm.
	if (!rfm95_set_power(handle, 17)) return false;

	// Set LNA to the highest gain with 150% boost.
	if (!write_register(handle, RFM95_REGISTER_LNA, 0x23)) return false;

	// Preamble set to 8.
	if (!write_register(handle, RFM95_REGISTER_PREAMBLE_MSB, 0x00)) return false;
	if (!write_register(handle, RFM95_REGISTER_PREAMBLE_LSB, 0x08)) return false;

	// Set standard sync word 0x12.
	if (!write_register(handle, RFM95_REGISTER_SYNC_WORD, 0x12)) return false;

	// Set up TX and RX FIFO base addresses.
	if (!write_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, 0x80)) return false;
	if (!write_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, 0x00)) return false;

	// Maximum payload length of the RFM95 is 252 (with radiohead headers max is usually 255 but keep 252 for saftey)
	if (!write_register(handle, RFM95_REGISTER_MAX_PAYLOAD_LENGTH, 252)) return false;

	// Configure modem (125kHz, 4/5 error coding rate, SF7, single packet, CRC enable, AGC auto on)
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return false;
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x74)) return false;
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, 0x04)) return false;

	// Let module sleep after initialisation.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	return true;
}

bool rfm95_send_package(rfm95_handle_t *handle, const uint8_t *send_data, size_t send_data_length)
{
	if (send_data_length > 248) return false; // Max payload

	uint8_t payload_buf[252];
	
	// RadioHead format header
	payload_buf[0] = 0xff; // Destination
	payload_buf[1] = 0xff; // Node
	payload_buf[2] = 0x00; // Identifier
	payload_buf[3] = 0x00; // Flags
	
	// Copy payload
	memcpy(payload_buf + 4, send_data, send_data_length);
	size_t payload_len = send_data_length + 4;

	// Set IQ registers for standard node-to-node.
	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_TX)) return false;
	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_TX)) return false;

	// Set the payload length.
	if (!write_register(handle, RFM95_REGISTER_PAYLOAD_LENGTH, payload_len)) return false;

	// Enable tx-done interrupt, clear flags and previous interrupt time.
	if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE)) return false;
	if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return false;
	handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;

	// Move modem to lora standby.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;

	// Set pointer to start of TX section in FIFO.
	if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, 0x80)) return false;

	// Write payload to FIFO.
	for (size_t i = 0; i < payload_len; i++) {
		write_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload_buf[i]);
	}

	// Set modem to tx mode.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_TX)) return false;

	// Wait for the transfer complete interrupt.
	if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO0, RFM95_SEND_TIMEOUT)) return false;

	// Return modem to sleep.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	return true;
}

bool rfm95_receive_package(rfm95_handle_t *handle, uint8_t *receive_data, size_t *receive_data_length, int8_t *snr)
{
	*receive_data_length = 0;

	// Set IQ registers for standard node-to-node.
	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_TX)) return false;
	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_TX)) return false;

	// Clear flags and previous interrupt time, configure mapping for RX done.
	if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;
	if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return false;
	handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;

	// Enter RX Continuous mode
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_RX_CONT)) return false;

	// Wait for RX done interrupt
	if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO0, RFM95_RECEIVE_TIMEOUT)) {
		// Timeout
		if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;
		return false;
	}

	uint8_t irq_flags;
	read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irq_flags, 1);

	// Check if there was a CRC error.
	if (irq_flags & 0x20) {
        if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;
		return false;
	}

	int8_t packet_snr;
	if (!read_register(handle, RFM95_REGISTER_PACKET_SNR, (uint8_t *)&packet_snr, 1)) return false;
	if (snr) *snr = (int8_t)(packet_snr / 4);

	// Read received payload length.
	uint8_t payload_len_internal;
	if (!read_register(handle, RFM95_REGISTER_FIFO_RX_BYTES_NB, &payload_len_internal, 1)) return false;

	// Read received payload itself.
    uint8_t rx_current_addr;
    if (!read_register(handle, RFM95_REGISTER_FIFO_RX_CURRENT_ADDR, &rx_current_addr, 1)) return false;
	if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, rx_current_addr)) return false;
    
    uint8_t payload_buf[252];
	if (!read_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload_buf, payload_len_internal)) return false;

	// Return modem to sleep.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

    // Decode RadioHead header
    if (payload_len_internal >= 4) {
        *receive_data_length = payload_len_internal - 4;
        memcpy(receive_data, payload_buf + 4, *receive_data_length);
        return true;
    }

	return false;
}

void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt)
{
	handle->interrupt_times[interrupt] = handle->get_precision_tick();
}