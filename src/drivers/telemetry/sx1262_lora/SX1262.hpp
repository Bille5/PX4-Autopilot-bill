#pragma once

#include <drivers/device/spi.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include "sx126x_regs.hpp"
#include "LoraLink.hpp"

using namespace time_literals;

class SX1262 : public device::SPI, public ModuleBase
{
public:
	static ModuleBase::Descriptor desc;

	SX1262(int bus, uint32_t device, spi_mode_e mode, uint32_t frequency,
	       uint32_t gpio_busy, uint32_t gpio_nrst, uint32_t gpio_txen, uint32_t gpio_rxen,
	       uint32_t rf_freq_hz);
	~SX1262() override;

	static int task_spawn(int argc, char *argv[]);
	static int run_trampoline(int argc, char *argv[]);
	static SX1262 *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

	static bool can_stop() { return !get_instance<SX1262>(desc)->_link.in_use(); }

private:
	int radio_init();
	bool wait_busy(hrt_abstime timeout_us = 5_ms);
	int cmd_write(sx126x::OpCode op, const uint8_t *params, size_t n);
	int cmd_read(sx126x::OpCode op, uint8_t *out, size_t n);
	int cmd_write_buffer(uint8_t offset, const uint8_t *data, size_t len);
	int cmd_read_buffer(uint8_t offset, uint8_t *out, size_t len);
	void reset_chip();
	void set_gpio(uint32_t gpio, bool value);

	uint16_t read_irq_status();
	void clear_irq_status(uint16_t mask);

	void start_rx();
	void service_tx();
	void service_rx();

	LoraLink _link{"/dev/lora0"};

	uint32_t _gpio_busy;
	uint32_t _gpio_nrst;
	uint32_t _gpio_txen;
	uint32_t _gpio_rxen;
	uint32_t _rf_freq_hz;

	enum class State { Rx, TxPending } _state{State::Rx};
	hrt_abstime _tx_deadline{0};

	perf_counter_t _tx_ok;
	perf_counter_t _rx_ok;
	perf_counter_t _crc_err;
	perf_counter_t _tx_timeout;
	perf_counter_t _busy_timeout;
};
