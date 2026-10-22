#include "SX1262.hpp"

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_config.h>
#include <px4_arch/micro_hal.h>
#include <drivers/drv_sensor.h>
#include <string.h>

using namespace sx126x;

ModuleBase::Descriptor SX1262::desc{task_spawn, custom_command, print_usage};

#define MAX_PAYLOAD 255

SX1262::SX1262(int bus, uint32_t device, spi_mode_e mode, uint32_t frequency,
	       uint32_t gpio_busy, uint32_t gpio_nrst, uint32_t gpio_txen, uint32_t gpio_rxen,
	       uint32_t rf_freq_hz)
	: SPI(DRV_TEL_DEVTYPE_SX1262, "sx1262_lora", bus, device, mode, frequency),
	  _gpio_busy(gpio_busy), _gpio_nrst(gpio_nrst), _gpio_txen(gpio_txen), _gpio_rxen(gpio_rxen),
	  _rf_freq_hz(rf_freq_hz)
{
	_tx_ok = perf_alloc(PC_COUNT, "sx1262: tx_ok");
	_rx_ok = perf_alloc(PC_COUNT, "sx1262: rx_ok");
	_crc_err = perf_alloc(PC_COUNT, "sx1262: crc_err");
	_tx_timeout = perf_alloc(PC_COUNT, "sx1262: tx_timeout");
	_busy_timeout = perf_alloc(PC_COUNT, "sx1262: busy_timeout");
}

SX1262::~SX1262()
{
	perf_free(_tx_ok);
	perf_free(_rx_ok);
	perf_free(_crc_err);
	perf_free(_tx_timeout);
	perf_free(_busy_timeout);
}

void SX1262::set_gpio(uint32_t gpio, bool value)
{
	px4_arch_gpiowrite(gpio, value);
}

bool SX1262::wait_busy(hrt_abstime timeout_us)
{
	hrt_abstime start = hrt_absolute_time();

	while (px4_arch_gpioread(_gpio_busy)) {
		if (hrt_elapsed_time(&start) > timeout_us) {
			perf_count(_busy_timeout);
			return false;
		}
	}

	return true;
}

void SX1262::reset_chip()
{
	set_gpio(_gpio_nrst, false);
	px4_usleep(200);
	set_gpio(_gpio_nrst, true);
	wait_busy(10_ms);
}

int SX1262::cmd_write(OpCode op, const uint8_t *params, size_t n)
{
	if (n > 16) {
		return -EINVAL;
	}

	if (!wait_busy()) {
		return -ETIMEDOUT;
	}

	uint8_t tx[1 + 16] = {};
	uint8_t rx[1 + 16] = {};
	tx[0] = (uint8_t)op;

	if (n) {
		memcpy(&tx[1], params, n);
	}

	return transfer(tx, rx, n + 1);
}

int SX1262::cmd_read(OpCode op, uint8_t *out, size_t n)
{
	if (n > 16) {
		return -EINVAL;
	}

	if (!wait_busy()) {
		return -ETIMEDOUT;
	}

	// frame: [opcode][RFU/status][data...]
	uint8_t tx[2 + 16] = {};
	uint8_t rx[2 + 16] = {};
	tx[0] = (uint8_t)op;

	int ret = transfer(tx, rx, n + 2);

	if (ret != PX4_OK) {
		return ret;
	}

	if (out && n) {
		memcpy(out, &rx[2], n);
	}

	return PX4_OK;
}

uint16_t SX1262::read_irq_status()
{
	uint8_t buf[2] = {};
	cmd_read(OpCode::GetIrqStatus, buf, 2);
	return ((uint16_t)buf[0] << 8) | buf[1];
}

void SX1262::clear_irq_status(uint16_t mask)
{
	uint8_t params[2] = {(uint8_t)(mask >> 8), (uint8_t)(mask & 0xFF)};
	cmd_write(OpCode::ClearIrqStatus, params, 2);
}

int SX1262::radio_init()
{
	set_gpio(_gpio_txen, false);
	set_gpio(_gpio_rxen, false);

	reset_chip();

	uint8_t p1[1] = {STDBY_RC};
	cmd_write(OpCode::SetStandby, p1, 1);

	// uint8_t p2[1] = {REGULATOR_DC_DC}; // TODO: confirm vs REGULATOR_LDO_ONLY for this module/board power design
	// cmd_write(OpCode::SetRegulatorMode, p2, 1);

	uint8_t calib[1] = {0x7F};
	cmd_write(OpCode::Calibrate, calib, 1);
	px4_usleep(5000); // calibration takes a few ms; wait_busy() in next cmd also covers this

	uint8_t img[2] = {0xE1, 0xE9}; // 902-928 MHz image calibration range
	cmd_write(OpCode::CalibrateImage, img, 2);

	uint8_t pkt_type[1] = {PACKET_TYPE_LORA};
	cmd_write(OpCode::SetPacketType, pkt_type, 1);

	uint32_t freq_reg = freq_to_reg(_rf_freq_hz);
	uint8_t freq_params[4] = {
		(uint8_t)(freq_reg >> 24), (uint8_t)(freq_reg >> 16),
		(uint8_t)(freq_reg >> 8),  (uint8_t)(freq_reg)
	};
	cmd_write(OpCode::SetRfFrequency, freq_params, 4);

	// +22 dBm PA config for SX1262 (see datasheet table 13-21)
	uint8_t pa_config[4] = {0x04, 0x07, 0x00, 0x01};
	cmd_write(OpCode::SetPaConfig, pa_config, 4);

	// power=22dBm, ramp=200us (0x04)
	uint8_t tx_params[2] = {22, 0x04};
	cmd_write(OpCode::SetTxParams, tx_params, 2);

	uint8_t buf_base[2] = {0x00, 0x00}; // tx base 0, rx base 0
	cmd_write(OpCode::SetBufferBaseAddress, buf_base, 2);

	// SF9 / BW125 / CR4_5 / LowDataRateOptimize off  -- TODO tune for your range/rate needs
	uint8_t mod_params[4] = {LoRaSF::SF9, LoRaBW::BW125, LoRaCR::CR4_5, 0x00};
	cmd_write(OpCode::SetModulationParams, mod_params, 4);

	// preamble=8, explicit header, max payload 255, CRC on, standard IQ
	uint8_t pkt_params[9] = {
		0x00, 0x08,   // preamble length (16-bit)
		0x00,         // explicit header
		MAX_PAYLOAD,  // payload length
		0x01,         // CRC on
		0x00,         // standard IQ
		0x00, 0x00, 0x00 // padding for alignment/future use, harmless extra bytes if API expects exactly 6
	};
	cmd_write(OpCode::SetPacketParams, pkt_params, 6); // NOTE: verify exact param count against datasheet before relying on this

	uint16_t irq_mask = irq::TxDone | irq::RxDone | irq::HeaderErr | irq::CrcErr | irq::Timeout;
	uint8_t dio_params[8] = {
		(uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask), // IRQ mask
		0x00, 0x00, // DIO1 mask (unused, not wired)
		0x00, 0x00, // DIO2 mask (unused)
		0x00, 0x00, // DIO3 mask (unused)
	};
	cmd_write(OpCode::SetDioIrqParams, dio_params, 8);

	clear_irq_status(irq::All);

	return PX4_OK;
}


int SX1262::cmd_write_buffer(uint8_t offset, const uint8_t *data, size_t len)
{
	if (len > MAX_PAYLOAD || !wait_busy()) {
		return -EINVAL;
	}

	uint8_t tx[2 + MAX_PAYLOAD] = {};
	uint8_t rx[2 + MAX_PAYLOAD] = {};
	tx[0] = (uint8_t)OpCode::WriteBuffer;
	tx[1] = offset;
	memcpy(&tx[2], data, len);

	return transfer(tx, rx, len + 2);
}

int SX1262::cmd_read_buffer(uint8_t offset, uint8_t *out, size_t len)
{
	if (len > MAX_PAYLOAD || !wait_busy()) {
		return -EINVAL;
	}

	// frame: [opcode][offset][RFU/status][data...]
	uint8_t tx[3 + MAX_PAYLOAD] = {};
	uint8_t rx[3 + MAX_PAYLOAD] = {};
	tx[0] = (uint8_t)OpCode::ReadBuffer;
	tx[1] = offset;

	int ret = transfer(tx, rx, len + 3);

	if (ret != PX4_OK) {
		return ret;
	}

	memcpy(out, &rx[3], len);
	return PX4_OK;
}

void SX1262::start_rx()
{
	set_gpio(_gpio_txen, false);
	set_gpio(_gpio_rxen, true);

	uint8_t rx_pkt_params[6] = {0x00, 0x08, 0x00, MAX_PAYLOAD, 0x01, 0x00};
	cmd_write(OpCode::SetPacketParams, rx_pkt_params, 6);

	uint8_t rx_timeout[3] = {0xFF, 0xFF, 0xFF}; // continuous RX
	cmd_write(OpCode::SetRx, rx_timeout, 3);

	_state = State::Rx;
}

void SX1262::service_tx()
{
	uint8_t chunk[MAX_PAYLOAD];
	size_t n = _link.pull_tx(chunk, MAX_PAYLOAD);

	if (n == 0) {
		return;
	}

	set_gpio(_gpio_rxen, false);
	set_gpio(_gpio_txen, true);

	cmd_write_buffer(0x00, chunk, n);

	clear_irq_status(irq::All);

	// timeout ~ generous margin over time-on-air at SF9/BW125 for MAX_PAYLOAD
	// explicit header: this length is what actually gets transmitted
	uint8_t tx_pkt_params[6] = {0x00, 0x08, 0x00, (uint8_t)n, 0x01, 0x00};
	cmd_write(OpCode::SetPacketParams, tx_pkt_params, 6);

	uint8_t tx_timeout[3] = {0x00, 0x00, 0x00}; // chip timeout off; software deadline covers it
	// (old placeholder:) uint8_t unused_timeout[3] = {0x00, 0x4E, 0x20}; // ~5s in 15.625us units, conservative placeholder -- tune down later
	cmd_write(OpCode::SetTx, tx_timeout, 3);

	_state = State::TxPending;
	_tx_deadline = hrt_absolute_time() + 3_s;
}

void SX1262::service_rx()
{
	uint16_t irqs = read_irq_status();

	if (irqs & irq::RxDone) {
		if (irqs & (irq::CrcErr | irq::HeaderErr)) {
			perf_count(_crc_err);

		} else {
			uint8_t status_buf[2] = {};
			cmd_read(OpCode::GetRxBufferStatus, status_buf, 2);
			uint8_t payload_len = status_buf[0];
			uint8_t start_ptr = status_buf[1];

			uint8_t payload[MAX_PAYLOAD];
			cmd_read_buffer(start_ptr, payload, payload_len);

			_link.push_rx(payload, payload_len);
			perf_count(_rx_ok);
		}

		clear_irq_status(irq::RxDone | irq::CrcErr | irq::HeaderErr);
	}

	if (irqs) {
		clear_irq_status(irqs);
	}
}

void SX1262::run()
{
	radio_init();
	start_rx();

	while (!should_exit()) {
		switch (_state) {
		case State::Rx:
			service_rx();

			if (_link.tx_pending() > 0) {
				service_tx();
			}

			break;

		case State::TxPending: {
				uint16_t irqs = read_irq_status();

				if (irqs & irq::TxDone) {
					clear_irq_status(irq::TxDone);
					perf_count(_tx_ok);
					start_rx();

				} else if (hrt_absolute_time() > _tx_deadline) {
					perf_count(_tx_timeout);
					// recovery: standby then back to rx, per RadioLib/community findings
					uint8_t p[1] = {STDBY_RC};
					cmd_write(OpCode::SetStandby, p, 1);
					start_rx();
				}

				break;
			}
		}

		px4_usleep(2000); // 2ms poll interval
	}
}

int SX1262::print_status()
{
	PX4_INFO("state: %s", _state == State::Rx ? "RX" : "TX_PENDING");
	perf_print_counter(_tx_ok);
	perf_print_counter(_rx_ok);
	perf_print_counter(_crc_err);
	perf_print_counter(_tx_timeout);
	perf_print_counter(_busy_timeout);
	return 0;
}

int SX1262::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return SX1262::instantiate(ac, av);
	}, argc, argv);
}

int SX1262::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("sx1262_lora",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_SLOW_DRIVER,
					   3000,
					   (px4_main_t)&run_trampoline,
					   (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	if (wait_until_running(desc, 6000) < 0) {
		desc.task_id = -1;
		return -1;
	}

	return 0;
}

SX1262 *SX1262::instantiate(int argc, char *argv[])
{
	// TODO: parse bus/CS/GPIO/frequency from argv instead of hardcoding;
	// for now these must match board_config.h defines exactly.
	int bus = 1;
	uint32_t device = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, DRV_TEL_DEVTYPE_SX1262);
	uint32_t freq_hz = 915000000;

	SX1262 *instance = new SX1262(bus, device, SPIDEV_MODE0, 2 * 1000 * 1000,
				       GPIO_LORA_BUSY, GPIO_LORA_NRST, GPIO_LORA_TXEN, GPIO_LORA_RXEN,
				       freq_hz);

	if (instance) {
		if (instance->SPI::init() != PX4_OK) {
			delete instance;
			return nullptr;
		}

		if (instance->_link.init() != PX4_OK) {
			delete instance;
			return nullptr;
		}
	}

	return instance;
}

int SX1262::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SX1262 (Ebyte E22-900MM22S) LoRa driver.
Creates /dev/lora0 for mavlink to use as its link.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sx1262_lora", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 1;
}

int SX1262::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

extern "C" __EXPORT int sx1262_lora_main(int argc, char *argv[])
{
	if (argc >= 2 && !strcmp(argv[1], "stop") && SX1262::is_running(SX1262::desc)) {
		if (!SX1262::can_stop()) {
			PX4_ERR("Device is used. Stop mavlink first (mavlink stop-all)");
			return -1;
		}
	}

	return ModuleBase::main(SX1262::desc, argc, argv);
}
