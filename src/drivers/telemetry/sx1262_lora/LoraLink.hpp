#pragma once

#include <lib/cdev/CDev.hpp>
#include <px4_platform_common/atomic.h>
#include <pthread.h>

/**
 * Virtual character device exposing the LoRa link as a byte pipe.
 * The mavlink module opens this exactly as it would a UART.
 *
 * NOTE: this class deliberately does NOT answer TCGETS/TCSETS.
 * MAVLink's tcgetattr() call on this device must be bypassed via a
 * MAVLINK_MODE_LORA early-return patch in mavlink_main.cpp, mirroring
 * how MAVLINK_MODE_IRIDIUM is handled. See project notes.
 */
class LoraLink : public cdev::CDev
{
public:
	explicit LoraLink(const char *device_path);
	~LoraLink() override;

	int init();

	ssize_t read(struct file *filp, char *buffer, size_t buflen) override;
	ssize_t write(struct file *filp, const char *buffer, size_t buflen) override;
	int ioctl(struct file *filp, int cmd, unsigned long arg) override;
	pollevent_t poll_state(struct file *filp) override;

	int open_first(struct file *filep) override;
	int close_last(struct file *filep) override;

	bool in_use() const { return _cdev_used.load(); }

	/** Radio-side API: pull up to `max_len` queued TX bytes. Returns bytes copied. */
	size_t pull_tx(uint8_t *dst, size_t max_len);

	/** Radio-side API: push received payload bytes into the RX buffer. */
	void push_rx(const uint8_t *src, size_t len);

	size_t tx_pending() const { return _tx_write_idx - _tx_read_idx; }

private:
	static constexpr size_t TX_BUF_LEN = 2048;
	static constexpr size_t RX_BUF_LEN = 2048;

	uint8_t _tx_buf[TX_BUF_LEN];
	size_t _tx_write_idx{0};
	size_t _tx_read_idx{0};

	uint8_t _rx_buf[RX_BUF_LEN];
	size_t _rx_write_idx{0};
	size_t _rx_read_idx{0};

	pthread_mutex_t _tx_mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_mutex_t _rx_mutex = PTHREAD_MUTEX_INITIALIZER;

	px4::atomic_bool _cdev_used{false};
};
