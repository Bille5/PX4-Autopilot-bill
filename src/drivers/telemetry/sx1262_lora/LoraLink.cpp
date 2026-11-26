#include "LoraLink.hpp"
#include <px4_platform_common/defines.h>
#include <string.h>
#include <errno.h>
#include <poll.h>

LoraLink::LoraLink(const char *device_path) : CDev(device_path)
{
}

LoraLink::~LoraLink()
{
	pthread_mutex_destroy(&_tx_mutex);
	pthread_mutex_destroy(&_rx_mutex);
}

int LoraLink::init()
{
	return CDev::init();
}

ssize_t LoraLink::write(struct file *filp, const char *buffer, size_t buflen)
{
	pthread_mutex_lock(&_tx_mutex);

	size_t space = TX_BUF_LEN - (_tx_write_idx - _tx_read_idx);

	if (buflen > space) {
		// drop oldest data rather than block MAVLink's sender thread
		size_t to_drop = buflen - space;
		_tx_read_idx += to_drop;
	}

	for (size_t i = 0; i < buflen; i++) {
		_tx_buf[_tx_write_idx % TX_BUF_LEN] = (uint8_t)buffer[i];
		_tx_write_idx++;
	}

	pthread_mutex_unlock(&_tx_mutex);
	return buflen;
}

ssize_t LoraLink::read(struct file *filp, char *buffer, size_t buflen)
{
	pthread_mutex_lock(&_rx_mutex);

	size_t available = _rx_write_idx - _rx_read_idx;

	if (available == 0) {
		pthread_mutex_unlock(&_rx_mutex);
		return -EAGAIN;
	}

	size_t to_copy = available < buflen ? available : buflen;

	for (size_t i = 0; i < to_copy; i++) {
		buffer[i] = (char)_rx_buf[_rx_read_idx % RX_BUF_LEN];
		_rx_read_idx++;
	}

	pthread_mutex_unlock(&_rx_mutex);
	return to_copy;
}

int LoraLink::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case FIONREAD: {
			pthread_mutex_lock(&_rx_mutex);
			*(int *)arg = (int)(_rx_write_idx - _rx_read_idx);
			pthread_mutex_unlock(&_rx_mutex);
			return OK;
		}

	case FIONSPACE: {
			pthread_mutex_lock(&_tx_mutex);
			*(int *)arg = (int)(TX_BUF_LEN - (_tx_write_idx - _tx_read_idx));
			pthread_mutex_unlock(&_tx_mutex);
			return OK;
		}

	default:
		return CDev::ioctl(filp, cmd, arg);
	}
}

pollevent_t LoraLink::poll_state(struct file *filp)
{
	pollevent_t state = 0;

	pthread_mutex_lock(&_rx_mutex);

	if (_rx_write_idx > _rx_read_idx) {
		state |= POLLIN;
	}

	pthread_mutex_unlock(&_rx_mutex);
	pthread_mutex_lock(&_tx_mutex);

	if (TX_BUF_LEN - (_tx_write_idx - _tx_read_idx) > 0) {
		state |= POLLOUT;
	}

	pthread_mutex_unlock(&_tx_mutex);
	return state;
}

int LoraLink::open_first(struct file *filep)
{
	_cdev_used.store(true);
	return CDev::open_first(filep);
}

int LoraLink::close_last(struct file *filep)
{
	_cdev_used.store(false);
	return CDev::close_last(filep);
}

size_t LoraLink::pull_tx(uint8_t *dst, size_t max_len)
{
	pthread_mutex_lock(&_tx_mutex);

	size_t available = _tx_write_idx - _tx_read_idx;
	size_t to_copy = available < max_len ? available : max_len;

	for (size_t i = 0; i < to_copy; i++) {
		dst[i] = _tx_buf[_tx_read_idx % TX_BUF_LEN];
		_tx_read_idx++;
	}

	pthread_mutex_unlock(&_tx_mutex);
	return to_copy;
}

void LoraLink::push_rx(const uint8_t *src, size_t len)
{
	pthread_mutex_lock(&_rx_mutex);

	size_t space = RX_BUF_LEN - (_rx_write_idx - _rx_read_idx);

	if (len > space) {
		// drop oldest unread data on overflow
		_rx_read_idx += (len - space);
	}

	for (size_t i = 0; i < len; i++) {
		_rx_buf[_rx_write_idx % RX_BUF_LEN] = src[i];
		_rx_write_idx++;
	}

	pthread_mutex_unlock(&_rx_mutex);

	if (len > 0) {
		poll_notify(POLLIN);
	}
}
