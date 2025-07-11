#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_test);

int main(void)
{
	const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(usart1));
	if (!device_is_ready(uart)) {
		LOG_ERR("UART device not ready");
		return;
	}

	LOG_INF("UART test started, sending 'T' every second");

	while (1) {
		uart_poll_out(uart, 'T');
		LOG_INF("Sent: T");
		k_msleep(1000);
	}
	return 0;
}
