/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdbool.h>
#include "usb_dev.h"
#include "usb_desc.h"
#include "usb_ecm_internal.h"
#include "usb_ecm.h"
#include "core_pins.h"// for delay()
#include <string.h> // for memcpy()
#include "avr/pgmspace.h" // for PROGMEM, DMAMEM, FASTRUN

#include "debug/printf.h"
#include "util/fifo.h"
#include "core_pins.h"

// defined by usb_desc.h
#if defined(ECM_STATUS_INTERFACE)

struct usb_ecm_counters {
       uint32_t rx_error;
       uint32_t rx;
       uint32_t tx_error;
       uint32_t tx;
};

#define USB_ETH_MTU 1440

//#if F_CPU >= 20000000

// At very slow CPU speeds, the OCRAM just isn't fast enough for
// USB to work reliably.  But the precious/limited DTCM is.  So
// as an ugly workaround, undefine DMAMEM so all buffers which
// would normally be allocated in OCRAM are placed in DTCM.
#if defined(F_CPU) && F_CPU < 30000000
#undef DMAMEM
#define DMAMEM
#endif

extern volatile uint8_t usb_high_speed;
extern volatile uint8_t usb_configuration; // non-zero when USB host as configured device

int usb_eth_is_active = 1;
struct usb_ecm_counters ecm_counters;

#define RX_NUM   4
#define RX_SIZE ECM_RX_SIZE_480

static struct fifo_cnt ecm_data_rx_fifo = FIFO_INIT(RX_NUM);

static transfer_t rx_transfer[RX_NUM] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[RX_NUM * RX_SIZE] __attribute__ ((aligned(32)));
static uint16_t usb_packet_size = 0;

#define TX_NUM   4
#define TX_SIZE ECM_TX_SIZE_480

static transfer_t tx_transfer[TX_NUM] __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t txbuffer[TX_NUM * TX_SIZE] __attribute__ ((aligned(32)));

static struct fifo_cnt tx_transfer_fifo = FIFO_INIT(TX_NUM); // contains transfers in use

#define NUM_NOTIFY_XFERS 4
static transfer_t notify_xfer[NUM_NOTIFY_XFERS] __attribute__ ((used, aligned(32)));
static size_t current_notify_xfer;

//Do we have data to send back?
bool ecm_initialized = false;

//   --- forward declarations ---

static void rx_queue_transfer(int i);
static void rx_event(transfer_t *t);
static void tx_event(transfer_t *t);
static void usb_ecm_send_notify(uint8_t *data, int size);

//   --- end of forward declarations ---

static inline uint8_t* get_rx_buffer(uint16_t index) {
	return rx_buffer + (RX_SIZE * index);
}

static  __attribute__ ((aligned(32))) usb_request_t notify_nc = 
{
  .bmRequestType = 0xA1,
  .bRequest = 0 /* NETWORK_CONNECTION */,
  .wValue = 1 /* Connected */,
  .wIndex = ECM_STATUS_INTERFACE,
  .wLength = 0,
};
static __attribute__ ((aligned(32))) struct
{
  usb_request_t header;
  uint32_t downlink, uplink;
} notify_csc = 
{
  .header =
  {
    .bmRequestType = 0xA1,
    .bRequest = 0x2A /* CONNECTION_SPEED_CHANGE */,
    .wIndex = ECM_STATUS_INTERFACE,
    .wLength = 0,
  },
  .downlink = 9728000,
  .uplink = 9728000,
};


void usb_ecm_configure(void) {

	printf("usb_ecm_configure\n");
	if (usb_high_speed) {
		usb_packet_size = ECM_RX_SIZE_480;
	} else {
		usb_packet_size = ECM_RX_SIZE_12;
	}

	memset(rx_transfer, 0, sizeof(rx_transfer));
	memset(tx_transfer, 0, sizeof(tx_transfer));
	memset(notify_xfer, 0, sizeof(notify_xfer));

	usb_config_tx(ECM_NOTIFY_ENDPOINT, ECM_ACM_SIZE, 1, NULL); // size same 12 & 480
	usb_config_rx(ECM_RX_ENDPOINT, usb_packet_size, 0, rx_event);
	usb_config_tx(ECM_TX_ENDPOINT, usb_packet_size, 0, tx_event);
	for (int i = 0; i < RX_NUM; i++)
		rx_queue_transfer(i);
}

void ecm_report(bool nc)
{
  if (nc)
  {
    /* provide a Management Element Notification with a NETWORK_CONNECTION status */
    usb_ecm_send_notify((uint8_t *)&notify_nc, sizeof(notify_nc));
  }
  else
  {
    /* provide a Management Element Notification with a CONNECTION_SPEED_CHANGE status */
    usb_ecm_send_notify((uint8_t *)&notify_csc, sizeof(notify_csc));
  }
}

void usb_ecm_interface_callback(int interface)
{
	if (interface)
		ecm_report(true);
}

/*************************************************************************/
/**                               Receive                               **/
/*************************************************************************/

static void rx_queue_transfer(int i) {
	NVIC_DISABLE_IRQ(IRQ_USB1);
	//printf("rx queue i=%d\n", i);
	void *buffer = get_rx_buffer(i);
	usb_prepare_transfer(rx_transfer + i, buffer, RX_SIZE, i);
	arm_dcache_delete(buffer, RX_SIZE);
	usb_receive(ECM_RX_ENDPOINT, rx_transfer + i);
	NVIC_ENABLE_IRQ(IRQ_USB1);
}

// called by USB interrupt when any packet is received
static void rx_event(transfer_t *t) {
	int len = RX_SIZE - ((t->status >> 16) & 0x7FFF);
	int i = t->callback_param;

	if (fifo_remaining(&ecm_data_rx_fifo) == 0)
	{
		printf("rx full!\n");
		return;
	}
	//printf("rx event, len=%d, i=%d\n", len, i);
	fifo_add(&ecm_data_rx_fifo, 1);
	if (1)
		printf("len=%d i=%d rxi=%d\n", len, i, fifo_write_index(&ecm_data_rx_fifo));
}

const void* usb_ecm_read(size_t *len) {

	if (!usb_configuration)
		return NULL;

	if (fifo_level(&ecm_data_rx_fifo) == 0)
		return NULL;

	size_t read_index = fifo_read_index(&ecm_data_rx_fifo);
	transfer_t *xfer = &rx_transfer[read_index];
	uint32_t status = usb_transfer_status(xfer);
	if (!(status & 0x80)) {
		if (status & 0x68) {
			goto rx_error;
		}
	} else {
		// still busy?
		return NULL; // should never happen
	}


	int xfer_len = RX_SIZE - ((xfer->status >> 16) & 0x7FFF);
	printf("rrr %d %d %d\n", xfer->callback_param, read_index, xfer_len);

	const uint8_t *buf = get_rx_buffer(read_index);
	*len = xfer_len;
	return buf;

	rx_error: // error above
	usb_ecm_read_done();
	ecm_counters.rx--; // fix counter value
	ecm_counters.rx_error++;
	return NULL;
}

void usb_ecm_read_done(void) {
	size_t read_index = fifo_read_index(&ecm_data_rx_fifo);
	rx_queue_transfer(read_index);
	ecm_counters.rx++;
	fifo_remove(&ecm_data_rx_fifo, 1);
}

/*************************************************************************/
/**                               Transmit                              **/
/*************************************************************************/
// called by USB interrupt when any packet is finished transmitting
static void tx_event(transfer_t *t)
{
	//void(t);
	fifo_remove(&tx_transfer_fifo, 1);
}

bool usb_ecm_can_transmit(void)
{
	if (fifo_remaining(&tx_transfer_fifo) == 0) {
		return false;
	}
	return true;
}

void* usb_ecm_write_start(size_t *buffer_size) {

	if (!usb_configuration)
		return NULL;

	if (fifo_remaining(&tx_transfer_fifo) == 0) {
		/*
		size_t read_index = fifo_read_index(&tx_transfer_fifo);
		transfer_t *trans = &tx_transfer[read_index];
		uint32_t status = usb_transfer_status(trans);
		if (!(status & 0x80)) {
			if (status & 0x68) {
				ecm_counters.tx_error++;
			}
			printf("release txbuf %d\n", read_index);
			fifo_remove(&tx_transfer_fifo, 1);
		} else {
			printf("out of buffers");
			return NULL;
		}
		*/
		printf("tx:out of buffers");
		return NULL;
	}

	*buffer_size = TX_SIZE;
	// now there should be a free buffer
	return txbuffer + fifo_write_index(&tx_transfer_fifo) * TX_SIZE;
}

void usb_ecm_write_done(size_t len) {

	size_t write_index = fifo_write_index(&tx_transfer_fifo);
	void *buf = txbuffer + write_index * TX_SIZE;


	transfer_t *trans = &tx_transfer[write_index];

	printf("tx %d\n", write_index);
	NVIC_DISABLE_IRQ(IRQ_USB1);
	usb_prepare_transfer(trans, buf, len, 0);
	arm_dcache_flush_delete(buf, len);
	usb_transmit(ECM_TX_ENDPOINT, trans);
	NVIC_ENABLE_IRQ(IRQ_USB1);

	fifo_add(&tx_transfer_fifo, 1);
	ecm_counters.tx++;
}




/**
 * \brief Send an interrupt over the interrupt endpoint to the host.
 */
static void usb_ecm_send_notify(uint8_t *data, int size) {

	static DMAMEM __attribute__ ((aligned(32))) uint8_t resp_available[ECM_ACM_SIZE];

	if (size > ECM_ACM_SIZE)
	{
		printf("ERROR send intr: too large %d\n", size);
		return;
	}

	transfer_t *xfer = &notify_xfer[current_notify_xfer++];
	if (current_notify_xfer == NUM_NOTIFY_XFERS)
		current_notify_xfer = 0;

	uint32_t status = usb_transfer_status(xfer);
	if (!(status & 0x80)) {
		if (status & 0x68) {
			printf("ERROR send intr: status = %x, ms=%u\n", status, systick_millis_count);
		}
	} else {
		// previous transfer pending
		printf("previous transfer pending\n");
		return;
	}

	//	printf("INTR\n");
	// dma memory must by initialized dynamically
	memcpy(resp_available, data, size);

	usb_prepare_transfer(xfer, resp_available, size, 0);
	arm_dcache_flush_delete(resp_available, size);
	usb_transmit(ECM_NOTIFY_ENDPOINT, xfer);
}


#endif
