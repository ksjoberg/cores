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

#ifndef USBecm_h_
#define USBecm_h_

#include "usb_desc.h"

#if defined(ECM_DATA_INTERFACE) 
#include <stdbool.h>

#define ETH_HEADER_SIZE             14
#define ECM_MTU                     (ECM_MAX_SEGMENT_SIZE - ETH_HEADER_SIZE)

// C language implementation
#ifdef __cplusplus
extern "C" {
#endif

void usb_ecm_configure(void);
void usb_ecm_interface_callback(int interface);
const void* usb_ecm_read(size_t *len);
void usb_ecm_read_done(void);
void ecm_report(bool nc);

bool usb_ecm_can_transmit(void);
void* usb_ecm_write_start(size_t *buffer_size);
void usb_ecm_write_done(size_t len);

#ifdef __cplusplus
}
#endif


#endif // ECM_DATA_INTERFACE

#endif // USBecm_h_

