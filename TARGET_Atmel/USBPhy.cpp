/*
 * Copyright (c) 2018-2019, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "USBPhyHw.h"
#include "cmsis.h"
#include "system.h"
#include "usb_drv.h"

#define MAX_PACKET_SIZE (64)

static USBPhyHw *instance;

USBPhy *get_usb_phy()
{
    static USBPhyHw usbphy;
    return &usbphy;
}

USBPhyHw::USBPhyHw(): events(NULL)
{
}

USBPhyHw::~USBPhyHw()
{
}

extern uint8_t g_sys_init;
struct usb_module usb_device;
uint8_t device_address = 0;
IRQn_Type current_irq_n;

COMPILER_WORD_ALIGNED uint8_t ep_buf[USB_DEVICE_ENDPOINT_NUMBER][2][MAX_PACKET_SIZE];
#define EP_OUT (0)
#define EP_IN  (1)
uint8_t *ep_readreq_buffer[USB_DEVICE_ENDPOINT_NUMBER];

uint8_t *USBPhyHw::endpoint_buffer(usb_ep_t endpoint)
{
    return ep_buf[endpoint & 0xf][endpoint & 0x80];
}

void USBPhyHw::init(USBPhyEvents *events)
{
    this->events = events;

    NVIC_DisableIRQ(USB_OTHER_IRQn);
    NVIC_DisableIRQ(USB_SOF_HSOF_IRQn);
    NVIC_DisableIRQ(USB_TRCPT0_IRQn);
    NVIC_DisableIRQ(USB_TRCPT1_IRQn);

    if (g_sys_init == 0) {
        system_init();
        g_sys_init = 1;
    }

    // Pass instance for usage inside call back
    instance = this;

    struct usb_config config_usb;

    usb_get_config_defaults(&config_usb);
    config_usb.source_generator = GCLK_GENERATOR_1;

    usb_init(&usb_device, USB_REGS, &config_usb);

    usb_enable(&usb_device);

    /* NVIC_SetVector(USB_OTHER_IRQn, (uint32_t)&_usbisr); */

    NVIC_SetPriority(USB_OTHER_IRQn, 2);
    NVIC_SetPriority(USB_SOF_HSOF_IRQn, 2);
    NVIC_SetPriority(USB_TRCPT0_IRQn, 1);
    NVIC_SetPriority(USB_TRCPT1_IRQn, 1);
    NVIC_EnableIRQ(USB_OTHER_IRQn);
    NVIC_EnableIRQ(USB_SOF_HSOF_IRQn);
    NVIC_EnableIRQ(USB_TRCPT0_IRQn);
    NVIC_EnableIRQ(USB_TRCPT1_IRQn);
}

void USBPhyHw::deinit()
{
    // Disconnect and disable interrupt
    disconnect();

    NVIC_DisableIRQ(USB_OTHER_IRQn);
    NVIC_DisableIRQ(USB_SOF_HSOF_IRQn);
    NVIC_DisableIRQ(USB_TRCPT0_IRQn);
    NVIC_DisableIRQ(USB_TRCPT1_IRQn);
}

bool USBPhyHw::powered()
{
    // TODO - return true if powered false otherwise. Devices which don't support
    //    this should always return true
    return true;
}

void USBPhyHw::connect()
{
    usb_device_register_callback(&usb_device, USB_DEVICE_CALLBACK_SUSPEND, (usb_device_callback_t)_usbisr);
    usb_device_register_callback(&usb_device, USB_DEVICE_CALLBACK_SOF, (usb_device_callback_t)_usbisr);
    usb_device_register_callback(&usb_device, USB_DEVICE_CALLBACK_RESET, (usb_device_callback_t)_usbisr);
    usb_device_register_callback(&usb_device, USB_DEVICE_CALLBACK_WAKEUP, (usb_device_callback_t)_usbisr);

    usb_device_enable_callback(&usb_device, USB_DEVICE_CALLBACK_SUSPEND);
    usb_device_enable_callback(&usb_device, USB_DEVICE_CALLBACK_RESET);
    usb_device_enable_callback(&usb_device, USB_DEVICE_CALLBACK_WAKEUP);

    usb_device_attach(&usb_device);
}

void USBPhyHw::disconnect()
{
    // TODO - Disable all endpoints

    // TODO - Clear all endpoint interrupts

    // TODO - Disable pullup on D+
}

void USBPhyHw::configure()
{
    // TODO - set device to configured. Most device will not need this
}

void USBPhyHw::unconfigure()
{
    // TODO - set device to unconfigured. Most device will not need this
}

void USBPhyHw::sof_enable()
{
    usb_device_enable_callback(&usb_device, USB_DEVICE_CALLBACK_SOF);
}

void USBPhyHw::sof_disable()
{
    usb_device_disable_callback(&usb_device, USB_DEVICE_CALLBACK_SOF);
}

void USBPhyHw::set_address(uint8_t address)
{
    device_address = address;
}

void USBPhyHw::remote_wakeup()
{
    // TODO - Sent remote wakeup over USB lines (if supported)
}

const usb_ep_table_t *USBPhyHw::endpoint_table()
{
    static const usb_ep_table_t ep_table = {
        4096 - 32 * 4, // 32 words for endpoint buffers
        // +3 based added to interrupt and isochronous to ensure enough
        // space for 4 byte alignment
        {
            {USB_EP_ATTR_ALLOW_CTRL | USB_EP_ATTR_DIR_IN_AND_OUT, 1, 0},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_ISO | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_ISO | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_ISO | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_ISO | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_INT | USB_EP_ATTR_DIR_IN_AND_OUT,  1, 3},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0},
            {USB_EP_ATTR_ALLOW_BULK | USB_EP_ATTR_DIR_IN_AND_OUT, 2, 0}
        }
    };
    return &ep_table;
}

uint32_t USBPhyHw::ep0_set_max_packet(uint32_t max_packet)
{
    return MAX_PACKET_SIZE;
}

void USBPhyHw::ep0_setup_read_result(uint8_t *buffer, uint32_t size)
{
    uint8_t *rcvbuf = (uint8_t *)usb_descriptor_table.usb_endpoint_table[0].DEVICE_DESC_BANK[0].USB_ADDR;
    memcpy(buffer, rcvbuf, size);
}

void USBPhyHw::ep0_read(uint8_t *data, uint32_t size)
{
    ep_readreq_buffer[0] = data;
    usb_device_endpoint_read_buffer_job(&usb_device, 0, endpoint_buffer(0), MAX_PACKET_SIZE);
}

uint32_t USBPhyHw::ep0_read_result()
{
    uint32_t received_bytes;
    uint8_t *buffer = (uint8_t *)usb_descriptor_table.usb_endpoint_table[0].DEVICE_DESC_BANK[0].USB_ADDR;
    received_bytes = __bits_get(usb_descriptor_table.usb_endpoint_table[0].DEVICE_DESC_BANK[0].USB_PCKSIZE,
                                USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk, USB_DEVICE_PCKSIZE_BYTE_COUNT_Pos);

    memcpy(ep_readreq_buffer[0], buffer, received_bytes);
    return received_bytes;
}

void USBPhyHw::ep0_write(uint8_t *buffer, uint32_t size)
{
    memcpy(endpoint_buffer(0 | USB_EP_DIR_IN), buffer, size);
    usb_device_endpoint_write_buffer_job(&usb_device, 0, endpoint_buffer(0 | USB_EP_DIR_IN), size);
}

void USBPhyHw::ep0_stall()
{
    // TODO - protocol stall endpoint 0. This stall must be automatically
    //        cleared by the next setup packet
}

bool USBPhyHw::endpoint_add(usb_ep_t endpoint, uint32_t max_packet, usb_ep_type_t type)
{
    struct usb_device_endpoint_config config_ep;
    struct endpoint_size_table {
        enum usb_endpoint_size size;
        uint32_t bytes;
    } size_table[] = {
        { USB_ENDPOINT_8_BYTE, 8 },
        { USB_ENDPOINT_16_BYTE, 16 },
        { USB_ENDPOINT_32_BYTE, 32 },
        { USB_ENDPOINT_64_BYTE, 64 },
        { USB_ENDPOINT_128_BYTE, 128 },
        { USB_ENDPOINT_256_BYTE, 256 },
        { USB_ENDPOINT_512_BYTE, 512 },
        { USB_ENDPOINT_1023_BYTE, 1023 }
    };
    int size_id = 0;
    enum usb_device_endpoint_type type_table[] = {
        /* [USB_EP_TYPE_CTRL] */ USB_DEVICE_ENDPOINT_TYPE_CONTROL,
        /* [USB_EP_TYPE_ISO] */ USB_DEVICE_ENDPOINT_TYPE_ISOCHRONOUS,
        /* [USB_EP_TYPE_BULK] */ USB_DEVICE_ENDPOINT_TYPE_BULK,
        /* [USB_EP_TYPE_INT] */ USB_DEVICE_ENDPOINT_TYPE_INTERRUPT,
    };
    int ep = (int)(endpoint & 0xf);

    usb_device_endpoint_get_config_defaults(&config_ep);
    config_ep.ep_address = endpoint;
    config_ep.ep_type = type_table[type];
    if (max_packet > 1023) {
        return false;
    }
    for (int i = 0; i < 8; i++) {
        if (size_table[i].bytes <= max_packet) {
            config_ep.ep_size = size_table[i].size;
            size_id = i;
            break;
        }
    }
    usb_device_endpoint_set_config(&usb_device, &config_ep);

    usb_device_endpoint_register_callback(&usb_device, ep, USB_DEVICE_ENDPOINT_CALLBACK_RXSTP, (usb_device_callback_t)_usbisr);
    usb_device_endpoint_register_callback(&usb_device, ep, USB_DEVICE_ENDPOINT_CALLBACK_TRCPT, (usb_device_callback_t)_usbisr);
    //usb_device_endpoint_register_callback(&usb_device, ep, USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL, (usb_device_callback_t)_usbisr);
    usb_device_endpoint_enable_callback(&usb_device, endpoint, USB_DEVICE_ENDPOINT_CALLBACK_RXSTP);
    usb_device_endpoint_enable_callback(&usb_device, endpoint, USB_DEVICE_ENDPOINT_CALLBACK_TRCPT);
    //usb_device_endpoint_enable_callback(&usb_device, endpoint, USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL);

    return true;
}

void USBPhyHw::endpoint_remove(usb_ep_t endpoint)
{
    // TODO - disable and remove this endpoint
}

void USBPhyHw::endpoint_stall(usb_ep_t endpoint)
{
    // TODO - stall this endpoint until it is explicitly cleared
}

void USBPhyHw::endpoint_unstall(usb_ep_t endpoint)
{
    // TODO - unstall this endpoint
}

bool USBPhyHw::endpoint_read(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    ep_readreq_buffer[endpoint & 0xf] = data;
    usb_device_endpoint_read_buffer_job(&usb_device, endpoint & 0xf, endpoint_buffer(endpoint), MAX_PACKET_SIZE);
    return true;
}

uint32_t USBPhyHw::endpoint_read_result(usb_ep_t endpoint)
{
    int ep = endpoint & 0xf;
    uint32_t received_bytes;
    uint8_t *buffer = (uint8_t *)usb_descriptor_table.usb_endpoint_table[ep].DEVICE_DESC_BANK[0].USB_ADDR;
    received_bytes = __bits_get(usb_descriptor_table.usb_endpoint_table[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE,
                                USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk, USB_DEVICE_PCKSIZE_BYTE_COUNT_Pos);
    memcpy(ep_readreq_buffer[ep], buffer, received_bytes);
    return received_bytes;
}

bool USBPhyHw::endpoint_write(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    memcpy(endpoint_buffer(endpoint), data, size);
    usb_device_endpoint_write_buffer_job(&usb_device, endpoint & 0xf, endpoint_buffer(endpoint), size);
    return true;
}

void USBPhyHw::endpoint_abort(usb_ep_t endpoint)
{
    // TODO - stop the current transfer on this endpoint and don't call the IN or OUT callback
}

void USBPhyHw::process()
{
    uint16_t epintsmry;
    epintsmry = usb_device.hw->DEVICE.USB_EPINTSMRY;

    for (uint8_t i = 0; i < USB_DEVICE_ENDPOINT_NUMBER; i++) {
        if (epintsmry & (1 << i)) {
            if (current_irq_n == USB_TRCPT0_IRQn) {
                if (usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_TRCPT0(1)) {
                    usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT0(1);

                    if (i == 0) {
                      instance->events->ep0_out();
                    } else {
                        instance->events->out(i);
                    }
                }
            }

            if (current_irq_n == USB_TRCPT1_IRQn) {
                if (usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_TRCPT1(1)) {
                    usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT1(1);

                    if (device_address != (usb_device.hw->DEVICE.USB_DADD & USB_DEVICE_DADD_DADD_Msk)) {
                        usb_device_set_address(&usb_device, device_address);
                    }

                    if (i == 0) {
                        instance->events->ep0_in();
                    } else {
                        instance->events->in(0x80 | i);
                    }
                }
            }
        }
    }

    if (current_irq_n == USB_SOF_HSOF_IRQn) {
        usb_device.hw->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_SOF(1);

        // SOF event, read frame number
        uint16_t frame = usb_device_get_frame_number(&usb_device);
        instance->events->sof(frame);
    }

    if (current_irq_n == USB_OTHER_IRQn) {
        if (0 == epintsmry) {
            uint16_t intflag, intpending;
            intflag = usb_device.hw->DEVICE.USB_INTFLAG;
            intpending = intflag & usb_device.hw->DEVICE.USB_INTENSET;

            if (intpending & USB_DEVICE_INTFLAG_EORST(1)) {
                usb_device.hw->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_EORST(1);

                device_address = 0;
                usb_device_disable_callback(&usb_device, USB_DEVICE_CALLBACK_SUSPEND);
                usb_device_disable_callback(&usb_device, USB_DEVICE_CALLBACK_WAKEUP);

                usb_device_endpoint_abort_job(&usb_device, 0 | USB_EP_DIR_IN);
                usb_device_endpoint_abort_job(&usb_device, 0 | USB_EP_DIR_OUT);

                struct usb_device_endpoint_config config_ep0;
                usb_device_endpoint_get_config_defaults(&config_ep0);
                config_ep0.ep_size = USB_ENDPOINT_64_BYTE;
                usb_device_endpoint_set_config(&usb_device, &config_ep0);

                usb_device_endpoint_setup_buffer_job(&usb_device, endpoint_buffer(0));

                usb_device_endpoint_register_callback(&usb_device, 0, USB_DEVICE_ENDPOINT_CALLBACK_RXSTP, (usb_device_callback_t)_usbisr);
                usb_device_endpoint_register_callback(&usb_device, 0, USB_DEVICE_ENDPOINT_CALLBACK_TRCPT, (usb_device_callback_t)_usbisr);
                //usb_device_endpoint_register_callback(&usb_device, 0, USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL, (usb_device_callback_t)_usbisr);
                usb_device_endpoint_enable_callback(&usb_device, 0, USB_DEVICE_ENDPOINT_CALLBACK_RXSTP);
                usb_device_endpoint_enable_callback(&usb_device, 0, USB_DEVICE_ENDPOINT_CALLBACK_TRCPT);
                //usb_device_endpoint_enable_callback(&usb_device, 0, USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL);
                usb_device.hw->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_Msk;

                // reset bus for USBDevice layer
                instance->events->reset();
                instance->events->power(true);

                // Re-enable interrupt
                NVIC_ClearPendingIRQ(USB_OTHER_IRQn);
                NVIC_ClearPendingIRQ(USB_SOF_HSOF_IRQn);
                NVIC_ClearPendingIRQ(USB_TRCPT0_IRQn);
                NVIC_ClearPendingIRQ(USB_TRCPT1_IRQn);

                NVIC_EnableIRQ(USB_OTHER_IRQn);
                NVIC_EnableIRQ(USB_SOF_HSOF_IRQn);
                NVIC_EnableIRQ(USB_TRCPT0_IRQn);
                NVIC_EnableIRQ(USB_TRCPT1_IRQn);
                return;
            }

            if (intpending & USB_DEVICE_INTFLAG_SUSPEND(1)) {
                usb_device.hw->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_SUSPEND(1);
                instance->events->suspend(true);
            }

            if (intpending & (USB_DEVICE_INTFLAG_WAKEUP(1) | USB_DEVICE_INTFLAG_EORSM(1) | USB_DEVICE_INTFLAG_UPRSM(1))) {
                usb_device.hw->DEVICE.USB_INTFLAG = (USB_DEVICE_INTFLAG_WAKEUP(1) | USB_DEVICE_INTFLAG_EORSM(1) | USB_DEVICE_INTFLAG_UPRSM(1));
                instance->events->suspend(false);
            }

        } else {
            for (uint8_t i = 0; i < USB_DEVICE_ENDPOINT_NUMBER; i++) {
                if (epintsmry & (1 << i)) {
                    uint8_t epintflag, epintpending;
                    epintflag = usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG;
                    epintpending = epintflag &
                        usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTENSET;

                    if (epintpending & USB_DEVICE_EPINTFLAG_STALL1(1)) {
                        usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_STALL1(1);
                    }

                    if (epintpending & USB_DEVICE_EPINTFLAG_STALL0(1)) {
                        usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_STALL0(1);
                    }

                    if (epintpending & USB_DEVICE_EPINTFLAG_RXSTP(1)) {
                        usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_RXSTP(1);
                        instance->events->ep0_setup();

                        usb_device_endpoint_setup_buffer_job(&usb_device, endpoint_buffer(0));
                    }

                    if (epintpending & USB_DEVICE_EPINTFLAG_TRFAIL1(1)) {
                        usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRFAIL1(1);
                        if (usb_descriptor_table.usb_endpoint_table[i].DEVICE_DESC_BANK[1].USB_STATUS_BK & USB_DEVICE_STATUS_BK_ERRORFLOW(1)) {
                            usb_descriptor_table.usb_endpoint_table[i].DEVICE_DESC_BANK[1].USB_STATUS_BK &= ~USB_DEVICE_STATUS_BK_ERRORFLOW_Msk;
                        }

                        usb_device_endpoint_abort_job(&usb_device, 0x80 | i);
                    }

                    if(epintpending & USB_DEVICE_EPINTFLAG_TRFAIL0(1)) {
                        usb_device.hw->DEVICE.DEVICE_ENDPOINT[i].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRFAIL0(1);
                        if (usb_descriptor_table.usb_endpoint_table[i].DEVICE_DESC_BANK[0].USB_STATUS_BK & USB_DEVICE_STATUS_BK_ERRORFLOW(1)) {
                            usb_descriptor_table.usb_endpoint_table[i].DEVICE_DESC_BANK[0].USB_STATUS_BK &= ~USB_DEVICE_STATUS_BK_ERRORFLOW_Msk;
                        }

                        usb_device_endpoint_read_buffer_job(&usb_device, 0, NULL, 64);
                    }
                }
            }
        }
    }

    NVIC_EnableIRQ(USB_OTHER_IRQn);
    NVIC_EnableIRQ(USB_SOF_HSOF_IRQn);
    NVIC_EnableIRQ(USB_TRCPT0_IRQn);
    NVIC_EnableIRQ(USB_TRCPT1_IRQn);
}

void USBPhyHw::_usbisr(IRQn_Type irq_n)
{
    current_irq_n = irq_n;

    NVIC_DisableIRQ(USB_OTHER_IRQn);
    NVIC_DisableIRQ(USB_SOF_HSOF_IRQn);
    NVIC_DisableIRQ(USB_TRCPT0_IRQn);
    NVIC_DisableIRQ(USB_TRCPT1_IRQn);

    instance->events->start_process();
}
