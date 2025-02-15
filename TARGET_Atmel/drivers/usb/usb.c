/**
 * \file
 *
 * \brief SAM USB Driver.
 *
 * Copyright (C) 2014-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <string.h>
#include "cmsis.h"
#include "usb_drv.h"

/** Fields definition from a LPM TOKEN  */
#define  USB_LPM_ATTRIBUT_BLINKSTATE_MASK      (0xF << 0)
#define  USB_LPM_ATTRIBUT_HIRD_MASK            (0xF << 4)
#define  USB_LPM_ATTRIBUT_REMOTEWAKE_MASK      (1 << 8)
#define  USB_LPM_ATTRIBUT_BLINKSTATE(value)    ((value & 0xF) << 0)
#define  USB_LPM_ATTRIBUT_HIRD(value)          ((value & 0xF) << 4)
#define  USB_LPM_ATTRIBUT_REMOTEWAKE(value)    ((value & 1) << 8)
#define  USB_LPM_ATTRIBUT_BLINKSTATE_L1        USB_LPM_ATTRIBUT_BLINKSTATE(1)

/**
 * \name USB SRAM data containing pipe descriptor table
 * The content of the USB SRAM can be :
 * - modified by USB hardware interface to update pipe status.
 *   Thereby, it is read by software.
 * - modified by USB software to control pipe.
 *   Thereby, it is read by hardware.
 * This data section is volatile.
 *
 * @{
 */
COMPILER_PACK_SET(1)
COMPILER_WORD_ALIGNED union usb_descriptor_table_t usb_descriptor_table;
COMPILER_PACK_RESET()
/** @} */

/**
 * \brief Local USB module instance
 */
static struct usb_module *_usb_instances;

/**
 * \brief Host pipe callback structure variable
 */
static struct usb_pipe_callback_parameter pipe_callback_para;

/* Device LPM callback variable */
static uint32_t device_callback_lpm_wakeup_enable;

/**
 * \brief Device endpoint callback parameter variable, used to transfer info to UDD wrapper layer
 */
static struct usb_endpoint_callback_parameter ep_callback_para;

/**
 * \internal USB Device IRQ Mask Bits Map
 */
static const uint16_t _usb_device_irq_bits[USB_DEVICE_CALLBACK_N] = {
    USB_DEVICE_INTFLAG_SOF(1),
    USB_DEVICE_INTFLAG_EORST(1),
    USB_DEVICE_INTFLAG_WAKEUP(1) | USB_DEVICE_INTFLAG_EORSM(1) | USB_DEVICE_INTFLAG_UPRSM(1),
    USB_DEVICE_INTFLAG_RAMACER(1),
    USB_DEVICE_INTFLAG_SUSPEND(1),
    USB_DEVICE_INTFLAG_LPMNYET(1),
    USB_DEVICE_INTFLAG_LPMSUSP(1),
};

/**
 * \internal USB Device IRQ Mask Bits Map
 */
static const uint8_t _usb_endpoint_irq_bits[USB_DEVICE_EP_CALLBACK_N] = {
    USB_DEVICE_EPINTFLAG_TRCPT_Msk,
    USB_DEVICE_EPINTFLAG_TRFAIL_Msk,
    USB_DEVICE_EPINTFLAG_RXSTP(1),
    USB_DEVICE_EPINTFLAG_STALL_Msk
};

/**
 * \brief Bit mask for pipe job busy status
 */
uint32_t host_pipe_job_busy_status = 0;

/**
 * \brief Registers a USB host callback
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref usb_host_enable_callback,
 * in order for the interrupt handler to call it when the conditions for the
 * callback type is met.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 * \param[in]     callback_func Pointer to callback function
 *
 * \return Status of the registration operation.
 * \retval STATUS_OK    The callback was registered successfully.
 */
enum status_code usb_host_register_callback(struct usb_module *module_inst,
        enum usb_host_callback callback_type,
        usb_host_callback_t callback_func)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(callback_func);

    /* Register callback function */
    module_inst->host_callback[callback_type] = callback_func;

    /* Set the bit corresponding to the callback_type */
    module_inst->host_registered_callback_mask |= (1 << callback_type);

    return STATUS_OK;
}

/**
 * \brief Unregisters a USB host callback
 *
 * Unregisters an asynchronous callback implemented by the user. Removing it
 * from the internal callback registration table.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the de-registration operation.
 * \retval STATUS_OK    The callback was unregistered successfully.
 */
enum status_code usb_host_unregister_callback(struct usb_module *module_inst,
        enum usb_host_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);

    /* Unregister callback function */
    module_inst->host_callback[callback_type] = NULL;

    /* Clear the bit corresponding to the callback_type */
    module_inst->host_registered_callback_mask &= ~(1 << callback_type);

    return STATUS_OK;
}

/**
 * \brief Enables USB host callback generation for a given type.
 *
 * Enables asynchronous callbacks for a given logical type.
 * This must be called before USB host generate callback events.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback enable operation.
 * \retval STATUS_OK    The callback was enabled successfully.
 */
enum status_code usb_host_enable_callback(struct usb_module *module_inst,
        enum usb_host_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* Enable callback */
    module_inst->host_enabled_callback_mask |= (1 << callback_type);

    if (callback_type == USB_HOST_CALLBACK_SOF) {
        module_inst->hw->HOST.USB_INTENSET = USB_HOST_INTENSET_HSOF(1);
    }
    if (callback_type == USB_HOST_CALLBACK_RESET) {
        module_inst->hw->HOST.USB_INTENSET = USB_HOST_INTENSET_RST(1);
    }
    if (callback_type == USB_HOST_CALLBACK_WAKEUP) {
        module_inst->hw->HOST.USB_INTENSET = USB_HOST_INTENSET_WAKEUP(1);
    }
    if (callback_type == USB_HOST_CALLBACK_DNRSM) {
        module_inst->hw->HOST.USB_INTENSET = USB_HOST_INTENSET_DNRSM(1);
    }
    if (callback_type == USB_HOST_CALLBACK_UPRSM) {
        module_inst->hw->HOST.USB_INTENSET = USB_HOST_INTENSET_UPRSM(1);
    }
    if (callback_type == USB_HOST_CALLBACK_RAMACER) {
        module_inst->hw->HOST.USB_INTENSET = USB_HOST_INTENSET_RAMACER(1);
    }
    if (callback_type == USB_HOST_CALLBACK_CONNECT) {
        module_inst->hw->HOST.USB_INTENSET = USB_HOST_INTENSET_DCONN(1);
    }
    if (callback_type == USB_HOST_CALLBACK_DISCONNECT) {
        module_inst->hw->HOST.USB_INTENSET = USB_HOST_INTENSET_DDISC(1);
    }

    return STATUS_OK;
}

/**
 * \brief Disables USB host callback generation for a given type.
 *
 * Disables asynchronous callbacks for a given logical type.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback disable operation.
 * \retval STATUS_OK    The callback was disabled successfully.
 */
enum status_code usb_host_disable_callback(struct usb_module *module_inst,
        enum usb_host_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* Disable callback */
    module_inst->host_enabled_callback_mask &= ~(1 << callback_type);

    if (callback_type == USB_HOST_CALLBACK_SOF) {
        module_inst->hw->HOST.USB_INTENCLR = USB_HOST_INTENCLR_HSOF(1);
    }
    if (callback_type == USB_HOST_CALLBACK_RESET) {
        module_inst->hw->HOST.USB_INTENCLR = USB_HOST_INTENCLR_RST(1);
    }
    if (callback_type == USB_HOST_CALLBACK_WAKEUP) {
        module_inst->hw->HOST.USB_INTENCLR = USB_HOST_INTENCLR_WAKEUP(1);
    }
    if (callback_type == USB_HOST_CALLBACK_DNRSM) {
        module_inst->hw->HOST.USB_INTENCLR = USB_HOST_INTENCLR_DNRSM(1);
    }
    if (callback_type == USB_HOST_CALLBACK_UPRSM) {
        module_inst->hw->HOST.USB_INTENCLR = USB_HOST_INTENCLR_UPRSM(1);
    }
    if (callback_type == USB_HOST_CALLBACK_RAMACER) {
        module_inst->hw->HOST.USB_INTENCLR = USB_HOST_INTENCLR_RAMACER(1);
    }
    if (callback_type == USB_HOST_CALLBACK_CONNECT) {
        module_inst->hw->HOST.USB_INTENCLR = USB_HOST_INTENCLR_DCONN(1);
    }
    if (callback_type == USB_HOST_CALLBACK_DISCONNECT) {
        module_inst->hw->HOST.USB_INTENCLR = USB_HOST_INTENCLR_DDISC(1);
    }

    return STATUS_OK;
}

/**
 * \brief Initializes an USB host pipe configuration structure to defaults.
 *
 * Initializes a given USB host pipe configuration structure to a
 * set of known default values. This function should be called on all new
 * instances of these configuration structures before being modified by the
 * user application.
 *
 * The default configuration is as follows:
 * \li device address is 0
 * \li endpoint address is 0
 * \li pipe type is control
 * \li interval is 0
 * \li pipe size is 8
 *
 * \param[out] ep_config  Configuration structure to initialize to default values
 */
void usb_host_pipe_get_config_defaults(struct usb_host_pipe_config *ep_config)
{
    /* Sanity check arguments */
    Assert(ep_config);
    /* Write default config to config struct */
    ep_config->device_address = 0;
    ep_config->endpoint_address = 0;
    ep_config->pipe_type = USB_HOST_PIPE_TYPE_CONTROL;
    ep_config->binterval = 0;
    ep_config->size = 8;
}

/**
 * \brief Writes an USB host pipe configuration to the hardware module.
 *
 * Writes out a given configuration of an USB host pipe
 * configuration to the hardware module. If the pipe is already configured,
 * the new configuration will replace the existing one.
 *
 * \param[in] module_inst    Pointer to USB software instance struct
 * \param[in] pipe_num       Pipe to configure
 * \param[in] ep_config      Configuration settings for the pipe
 *
 * \return Status of the host pipe configuration operation.
 * \retval STATUS_OK    The host pipe was configured successfully.
 */
enum status_code usb_host_pipe_set_config(struct usb_module *module_inst, uint8_t pipe_num,
        struct usb_host_pipe_config *ep_config)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);
    Assert(ep_config);

    /* set pipe config */
    module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG &= ~(USB_HOST_PCFG_BK_Msk | USB_HOST_PCFG_PTYPE_Msk);
    module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG |= ep_config->pipe_type;
    module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_BINTERVAL =
            ep_config->binterval;
    if (ep_config->endpoint_address == 0) {
        __bits_set(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG,
                   USB_HOST_PCFG_PTOKEN_Msk, USB_HOST_PCFG_PTOKEN(USB_HOST_PIPE_TOKEN_SETUP));
    } else if (ep_config->endpoint_address & USB_EP_DIR_IN) {
        __bits_set(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG,
                   USB_HOST_PCFG_PTOKEN_Msk, USB_HOST_PCFG_PTOKEN(USB_HOST_PIPE_TOKEN_IN));
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PSTATUSSET =
                USB_HOST_PSTATUSSET_BK0RDY(1);
    } else {
        __bits_set(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG,
                   USB_HOST_PCFG_PTOKEN_Msk, USB_HOST_PCFG_PTOKEN(USB_HOST_PIPE_TOKEN_OUT));
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PSTATUSCLR =
                USB_HOST_PSTATUSCLR_BK0RDY(1);
    }

    memset((uint8_t *)&usb_descriptor_table.usb_pipe_table[pipe_num], 0,
            sizeof(usb_descriptor_table.usb_pipe_table[0]));
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_CTRL_PIPE,
               USB_HOST_CTRL_PIPE_PDADDR_Msk, USB_HOST_CTRL_PIPE_PDADDR(ep_config->device_address));
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_CTRL_PIPE,
               USB_HOST_CTRL_PIPE_PEPNUM_Msk, USB_HOST_CTRL_PIPE_PEPNUM(ep_config->endpoint_address & USB_EP_ADDR_MASK));
    if (ep_config->size == 1023) {
        __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
                   USB_HOST_PCKSIZE_SIZE_Msk, USB_HOST_PCKSIZE_SIZE(0x07));
    } else {
        __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
                   USB_HOST_PCKSIZE_SIZE_Msk, USB_HOST_PCKSIZE_SIZE(32 - clz(((uint32_t)min(max(ep_config->size, 8), 1024) << 1) - 1) - 1 - 3));
    }

    /* Clear busy status */
    host_pipe_job_busy_status &= ~(1 << pipe_num);

    return STATUS_OK;
}

/**
 * \brief Gets an USB host pipe configuration.
 *
 * Gets out the configuration of an USB host pipe from the hardware module.
 *
 * \param[in] module_inst    Pointer to USB software instance struct
 * \param[in] pipe_num       Pipe to configure
 * \param[out] ep_config     Configuration settings for the pipe
 *
 * \return Status of the get host pipe configuration operation.
 * \retval STATUS_OK    The host pipe configuration was read successfully.
 */
enum status_code usb_host_pipe_get_config(struct usb_module *module_inst, uint8_t pipe_num,
        struct usb_host_pipe_config *ep_config)
{
    uint32_t size;

    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);
    Assert(ep_config);

    /* get pipe config from setting register */
    ep_config->device_address =
        (usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_CTRL_PIPE & USB_HOST_CTRL_PIPE_PDADDR_Msk) >> USB_HOST_CTRL_PIPE_PDADDR_Pos;
    ep_config->endpoint_address =
            __bits_get(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_CTRL_PIPE,
                       USB_HOST_CTRL_PIPE_PEPNUM_Msk, USB_HOST_CTRL_PIPE_PEPNUM_Pos);

    if (__bits_get(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG, USB_HOST_PCFG_PTOKEN_Msk, USB_HOST_PCFG_PTOKEN_Pos) ==
                USB_HOST_PIPE_TOKEN_IN) {
        ep_config->endpoint_address |= USB_EP_DIR_IN;
    }

    ep_config->pipe_type = (enum usb_host_pipe_type)__bits_get(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG,
                                                               USB_HOST_PCFG_PTYPE_Msk, USB_HOST_PCFG_PTOKEN_Pos);
    ep_config->binterval =
            module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_BINTERVAL;
    size = __bits_get(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
                      USB_HOST_PCKSIZE_SIZE_Msk, USB_HOST_PCKSIZE_SIZE_Pos);
    if (size == 0x07) {
        ep_config->size = 1023;
    } else {
        ep_config->size = (8 << size);
    }

    return STATUS_OK;
}

/**
 * \brief Registers a USB host pipe callback
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref usb_host_pipe_enable_callback,
 * in order for the interrupt handler to call it when the conditions for the
 * callback type is met.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 * \param[in]     callback_type Callback type given by an enum
 * \param[in]     callback_func Pointer to callback function
 *
 * \return Status of the registration operation.
 * \retval STATUS_OK    The callback was registered successfully.
 */
enum status_code usb_host_pipe_register_callback(
        struct usb_module *module_inst, uint8_t pipe_num,
        enum usb_host_pipe_callback callback_type,
        usb_host_pipe_callback_t callback_func)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);
    Assert(callback_func);

    /* Register callback function */
    module_inst->host_pipe_callback[pipe_num][callback_type] = callback_func;

    /* Set the bit corresponding to the callback_type */
    module_inst->host_pipe_registered_callback_mask[pipe_num] |= (1 << callback_type);

    return STATUS_OK;
}

/**
 * \brief Unregisters a USB host pipe callback
 *
 * Unregisters an asynchronous callback implemented by the user. Removing it
 * from the internal callback registration table.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the de-registration operation.
 * \retval STATUS_OK    The callback was unregistered successfully.
 */
enum status_code usb_host_pipe_unregister_callback(
        struct usb_module *module_inst, uint8_t pipe_num,
        enum usb_host_pipe_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);

    /* Unregister callback function */
    module_inst->host_pipe_callback[pipe_num][callback_type] = NULL;

    /* Clear the bit corresponding to the callback_type */
    module_inst->host_pipe_registered_callback_mask[pipe_num] &= ~(1 << callback_type);

    return STATUS_OK;
}

/**
 * \brief Enables USB host pipe callback generation for a given type.
 *
 * Enables asynchronous callbacks for a given logical type.
 * This must be called before USB host pipe generate callback events.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback enable operation.
 * \retval STATUS_OK    The callback was enabled successfully.
 */
enum status_code usb_host_pipe_enable_callback(
        struct usb_module *module_inst, uint8_t pipe_num,
        enum usb_host_pipe_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);

    /* Enable callback */
    module_inst->host_pipe_enabled_callback_mask[pipe_num] |= (1 << callback_type);

    if (callback_type == USB_HOST_PIPE_CALLBACK_TRANSFER_COMPLETE) {
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PINTENSET = USB_HOST_PINTENSET_TRCPT_Msk;
    }
    if (callback_type == USB_HOST_PIPE_CALLBACK_ERROR) {
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PINTENSET =
                USB_HOST_PINTENSET_TRFAIL(1) | USB_HOST_PINTENSET_PERR(1);
    }
    if (callback_type == USB_HOST_PIPE_CALLBACK_SETUP) {
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PINTENSET = USB_HOST_PINTENSET_TXSTP(1);
    }
    if (callback_type == USB_HOST_PIPE_CALLBACK_STALL) {
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PINTENSET = USB_HOST_PINTENSET_STALL(1);
    }

    return STATUS_OK;
}

/**
 * \brief Disables USB host callback generation for a given type.
 *
 * Disables asynchronous callbacks for a given logical type.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback disable operation.
 * \retval STATUS_OK    The callback was disabled successfully.
 */
enum status_code usb_host_pipe_disable_callback(
        struct usb_module *module_inst, uint8_t pipe_num,
        enum usb_host_pipe_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);

    /* Enable callback */
    module_inst->host_pipe_enabled_callback_mask[pipe_num] &= ~(1 << callback_type);

    if (callback_type == USB_HOST_PIPE_CALLBACK_TRANSFER_COMPLETE) {
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PINTENCLR = USB_HOST_PINTENCLR_TRCPT_Msk;
    }
    if (callback_type == USB_HOST_PIPE_CALLBACK_ERROR) {
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PINTENCLR =
                USB_HOST_PINTENCLR_TRFAIL(1) | USB_HOST_PINTENCLR_PERR(1);
    }
    if (callback_type == USB_HOST_PIPE_CALLBACK_SETUP) {
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PINTENCLR = USB_HOST_PINTENCLR_TXSTP(1);
    }
    if (callback_type == USB_HOST_PIPE_CALLBACK_STALL) {
        module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PINTENCLR = USB_HOST_PINTENCLR_STALL(1);
    }

    return STATUS_OK;
}

/**
 * \brief Sends the setup package.
 *
 * Sends the setup package.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 * \param[in]     buf           Pointer to data buffer
 *
 * \return Status of the setup operation.
 * \retval STATUS_OK    The setup job was set successfully.
 * \retval STATUS_BUSY    The pipe is busy.
 * \retval STATUS_ERR_NOT_INITIALIZED    The pipe has not been configured.
 */
enum status_code usb_host_pipe_setup_job(struct usb_module *module_inst,
        uint8_t pipe_num, uint8_t *buf)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);

    if (host_pipe_job_busy_status & (1 << pipe_num)) {
        return STATUS_BUSY;
    }

    /* Set busy status */
    host_pipe_job_busy_status |= 1 << pipe_num;

    if (__bits_get(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG, USB_HOST_PCFG_PTYPE_Msk, USB_HOST_PCFG_PTYPE_Pos) ==
            USB_HOST_PIPE_TYPE_DISABLE) {
        return STATUS_ERR_NOT_INITIALIZED;
    }

    /* get pipe config from setting register */
    usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_ADDR = (uint32_t)buf;
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
               USB_HOST_PCKSIZE_BYTE_COUNT_Msk, USB_HOST_PCKSIZE_BYTE_COUNT(8));
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
               USB_HOST_PCKSIZE_MULTI_PACKET_SIZE_Msk, USB_HOST_PCKSIZE_MULTI_PACKET_SIZE(0));
    __bits_set(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG,
               USB_HOST_PCFG_PTOKEN_Msk, USB_HOST_PCFG_PTOKEN(USB_HOST_PIPE_TOKEN_SETUP));

    module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PSTATUSSET = USB_HOST_PSTATUSSET_BK0RDY(1);
    usb_host_pipe_unfreeze(module_inst, pipe_num);

    return STATUS_OK;
}

/**
 * \brief USB host pipe read job.
 *
 * USB host pipe read job by set and start an in transaction transfer.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 * \param[in]     buf           Pointer to data buffer
 * \param[in]     buf_size      Data buffer size
 * \note The buffer length should not larger than 0x3FFF
 *
 * \return Status of the setting operation.
 * \retval STATUS_OK    The read job was set successfully.
 * \retval STATUS_BUSY    The pipe is busy.
 * \retval STATUS_ERR_NOT_INITIALIZED    The pipe has not been configured.
 */
enum status_code usb_host_pipe_read_job(struct usb_module *module_inst,
        uint8_t pipe_num, uint8_t *buf, uint32_t buf_size)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);

    if (host_pipe_job_busy_status & (1 << pipe_num)) {
        return STATUS_BUSY;
    }

    /* Set busy status */
    host_pipe_job_busy_status |= 1 << pipe_num;

    if (__bits_get(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG, USB_HOST_PCFG_PTYPE_Msk, USB_HOST_PCFG_PTYPE_Pos) ==
            USB_HOST_PIPE_TYPE_DISABLE) {
        return STATUS_ERR_NOT_INITIALIZED;
    }

    /* get pipe config from setting register */
    usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_ADDR = (uint32_t)buf;
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
               USB_HOST_PCKSIZE_BYTE_COUNT_Msk, USB_HOST_PCKSIZE_BYTE_COUNT(0));
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
               USB_HOST_PCKSIZE_MULTI_PACKET_SIZE_Msk, USB_HOST_PCKSIZE_MULTI_PACKET_SIZE(buf_size));
    __bits_set(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG,
               USB_HOST_PCFG_PTOKEN_Msk, USB_HOST_PCFG_PTOKEN(USB_HOST_PIPE_TOKEN_IN));

    /* Start transfer */
    module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PSTATUSCLR = USB_HOST_PSTATUSCLR_BK0RDY(1);
    usb_host_pipe_unfreeze(module_inst, pipe_num);

    return STATUS_OK;
}

/**
 * \brief USB host pipe write job.
 *
 * USB host pipe write job by set and start an out transaction transfer.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 * \param[in]     buf           Pointer to data buffer
 * \param[in]     buf_size      Data buffer size
 * \note The buffer length should not larger than 0x3FFF
 *
 * \return Status of the setting operation.
 * \retval STATUS_OK    The write job was set successfully.
 * \retval STATUS_BUSY    The pipe is busy.
 * \retval STATUS_ERR_NOT_INITIALIZED    The pipe has not been configured.
 */
enum status_code usb_host_pipe_write_job(struct usb_module *module_inst,
        uint8_t pipe_num, uint8_t *buf, uint32_t buf_size)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);

    if (host_pipe_job_busy_status & (1 << pipe_num)) {
        return STATUS_BUSY;
    }

    /* Set busy status */
    host_pipe_job_busy_status |= 1 << pipe_num;

    if (__bits_get(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG, USB_HOST_PCFG_PTYPE_Msk, USB_HOST_PCFG_PTYPE_Pos) ==
            USB_HOST_PIPE_TYPE_DISABLE) {
        return STATUS_ERR_NOT_INITIALIZED;
    }

    /* get pipe config from setting register */
    usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_ADDR = (uint32_t)buf;
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
               USB_HOST_PCKSIZE_BYTE_COUNT_Msk, USB_HOST_PCKSIZE_BYTE_COUNT(buf_size));
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
               USB_HOST_PCKSIZE_MULTI_PACKET_SIZE_Msk, USB_HOST_PCKSIZE_MULTI_PACKET_SIZE(0));
    __bits_set(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG,
               USB_HOST_PCFG_PTOKEN_Msk, USB_HOST_PCFG_PTOKEN(USB_HOST_PIPE_TOKEN_OUT));

    /* Start transfer */
    module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PSTATUSSET = USB_HOST_PSTATUSSET_BK0RDY(1);
    usb_host_pipe_unfreeze(module_inst, pipe_num);

    return STATUS_OK;
}

/**
 * \brief USB host abort a pipe job.
 *
 * USB host pipe abort job by freeze the pipe.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 *
 * \return Status of the setting operation.
 * \retval STATUS_OK    The abort job was set successfully.
 * \retval STATUS_ERR_NOT_INITIALIZED    The pipe has not been configured.
 */
enum status_code usb_host_pipe_abort_job(struct usb_module *module_inst, uint8_t pipe_num)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);

    if (__bits_get(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG, USB_HOST_PCFG_PTYPE_Msk, USB_HOST_PCFG_PTYPE_Pos) ==
            USB_HOST_PIPE_TYPE_DISABLE) {
        return STATUS_ERR_NOT_INITIALIZED;
    }

    module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PSTATUSSET = USB_HOST_PSTATUSSET_PFREEZE(1);

    /* Clear busy status */
    host_pipe_job_busy_status &= ~(1 << pipe_num);

    return STATUS_OK;
}

/**
 * \brief Sends the LPM package.
 *
 * Sends the LPM package.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 * \param[in]     b_remotewakeup  Remote wake up flag
 * \param[in]     hird  Host Initiated Resume Duration
 *
 * \return Status of the setup operation.
 * \retval STATUS_OK    The setup job was set successfully.
 * \retval STATUS_BUSY    The pipe is busy.
 * \retval STATUS_ERR_NOT_INITIALIZED    The pipe has not been configured.
 */
enum status_code usb_host_pipe_lpm_job(struct usb_module *module_inst,
        uint8_t pipe_num, bool b_remotewakeup, uint8_t hird)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(pipe_num < USB_HOST_PIPE_NUMBER);

    if (host_pipe_job_busy_status & (1 << pipe_num)) {
        return STATUS_BUSY;
    }

    /* Set busy status */
    host_pipe_job_busy_status |= 1 << pipe_num;

    if (__bits_get(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG, USB_HOST_PCFG_PTYPE_Msk, USB_HOST_PCFG_PTYPE_Pos) ==
            USB_HOST_PIPE_TYPE_DISABLE) {
        return STATUS_ERR_NOT_INITIALIZED;
    }

    __bits_set(module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PCFG,
               USB_HOST_PCFG_PTYPE_Msk, USB_HOST_PCFG_PTYPE(USB_HOST_PIPE_TYPE_EXTENDED));

    /* get pipe config from setting register */
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_EXTREG,
               USB_HOST_EXTREG_SUBPID_Msk, USB_HOST_EXTREG_SUBPID(0x3));
    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_EXTREG,
               USB_HOST_EXTREG_VARIABLE_Msk, USB_HOST_EXTREG_VARIABLE(
                USB_LPM_ATTRIBUT_REMOTEWAKE(b_remotewakeup) |
                USB_LPM_ATTRIBUT_HIRD(hird) |
                USB_LPM_ATTRIBUT_BLINKSTATE_L1));

    module_inst->hw->HOST.HOST_PIPE[pipe_num].USB_PSTATUSSET = USB_HOST_PSTATUSSET_BK0RDY(1);
    usb_host_pipe_unfreeze(module_inst, pipe_num);

    return STATUS_OK;
}

/**
 * \internal
 * \brief Function called by USB interrupt to manage USB host interrupts
 *
 * USB host interrupt events are split into four sections:
 * - USB line events
 *   (Device dis/connection, SOF, reset, resume, wakeup, error)
 * - Pipe events
 *   (End of data transfer, setup, stall, error)
 */
static void _usb_host_interrupt_handler(IRQn_Type irq_n)
{
    uint32_t pipe_int;
    uint32_t flags;

    /* Manage pipe interrupts */
    pipe_int = ctz(_usb_instances->hw->HOST.USB_PINTSMRY);
    if (pipe_int < 32) {
        /* pipe interrupts */

        /* get interrupt flags */
        flags = _usb_instances->hw->HOST.HOST_PIPE[pipe_int].USB_PINTFLAG;

        /* host pipe transfer complete interrupt */
        if (flags & USB_HOST_PINTFLAG_TRCPT_Msk) {
            /* Clear busy status */
            host_pipe_job_busy_status &= ~(1 << pipe_int);
            /* clear the flag */
            _usb_instances->hw->HOST.HOST_PIPE[pipe_int].USB_PINTFLAG =
                    USB_HOST_PINTFLAG_TRCPT_Msk;
            if(_usb_instances->host_pipe_enabled_callback_mask[pipe_int] &
                    (1 << USB_HOST_PIPE_CALLBACK_TRANSFER_COMPLETE)) {
                pipe_callback_para.pipe_num = pipe_int;
                if (__bits_get(_usb_instances->hw->HOST.HOST_PIPE[pipe_int].USB_PCFG, USB_HOST_PCFG_PTOKEN_Msk, USB_HOST_PCFG_PTOKEN_Pos) ==
                            USB_HOST_PIPE_TOKEN_IN) {
                    /* in  */
                    pipe_callback_para.transfered_size =
                        __bits_get(usb_descriptor_table.usb_pipe_table[pipe_int].HOST_DESC_BANK[0].USB_PCKSIZE, USB_HOST_PCKSIZE_BYTE_COUNT_Msk, USB_HOST_PCKSIZE_BYTE_COUNT_Pos);
                    pipe_callback_para.required_size =
                        __bits_get(usb_descriptor_table.usb_pipe_table[pipe_int].HOST_DESC_BANK[0].USB_PCKSIZE, USB_HOST_PCKSIZE_MULTI_PACKET_SIZE_Msk, USB_HOST_PCKSIZE_MULTI_PACKET_SIZE_Pos);
                    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_int].HOST_DESC_BANK[0].USB_PCKSIZE,
                               USB_HOST_PCKSIZE_BYTE_COUNT_Msk, USB_HOST_PCKSIZE_BYTE_COUNT(0));
                } else {
                    /* out */
                    pipe_callback_para.transfered_size =
                        __bits_get(usb_descriptor_table.usb_pipe_table[pipe_int].HOST_DESC_BANK[0].USB_PCKSIZE, USB_HOST_PCKSIZE_MULTI_PACKET_SIZE_Msk, USB_HOST_PCKSIZE_MULTI_PACKET_SIZE_Pos);
                    pipe_callback_para.required_size =
                        __bits_get(usb_descriptor_table.usb_pipe_table[pipe_int].HOST_DESC_BANK[0].USB_PCKSIZE, USB_HOST_PCKSIZE_BYTE_COUNT_Msk, USB_HOST_PCKSIZE_BYTE_COUNT_Pos);
                    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_int].HOST_DESC_BANK[0].USB_PCKSIZE,
                               USB_HOST_PCKSIZE_BYTE_COUNT_Msk, USB_HOST_PCKSIZE_BYTE_COUNT(0));
                    if (0 == pipe_callback_para.transfered_size) {
                        pipe_callback_para.transfered_size =
                            __bits_get(usb_descriptor_table.usb_pipe_table[pipe_int].HOST_DESC_BANK[0].USB_PCKSIZE, USB_HOST_PCKSIZE_BYTE_COUNT_Msk, USB_HOST_PCKSIZE_BYTE_COUNT_Pos);
                    }
                }
                (_usb_instances->host_pipe_callback[pipe_int]
                        [USB_HOST_PIPE_CALLBACK_TRANSFER_COMPLETE])(_usb_instances, &pipe_callback_para);
            }
        }

        /* host pipe transfer fail interrupt */
        if (flags & USB_HOST_PINTFLAG_TRFAIL(1)) {
            /* Clear busy status */
            host_pipe_job_busy_status &= ~(1 << pipe_int);
            /* clear the flag */
            _usb_instances->hw->HOST.HOST_PIPE[pipe_int].USB_PINTFLAG =
                    USB_HOST_PINTFLAG_TRFAIL(1);
        }

        /* host pipe error interrupt */
        if (flags & USB_HOST_PINTFLAG_PERR(1)) {
            /* Clear busy status */
            host_pipe_job_busy_status &= ~(1 << pipe_int);
            /* clear the flag */
            _usb_instances->hw->HOST.HOST_PIPE[pipe_int].USB_PINTFLAG =
                    USB_HOST_PINTFLAG_PERR(1);
            if(_usb_instances->host_pipe_enabled_callback_mask[pipe_int] &
                    (1 << USB_HOST_PIPE_CALLBACK_ERROR)) {
                pipe_callback_para.pipe_num = pipe_int;
                pipe_callback_para.pipe_error_status =
                        usb_descriptor_table.usb_pipe_table[pipe_int].HOST_DESC_BANK[0].USB_STATUS_PIPE & 0x1F;
                (_usb_instances->host_pipe_callback[pipe_int]
                        [USB_HOST_PIPE_CALLBACK_ERROR])(_usb_instances, &pipe_callback_para);
            }
        }

        /* host pipe transmitted setup interrupt */
        if (flags & USB_HOST_PINTFLAG_TXSTP(1)) {
            /* Clear busy status */
            host_pipe_job_busy_status &= ~(1 << pipe_int);
            /* clear the flag */
            _usb_instances->hw->HOST.HOST_PIPE[pipe_int].USB_PINTFLAG =
                    USB_HOST_PINTFLAG_TXSTP(1);
            if(_usb_instances->host_pipe_enabled_callback_mask[pipe_int] &
                    (1 << USB_HOST_PIPE_CALLBACK_SETUP)) {
                pipe_callback_para.pipe_num = pipe_int;
                pipe_callback_para.transfered_size =
                    __bits_get(usb_descriptor_table.usb_pipe_table[pipe_int].HOST_DESC_BANK[0].USB_PCKSIZE, USB_HOST_PCKSIZE_MULTI_PACKET_SIZE_Msk, USB_HOST_PCKSIZE_MULTI_PACKET_SIZE_Pos);
                (_usb_instances->host_pipe_callback[pipe_int]
                        [USB_HOST_PIPE_CALLBACK_SETUP])(_usb_instances, NULL);
            }
        }

        /* host pipe stall interrupt */
        if (flags & USB_HOST_PINTFLAG_STALL(1)) {
            /* Clear busy status */
            host_pipe_job_busy_status &= ~(1 << pipe_int);
            /* clear the flag */
            _usb_instances->hw->HOST.HOST_PIPE[pipe_int].USB_PINTFLAG =
                    USB_HOST_PINTFLAG_STALL(1);
            if(_usb_instances->host_pipe_enabled_callback_mask[pipe_int] &
                    (1 << USB_HOST_PIPE_CALLBACK_STALL)) {
                pipe_callback_para.pipe_num = pipe_int;
                (_usb_instances->host_pipe_callback[pipe_int]
                        [USB_HOST_PIPE_CALLBACK_STALL])(_usb_instances, &pipe_callback_para);
            }
        }

    } else {
        /* host interrupts */

        /* get interrupt flags */
        flags = _usb_instances->hw->HOST.USB_INTFLAG;

        /* host SOF interrupt */
        if (flags & USB_HOST_INTFLAG_HSOF(1)) {
            /* clear the flag */
            _usb_instances->hw->HOST.USB_INTFLAG = USB_HOST_INTFLAG_HSOF(1);
            if(_usb_instances->host_enabled_callback_mask & (1 << USB_HOST_CALLBACK_SOF)) {
                (_usb_instances->host_callback[USB_HOST_CALLBACK_SOF])(_usb_instances);
            }
        }

        /* host reset interrupt */
        if (flags & USB_HOST_INTFLAG_RST(1)) {
            /* Clear busy status */
            host_pipe_job_busy_status = 0;
            /* clear the flag */
            _usb_instances->hw->HOST.USB_INTFLAG = USB_HOST_INTFLAG_RST(1);
            if(_usb_instances->host_enabled_callback_mask & (1 << USB_HOST_CALLBACK_RESET)) {
                (_usb_instances->host_callback[USB_HOST_CALLBACK_RESET])(_usb_instances);
            }
        }

        /* host upstream resume interrupts */
        if (flags & USB_HOST_INTFLAG_UPRSM(1)) {
            /* clear the flags */
            _usb_instances->hw->HOST.USB_INTFLAG = USB_HOST_INTFLAG_UPRSM(1);
            if(_usb_instances->host_enabled_callback_mask & (1 << USB_HOST_CALLBACK_UPRSM)) {
                (_usb_instances->host_callback[USB_HOST_CALLBACK_UPRSM])(_usb_instances);
            }
        }

        /* host downstream resume interrupts */
        if (flags & USB_HOST_INTFLAG_DNRSM(1)) {
            /* clear the flags */
            _usb_instances->hw->HOST.USB_INTFLAG = USB_HOST_INTFLAG_DNRSM(1);
            if(_usb_instances->host_enabled_callback_mask & (1 << USB_HOST_CALLBACK_DNRSM)) {
                (_usb_instances->host_callback[USB_HOST_CALLBACK_DNRSM])(_usb_instances);
            }
        }

        /* host wakeup interrupts */
        if (flags & USB_HOST_INTFLAG_WAKEUP(1)) {
            /* clear the flags */
            _usb_instances->hw->HOST.USB_INTFLAG = USB_HOST_INTFLAG_WAKEUP(1);
            if(_usb_instances->host_enabled_callback_mask & (1 << USB_HOST_CALLBACK_WAKEUP)) {
                (_usb_instances->host_callback[USB_HOST_CALLBACK_WAKEUP])(_usb_instances);
            }
        }

        /* host ram access interrupt  */
        if (flags & USB_HOST_INTFLAG_RAMACER(1)) {
            /* Clear busy status */
            host_pipe_job_busy_status = 0;
            /* clear the flag */
            _usb_instances->hw->HOST.USB_INTFLAG = USB_HOST_INTFLAG_RAMACER(1);
            if(_usb_instances->host_enabled_callback_mask & (1 << USB_HOST_CALLBACK_RAMACER)) {
                (_usb_instances->host_callback[USB_HOST_CALLBACK_RAMACER])(_usb_instances);
            }
        }

        /* host connect interrupt */
        if (flags & USB_HOST_INTFLAG_DCONN(1)) {
            /* Clear busy status */
            host_pipe_job_busy_status = 0;
            /* clear the flag */
            _usb_instances->hw->HOST.USB_INTFLAG = USB_HOST_INTFLAG_DCONN(1);
            if(_usb_instances->host_enabled_callback_mask & (1 << USB_HOST_CALLBACK_CONNECT)) {
                (_usb_instances->host_callback[USB_HOST_CALLBACK_CONNECT])(_usb_instances);
            }
        }

        /* host disconnect interrupt     */
        if (flags & USB_HOST_INTFLAG_DDISC(1)) {
            /* Clear busy status */
            host_pipe_job_busy_status = 0;
            /* clear the flag */
            _usb_instances->hw->HOST.USB_INTFLAG = USB_HOST_INTFLAG_DDISC(1);
            if(_usb_instances->host_enabled_callback_mask & (1 << USB_HOST_CALLBACK_DISCONNECT)) {
                (_usb_instances->host_callback[USB_HOST_CALLBACK_DISCONNECT])(_usb_instances);
            }
        }

    }
}

/**
 * \brief Sets USB host pipe auto ZLP setting value
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     pipe_num      Pipe to configure
 * \param[in]     value         Auto ZLP setting value, \c true to enable
 *
 */
void usb_host_pipe_set_auto_zlp(struct usb_module *module_inst, uint8_t pipe_num, bool value)
{
    Assert(module_inst);

    __bits_set(usb_descriptor_table.usb_pipe_table[pipe_num].HOST_DESC_BANK[0].USB_PCKSIZE,
               USB_HOST_PCKSIZE_AUTO_ZLP_Msk, USB_HOST_PCKSIZE_AUTO_ZLP(value));
}

/**
 * \brief Registers a USB device callback
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref usb_device_enable_callback,
 * in order for the interrupt handler to call it when the conditions for the
 * callback type is met.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 * \param[in]     callback_func Pointer to callback function
 *
 * \return Status of the registration operation.
 * \retval STATUS_OK    The callback was registered successfully.
 */
enum status_code usb_device_register_callback(struct usb_module *module_inst,
        enum usb_device_callback callback_type,
        usb_device_callback_t callback_func)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(callback_func);

    /* Register callback function */
    module_inst->device_callback[callback_type] = callback_func;

    /* Set the bit corresponding to the callback_type */
    module_inst->device_registered_callback_mask |= _usb_device_irq_bits[callback_type];

    return STATUS_OK;
}

/**
 * \brief Unregisters a USB device callback
 *
 * Unregisters an asynchronous callback implemented by the user. Removing it
 * from the internal callback registration table.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the de-registration operation.
 * \retval STATUS_OK    The callback was unregistered successfully.
 */
enum status_code usb_device_unregister_callback(struct usb_module *module_inst,
        enum usb_device_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);

    /* Unregister callback function */
    module_inst->device_callback[callback_type] = NULL;

    /* Clear the bit corresponding to the callback_type */
    module_inst->device_registered_callback_mask &= ~_usb_device_irq_bits[callback_type];

    return STATUS_OK;
}

/**
 * \brief Enables USB device callback generation for a given type.
 *
 * Enables asynchronous callbacks for a given logical type.
 * This must be called before USB device generate callback events.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback enable operation.
 * \retval STATUS_OK    The callback was enabled successfully.
 */
enum status_code usb_device_enable_callback(struct usb_module *module_inst,
        enum usb_device_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* clear related flag */
    module_inst->hw->DEVICE.USB_INTFLAG = _usb_device_irq_bits[callback_type];

    /* Enable callback */
    module_inst->device_enabled_callback_mask |= _usb_device_irq_bits[callback_type];

    module_inst->hw->DEVICE.USB_INTENSET = _usb_device_irq_bits[callback_type];

    return STATUS_OK;
}

/**
 * \brief Disables USB device callback generation for a given type.
 *
 * Disables asynchronous callbacks for a given logical type.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback disable operation.
 * \retval STATUS_OK    The callback was disabled successfully.
 */
enum status_code usb_device_disable_callback(struct usb_module *module_inst,
        enum usb_device_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* Disable callback */
    module_inst->device_enabled_callback_mask &= ~_usb_device_irq_bits[callback_type];

    module_inst->hw->DEVICE.USB_INTENCLR = _usb_device_irq_bits[callback_type];

    return STATUS_OK;
}

/**
 * \brief Registers a USB device endpoint callback
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref usb_device_endpoint_enable_callback,
 * in order for the interrupt handler to call it when the conditions for the
 * callback type is met.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     ep_num        Endpoint to configure
 * \param[in]     callback_type Callback type given by an enum
 * \param[in]     callback_func Pointer to callback function
 *
 * \return Status of the registration operation.
 * \retval STATUS_OK    The callback was registered successfully.
 */
enum status_code usb_device_endpoint_register_callback(
        struct usb_module *module_inst, uint8_t ep_num,
        enum usb_device_endpoint_callback callback_type,
        usb_device_endpoint_callback_t callback_func)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(ep_num < USB_DEVICE_ENDPOINT_NUMBER);
    Assert(callback_func);

    /* Register callback function */
    module_inst->device_endpoint_callback[ep_num][callback_type] = callback_func;

    /* Set the bit corresponding to the callback_type */
    module_inst->device_endpoint_registered_callback_mask[ep_num] |= _usb_endpoint_irq_bits[callback_type];

    return STATUS_OK;
}

/**
 * \brief Unregisters a USB device endpoint callback
 *
 * Unregisters an callback implemented by the user. Removing it
 * from the internal callback registration table.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     ep_num        Endpoint to configure
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the de-registration operation.
 * \retval STATUS_OK    The callback was unregistered successfully.
 */
enum status_code usb_device_endpoint_unregister_callback(
        struct usb_module *module_inst, uint8_t ep_num,
        enum usb_device_endpoint_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(ep_num < USB_DEVICE_ENDPOINT_NUMBER);

    /* Unregister callback function */
    module_inst->device_endpoint_callback[ep_num][callback_type] = NULL;

    /* Clear the bit corresponding to the callback_type */
    module_inst->device_endpoint_registered_callback_mask[ep_num] &= ~_usb_endpoint_irq_bits[callback_type];

    return STATUS_OK;
}

/**
 * \brief Enables USB device endpoint callback generation for a given type.
 *
 * Enables callbacks for a given logical type.
 * This must be called before USB device pipe generate callback events.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     ep            Endpoint to configure
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback enable operation.
 * \retval STATUS_OK    The callback was enabled successfully.
 */
enum status_code usb_device_endpoint_enable_callback(
        struct usb_module *module_inst, uint8_t ep,
        enum usb_device_endpoint_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    uint8_t ep_num = ep & USB_EP_ADDR_MASK;
    Assert(ep_num < USB_DEVICE_ENDPOINT_NUMBER);

    /* Enable callback */
    module_inst->device_endpoint_enabled_callback_mask[ep_num] |= _usb_endpoint_irq_bits[callback_type];

    if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_TRCPT) {
        if (ep_num == 0) { // control endpoint
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRCPT0(1) | USB_DEVICE_EPINTENSET_TRCPT1(1);
        } else if (ep & USB_EP_DIR_IN) {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRCPT1(1);
        } else {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRCPT0(1);
        }
    }

    if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL) {
        if (ep_num == 0) { // control endpoint
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRFAIL0(1) | USB_DEVICE_EPINTENSET_TRFAIL1(1);
        } else if (ep & USB_EP_DIR_IN) {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRFAIL1(1);
        } else {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRFAIL0(1);
        }
    }

    if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_RXSTP) {
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENSET = USB_DEVICE_EPINTENSET_RXSTP(1);
    }

    if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_STALL) {
        if (ep & USB_EP_DIR_IN) {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENSET = USB_DEVICE_EPINTENSET_STALL1(1);
        } else {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENSET = USB_DEVICE_EPINTENSET_STALL0(1);
        }
    }

    return STATUS_OK;
}

/**
 * \brief Disables USB device endpoint callback generation for a given type.
 *
 * Disables callbacks for a given logical type.
 *
 * \param[in]     module_inst   Pointer to USB software instance struct
 * \param[in]     ep            Endpoint to configure
 * \param[in]     callback_type Callback type given by an enum
 *
 * \return Status of the callback disable operation.
 * \retval STATUS_OK    The callback was disabled successfully.
 */
enum status_code usb_device_endpoint_disable_callback(
        struct usb_module *module_inst, uint8_t ep,
        enum usb_device_endpoint_callback callback_type)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    uint8_t ep_num = ep & USB_EP_ADDR_MASK;
    Assert(ep_num < USB_DEVICE_ENDPOINT_NUMBER);

    /* Enable callback */
    module_inst->device_endpoint_enabled_callback_mask[ep_num] &= ~_usb_endpoint_irq_bits[callback_type];

    if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_TRCPT) {
        if (ep_num == 0) { // control endpoint
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENCLR =  USB_DEVICE_EPINTENCLR_TRCPT0(1) | USB_DEVICE_EPINTENCLR_TRCPT1(1);
        } else if (ep & USB_EP_DIR_IN) {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENCLR =  USB_DEVICE_EPINTENCLR_TRCPT1(1);
        } else {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENCLR =  USB_DEVICE_EPINTENCLR_TRCPT0(1);
        }
    }

    if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL) {
        if (ep_num == 0) { // control endpoint
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_TRFAIL0(1) | USB_DEVICE_EPINTENCLR_TRFAIL1(1);
        } else if (ep & USB_EP_DIR_IN) {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_TRFAIL1(1);
        } else {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_TRFAIL0(1);
        }
    }

    if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_RXSTP) {
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_RXSTP(1);
    }

    if (callback_type == USB_DEVICE_ENDPOINT_CALLBACK_STALL) {
        if (ep & USB_EP_DIR_IN) {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_STALL1(1);
        } else {
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_STALL0(1);
        }
    }

    return STATUS_OK;
}

/**
 * \brief Initializes an USB device endpoint configuration structure to defaults.
 *
 * Initializes a given USB device endpoint configuration structure to a
 * set of known default values. This function should be called on all new
 * instances of these configuration structures before being modified by the
 * user application.
 *
 * The default configuration is as follows:
 * \li endpoint address is 0
 * \li endpoint size is 8 bytes
 * \li auto_zlp is false
 * \li endpoint type is control
 *
 * \param[out] ep_config  Configuration structure to initialize to default values
 */
void usb_device_endpoint_get_config_defaults(struct usb_device_endpoint_config *ep_config)
{
    /* Sanity check arguments */
    Assert(ep_config);

    /* Write default config to config struct */
    ep_config->ep_address = 0;
    ep_config->ep_size = USB_ENDPOINT_8_BYTE;
    ep_config->auto_zlp = false;
    ep_config->ep_type = USB_DEVICE_ENDPOINT_TYPE_CONTROL;
}

/**
 * \brief Writes an USB device endpoint configuration to the hardware module.
 *
 * Writes out a given configuration of an USB device endpoint
 * configuration to the hardware module. If the pipe is already configured,
 * the new configuration will replace the existing one.
 *
 * \param[in] module_inst    Pointer to USB software instance struct
 * \param[in] ep_config      Configuration settings for the endpoint
 *
 * \return Status of the device endpoint configuration operation
 * \retval STATUS_OK         The device endpoint was configured successfully
 * \retval STATUS_ERR_DENIED The endpoint address is already configured
 */
enum status_code usb_device_endpoint_set_config(struct usb_module *module_inst,
        struct usb_device_endpoint_config *ep_config)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(ep_config);

    uint8_t ep_num = ep_config->ep_address & USB_EP_ADDR_MASK;
    uint8_t ep_bank = (ep_config->ep_address & USB_EP_DIR_IN) ? 1 : 0;

    switch (ep_config->ep_type) {
        case USB_DEVICE_ENDPOINT_TYPE_DISABLE:
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG = USB_DEVICE_EPCFG_EPTYPE0(0) |  USB_DEVICE_EPCFG_EPTYPE1(0);
            return STATUS_OK;

        case USB_DEVICE_ENDPOINT_TYPE_CONTROL:
            if ((module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG & USB_DEVICE_EPCFG_EPTYPE0_Msk) == 0 && \
                (module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG & USB_DEVICE_EPCFG_EPTYPE1_Msk) == 0) {
                module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG = USB_DEVICE_EPCFG_EPTYPE0(1) | USB_DEVICE_EPCFG_EPTYPE1(1);
                module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY(1);
                module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY(1);
            } else {
                return STATUS_ERR_DENIED;
            }
            if (true == ep_config->auto_zlp) {
                usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[0].USB_PCKSIZE |= USB_DEVICE_PCKSIZE_AUTO_ZLP(1);
                usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[1].USB_PCKSIZE |= USB_DEVICE_PCKSIZE_AUTO_ZLP(1);
            } else {
                usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[0].USB_PCKSIZE &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP_Msk;
                usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[1].USB_PCKSIZE &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP_Msk;
            }
            __bits_set(usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[0].USB_PCKSIZE,
                       USB_HOST_PCKSIZE_SIZE_Msk, USB_HOST_PCKSIZE_SIZE(ep_config->ep_size));
            __bits_set(usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[1].USB_PCKSIZE,
                       USB_HOST_PCKSIZE_SIZE_Msk, USB_HOST_PCKSIZE_SIZE(ep_config->ep_size));
            return STATUS_OK;

        case USB_DEVICE_ENDPOINT_TYPE_ISOCHRONOUS:
            if (ep_bank) {
                if ((module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG & USB_DEVICE_EPCFG_EPTYPE1_Msk) == 0){
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG |= USB_DEVICE_EPCFG_EPTYPE1(2);
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY(1);
                } else {
                    return STATUS_ERR_DENIED;
                }
            } else {
                if ((module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG & USB_DEVICE_EPCFG_EPTYPE0_Msk) == 0){
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG |= USB_DEVICE_EPCFG_EPTYPE0(2);
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY(1);
                } else {
                    return STATUS_ERR_DENIED;
                }
            }
            break;

        case USB_DEVICE_ENDPOINT_TYPE_BULK:
            if (ep_bank) {
                if ((module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG & USB_DEVICE_EPCFG_EPTYPE1_Msk) == 0){
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG |= USB_DEVICE_EPCFG_EPTYPE1(3);
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY(1);
                } else {
                    return STATUS_ERR_DENIED;
                }
            } else {
                if ((module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG & USB_DEVICE_EPCFG_EPTYPE0_Msk) == 0){
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG |= USB_DEVICE_EPCFG_EPTYPE0(3);
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY(1);
                } else {
                    return STATUS_ERR_DENIED;
                }
            }
            break;

        case USB_DEVICE_ENDPOINT_TYPE_INTERRUPT:
            if (ep_bank) {
                if ((module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG & USB_DEVICE_EPCFG_EPTYPE1_Msk) == 0){
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG |= USB_DEVICE_EPCFG_EPTYPE1(4);
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY(1);
                } else {
                    return STATUS_ERR_DENIED;
                }
            } else {
                if ((module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG & USB_DEVICE_EPCFG_EPTYPE0_Msk) == 0){
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG |= USB_DEVICE_EPCFG_EPTYPE0(4);
                    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY(1);
                } else {
                    return STATUS_ERR_DENIED;
                }
            }
            break;

        default:
            break;
    }

    __bits_set(usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[ep_bank].USB_PCKSIZE,
               USB_HOST_PCKSIZE_SIZE_Msk, USB_HOST_PCKSIZE_SIZE(ep_config->ep_size));

    if (true == ep_config->auto_zlp) {
        usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[ep_bank].USB_PCKSIZE |= USB_DEVICE_PCKSIZE_AUTO_ZLP(1);
        } else {
        usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[ep_bank].USB_PCKSIZE &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP_Msk;
    }

    return STATUS_OK;
}

/**
 * \brief Check if current endpoint is configured
 *
 * \param module_inst   Pointer to USB software instance struct
 * \param ep            Endpoint address (direction & number)
 *
 * \return \c true if endpoint is configured and ready to use
 */
bool usb_device_endpoint_is_configured(struct usb_module *module_inst, uint8_t ep)
{
    uint8_t ep_num = ep & USB_EP_ADDR_MASK;
    uint8_t flag;

    if (ep & USB_EP_DIR_IN) {
        flag = (uint8_t)__bits_get(module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG,
                                   USB_DEVICE_EPCFG_EPTYPE1_Msk, USB_DEVICE_EPCFG_EPTYPE1_Pos);
    } else {
        flag = (uint8_t)__bits_get(module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG,
                                   USB_DEVICE_EPCFG_EPTYPE0_Msk, USB_DEVICE_EPCFG_EPTYPE0_Pos);
    }
    return ((enum usb_device_endpoint_type)(flag) != USB_DEVICE_ENDPOINT_TYPE_DISABLE);
}


/**
 * \brief Abort ongoing job on the endpoint
 *
 * \param module_inst Pointer to USB software instance struct
 * \param ep          Endpoint address
 */
void usb_device_endpoint_abort_job(struct usb_module *module_inst, uint8_t ep)
{
    uint8_t ep_num;
    ep_num = ep & USB_EP_ADDR_MASK;

    // Stop transfer
    if (ep & USB_EP_DIR_IN) {
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY(1);
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRFAIL1(1);
        // Eventually ack a transfer occur during abort
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT1(1);
    } else {
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY(1);
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRFAIL0(1);
        // Eventually ack a transfer occur during abort
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT0(1);
    }
}

/**
 * \brief Check if endpoint is halted
 *
 * \param module_inst Pointer to USB software instance struct
 * \param ep          Endpoint address
 *
 * \return \c true if the endpoint is halted
 */
bool usb_device_endpoint_is_halted(struct usb_module *module_inst, uint8_t ep)
{
    uint8_t ep_num = ep & USB_EP_ADDR_MASK;

    if (ep & USB_EP_DIR_IN) {
        return (module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUS & USB_DEVICE_EPSTATUSSET_STALLRQ1(1));
    } else {
        return (module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUS & USB_DEVICE_EPSTATUSSET_STALLRQ0(1));
    }
}

/**
 * \brief Halt the endpoint (send STALL)
 *
 * \param module_inst Pointer to USB software instance struct
 * \param ep          Endpoint address
 */
void usb_device_endpoint_set_halt(struct usb_module *module_inst, uint8_t ep)
{
    uint8_t ep_num = ep & USB_EP_ADDR_MASK;

    // Stall endpoint
    if (ep & USB_EP_DIR_IN) {
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_STALLRQ1(1);
    } else {
        module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_STALLRQ0(1);
    }
}

/**
 * \brief Clear endpoint halt state
 *
 * \param module_inst Pointer to USB software instance struct
 * \param ep          Endpoint address
 */
void usb_device_endpoint_clear_halt(struct usb_module *module_inst, uint8_t ep)
{
    uint8_t ep_num = ep & USB_EP_ADDR_MASK;

    if (ep & USB_EP_DIR_IN) {
        if (module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUS & USB_DEVICE_EPSTATUSSET_STALLRQ1(1)) {
            // Remove stall request
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ1(1);
            if (module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_STALL1(1)) {
                module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_STALL1(1);
                // The Stall has occurred, then reset data toggle
                module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSSET_DTGLIN(1);
            }
        }
    } else {
        if (module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUS & USB_DEVICE_EPSTATUSSET_STALLRQ0(1)) {
            // Remove stall request
            module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ0(1);
            if (module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_STALL0(1)) {
                module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_STALL0(1);
                // The Stall has occurred, then reset data toggle
                module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSSET_DTGLOUT(1);
            }
        }
    }
}

/**
 * \brief Start write buffer job on a endpoint
 *
 * \param module_inst Pointer to USB module instance
 * \param ep_num      Endpoint number
 * \param pbuf        Pointer to buffer
 * \param buf_size    Size of buffer
 *
 * \return Status of procedure
 * \retval STATUS_OK Job started successfully
 * \retval STATUS_ERR_DENIED Endpoint is not ready
 */
enum status_code usb_device_endpoint_write_buffer_job(struct usb_module *module_inst,uint8_t ep_num,
        uint8_t* pbuf, uint32_t buf_size)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(ep_num < USB_DEVICE_ENDPOINT_NUMBER);

    uint8_t flag;
    flag = (uint8_t)__bits_get(module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG,
                               USB_DEVICE_EPCFG_EPTYPE1_Msk, USB_DEVICE_EPCFG_EPTYPE1_Pos);
    if ((enum usb_device_endpoint_type)(flag) == USB_DEVICE_ENDPOINT_TYPE_DISABLE) {
        return STATUS_ERR_DENIED;
    };

    /* get endpoint configuration from setting register */
    usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[1].USB_ADDR = (uint32_t)pbuf;
    __bits_set(usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[1].USB_PCKSIZE,
               USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk, USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0));
    __bits_set(usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[1].USB_PCKSIZE,
               USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk, USB_DEVICE_PCKSIZE_BYTE_COUNT(buf_size));
    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK1RDY(1);

    return STATUS_OK;
}

/**
 * \brief Start read buffer job on a endpoint
 *
 * \param module_inst Pointer to USB module instance
 * \param ep_num      Endpoint number
 * \param pbuf        Pointer to buffer
 * \param buf_size    Size of buffer
 *
 * \return Status of procedure
 * \retval STATUS_OK Job started successfully
 * \retval STATUS_ERR_DENIED Endpoint is not ready
 */
enum status_code usb_device_endpoint_read_buffer_job(struct usb_module *module_inst,uint8_t ep_num,
        uint8_t* pbuf, uint32_t buf_size)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);
    Assert(ep_num < USB_DEVICE_ENDPOINT_NUMBER);

    uint8_t flag;
    flag = (uint8_t)__bits_get(module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPCFG,
                               USB_DEVICE_EPCFG_EPTYPE0_Msk, USB_DEVICE_EPCFG_EPTYPE0_Pos);
    if ((enum usb_device_endpoint_type)(flag) == USB_DEVICE_ENDPOINT_TYPE_DISABLE) {
        return STATUS_ERR_DENIED;
    };

    /* get endpoint configuration from setting register */
    usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[0].USB_ADDR = (uint32_t)pbuf;
    __bits_set(usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[0].USB_PCKSIZE,
               USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk, USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(buf_size));
    __bits_set(usb_descriptor_table.usb_endpoint_table[ep_num].DEVICE_DESC_BANK[0].USB_PCKSIZE,
               USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk, USB_DEVICE_PCKSIZE_BYTE_COUNT(0));
    module_inst->hw->DEVICE.DEVICE_ENDPOINT[ep_num].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK0RDY(1);

    return STATUS_OK;
}

/**
 * \brief Start setup packet read job on a endpoint
 *
 * \param module_inst Pointer to USB device module instance
 * \param pbuf        Pointer to buffer
 *
 * \return Status of procedure
 * \retval STATUS_OK Job started successfully
 * \retval STATUS_ERR_DENIED Endpoint is not ready
 */
enum status_code usb_device_endpoint_setup_buffer_job(struct usb_module *module_inst,
        uint8_t* pbuf)
{
    /* Sanity check arguments */
    Assert(module_inst);
    Assert(module_inst->hw);

    /* get endpoint configuration from setting register */
    usb_descriptor_table.usb_endpoint_table[0].DEVICE_DESC_BANK[0].USB_ADDR = (uint32_t)pbuf;
    __bits_set(usb_descriptor_table.usb_endpoint_table[0].DEVICE_DESC_BANK[0].USB_PCKSIZE,
               USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk, USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(8));
    __bits_set(usb_descriptor_table.usb_endpoint_table[0].DEVICE_DESC_BANK[0].USB_PCKSIZE,
               USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk, USB_DEVICE_PCKSIZE_BYTE_COUNT(0));
    module_inst->hw->DEVICE.DEVICE_ENDPOINT[0].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK0RDY(1);

    return STATUS_OK;
}

static void _usb_device_interrupt_handler(IRQn_Type irq_n)
{
    uint16_t epintsmry = _usb_instances->hw->DEVICE.USB_EPINTSMRY;

    uint16_t callback_enable = _usb_instances->device_enabled_callback_mask &
                               _usb_instances->device_registered_callback_mask;

    if (irq_n == USB_OTHER_IRQn) {
        for (uint8_t i = 0; i < USB_DEVICE_CALLBACK_N; i ++) {
            if (i == USB_DEVICE_CALLBACK_SOF) {
                continue;
            }
            if (callback_enable & _usb_device_irq_bits[i]) {
                if (i == USB_DEVICE_CALLBACK_LPMSUSP) {
                    device_callback_lpm_wakeup_enable =
                            __bits_get(usb_descriptor_table.usb_endpoint_table[0].DEVICE_DESC_BANK[0].USB_EXTREG,
                                       USB_DEVICE_EXTREG_VARIABLE_Msk, USB_DEVICE_EXTREG_VARIABLE_Pos)
                            & USB_LPM_ATTRIBUT_REMOTEWAKE_MASK;
                }
                (_usb_instances->device_callback[i])(irq_n);
            }
        }

        for (uint8_t i = 0; i < USB_DEVICE_ENDPOINT_NUMBER; i++) {
            if (epintsmry & (1 << i)) {
                uint16_t ep_callback_enable = _usb_instances->device_endpoint_enabled_callback_mask[i] &
                                              _usb_instances->device_endpoint_registered_callback_mask[i];
                if (ep_callback_enable & USB_DEVICE_EPINTFLAG_TRFAIL_Msk) {
                    (_usb_instances->device_endpoint_callback[i][USB_DEVICE_ENDPOINT_CALLBACK_TRFAIL])(irq_n);
                }
                if (ep_callback_enable & USB_DEVICE_EPINTFLAG_STALL_Msk) {
                    (_usb_instances->device_endpoint_callback[i][USB_DEVICE_ENDPOINT_CALLBACK_STALL])(irq_n);
                }
                if (ep_callback_enable & USB_DEVICE_EPINTFLAG_RXSTP(1)) {
                    (_usb_instances->device_endpoint_callback[i][USB_DEVICE_ENDPOINT_CALLBACK_RXSTP])(irq_n);
                }
            }
        }
    }

    if (irq_n == USB_SOF_HSOF_IRQn) {
        if (callback_enable & _usb_device_irq_bits[USB_DEVICE_CALLBACK_SOF]) {
            (_usb_instances->device_callback[USB_DEVICE_CALLBACK_SOF])(irq_n);
        }
    }

    if (irq_n == USB_TRCPT0_IRQn || irq_n == USB_TRCPT1_IRQn) {
        for (uint8_t i = 0; i < USB_DEVICE_ENDPOINT_NUMBER; i++) {
            if (epintsmry & (1 << i)) {
                uint16_t ep_callback_enable = _usb_instances->device_endpoint_enabled_callback_mask[i] &
                                              _usb_instances->device_endpoint_registered_callback_mask[i];

                if(ep_callback_enable & USB_DEVICE_EPINTFLAG_TRCPT_Msk) {
                    (_usb_instances->device_endpoint_callback[i][USB_DEVICE_ENDPOINT_CALLBACK_TRCPT])(irq_n);
                }
            }
        }
    }
}

/**
 * \brief Enable the USB module peripheral
 *
 * \param module_inst pointer to USB module instance
 */
void usb_enable(struct usb_module *module_inst)
{
    Assert(module_inst);
    Assert(module_inst->hw);

    module_inst->hw->DEVICE.USB_CTRLA |= USB_CTRLA_ENABLE(1);
    while (module_inst->hw->DEVICE.USB_SYNCBUSY == USB_SYNCBUSY_ENABLE(1));
}

/**
 * \brief Disable the USB module peripheral
 *
 * \param module_inst pointer to USB module instance
 */
void usb_disable(struct usb_module *module_inst)
{
    Assert(module_inst);
    Assert(module_inst->hw);

    module_inst->hw->DEVICE.USB_INTENCLR = USB_DEVICE_INTENCLR_Msk;
    module_inst->hw->DEVICE.USB_INTFLAG = USB_DEVICE_INTFLAG_Msk;
    module_inst->hw->DEVICE.USB_CTRLA &= ~USB_CTRLA_ENABLE_Msk;
    while (module_inst->hw->DEVICE.USB_SYNCBUSY == USB_SYNCBUSY_ENABLE(1));
}

/**
 * \brief Interrupt handler for the USB module.
 */
void USB_Handler(IRQn_Type irq_n)
{
    if (_usb_instances->hw->DEVICE.USB_CTRLA & USB_CTRLA_MODE_Msk) {
        /*host mode ISR */
        _usb_host_interrupt_handler(irq_n);
    } else {
        /*device mode ISR */
        _usb_device_interrupt_handler(irq_n);
    }
}

void USB_OTHER_Handler(void) { USB_Handler(USB_OTHER_IRQn); }
void USB_SOF_HSOF_Handler(void) { USB_Handler(USB_SOF_HSOF_IRQn); }
void USB_TRCPT0_Handler(void) { USB_Handler(USB_TRCPT0_IRQn); }
void USB_TRCPT1_Handler(void) { USB_Handler(USB_TRCPT1_IRQn); }

/**
 * \brief Get the default USB module settings
 *
 * \param[out] module_config  Configuration structure to initialize to default values
 */
void usb_get_config_defaults(struct usb_config *module_config)
{
    /* Sanity check arguments */
    Assert(module_config);

    /* Write default configuration to config struct */
    module_config->select_host_mode = 0;
    module_config->run_in_standby = 1;
    module_config->source_generator = GCLK_GENERATOR_0;
    module_config->speed_mode = USB_SPEED_FULL;
}

/**
 * \brief Initializes USB module instance
 *
 * Enables the clock and initializes the USB module, based on the given
 * configuration values.
 *
 * \param[in,out] module_inst   Pointer to the software module instance struct
 * \param[in]     hw            Pointer to the USB hardware module
 * \param[in]     module_config Pointer to the USB configuration options struct
 *
 * \return Status of the initialization procedure.
 *
 * \retval STATUS_OK           The module was initialized successfully
 */
enum status_code usb_init(struct usb_module *module_inst, usb_registers_t *const hw,
        struct usb_config *module_config)
{
    /* Sanity check arguments */
    Assert(hw);
    Assert(module_inst);
    Assert(module_config);

    uint32_t i,j;
    uint32_t pad_transn, pad_transp, pad_trim;
    struct system_pinmux_config pin_config;
    struct system_gclk_chan_config gclk_chan_config;

    host_pipe_job_busy_status = 0;

    _usb_instances = module_inst;

    /* Associate the software module instance with the hardware module */
    module_inst->hw = hw;

    /* Turn on the digital interface clock */
    system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBB, MCLK_APBBMASK_USB(1));

    /* Set up the USB DP/DN pins */
    system_pinmux_get_config_defaults(&pin_config);
    pin_config.mux_position = MUX_PA24H_USB_DM;
    system_pinmux_pin_set_config(PIN_PA24H_USB_DM, &pin_config);
    pin_config.mux_position = MUX_PA25H_USB_DP;
    system_pinmux_pin_set_config(PIN_PA25H_USB_DP, &pin_config);

    /* Setup clock for module */
    system_gclk_chan_get_config_defaults(&gclk_chan_config);
    gclk_chan_config.source_generator = module_config->source_generator;
    system_gclk_chan_set_config(GCLK_PCHCTRL_ID_10_GCLK_USB, &gclk_chan_config);
    system_gclk_chan_enable(GCLK_PCHCTRL_ID_10_GCLK_USB);

    /* Reset */
    hw->DEVICE.USB_CTRLA |= USB_CTRLA_SWRST(1);
    while (hw->DEVICE.USB_SYNCBUSY & USB_SYNCBUSY_SWRST_Msk) {
        /* Sync wait */
    }

    /* Change QOS values to have the best performance and correct USB behaviour */
    __bits_set(hw->DEVICE.USB_QOSCTRL, USB_QOSCTRL_CQOS_Msk, USB_QOSCTRL_CQOS(2));
    __bits_set(hw->DEVICE.USB_QOSCTRL, USB_QOSCTRL_DQOS_Msk, USB_QOSCTRL_DQOS(2));

    /* Load Pad Calibration */
#define SW0_USB_PAD_TRANSN_POS  (45)
#define SW0_USB_PAD_TRANSN_SIZE (5)
    pad_transn =( *((uint32_t *)(SW0_ADDR)
            + (SW0_USB_PAD_TRANSN_POS / 32))
        >> (SW0_USB_PAD_TRANSN_POS % 32))
        & ((1 << SW0_USB_PAD_TRANSN_SIZE) - 1);

    if (pad_transn == 0x1F) {
        pad_transn = 5;
    }

    __bits_set(hw->DEVICE.USB_PADCAL, USB_PADCAL_TRANSN_Msk, USB_PADCAL_TRANSN(pad_transn));

#define SW0_USB_PAD_TRANSP_POS  (50)
#define SW0_USB_PAD_TRANSP_SIZE (5)
    pad_transp =( *((uint32_t *)(SW0_ADDR)
            + (SW0_USB_PAD_TRANSP_POS / 32))
            >> (SW0_USB_PAD_TRANSP_POS % 32))
            & ((1 << SW0_USB_PAD_TRANSP_SIZE) - 1);

    if (pad_transp == 0x1F) {
        pad_transp = 29;
    }

    __bits_set(hw->DEVICE.USB_PADCAL, USB_PADCAL_TRANSP_Msk, USB_PADCAL_TRANSP(pad_transp));

#define SW0_USB_PAD_TRIM_POS  (55)
#define SW0_USB_PAD_TRIM_SIZE (3)
    pad_trim =( *((uint32_t *)(SW0_ADDR)
            + (SW0_USB_PAD_TRIM_POS / 32))
            >> (SW0_USB_PAD_TRIM_POS % 32))
            & ((1 << SW0_USB_PAD_TRIM_SIZE) - 1);

    if (pad_trim == 0x7) {
        pad_trim = 3;
    }

    __bits_set(hw->DEVICE.USB_PADCAL, USB_PADCAL_TRIM_Msk, USB_PADCAL_TRIM(pad_trim));

    /* Set the configuration */
    __bits_set(hw->DEVICE.USB_CTRLA, USB_CTRLA_MODE_Msk, USB_CTRLA_MODE(module_config->select_host_mode));
    __bits_set(hw->DEVICE.USB_CTRLA, USB_CTRLA_RUNSTDBY_Msk, USB_CTRLA_RUNSTDBY(module_config->run_in_standby));
    hw->DEVICE.USB_DESCADD = (uint32_t)(&usb_descriptor_table.usb_endpoint_table[0]);
    if (USB_SPEED_FULL == module_config->speed_mode) {
        __bits_set(module_inst->hw->DEVICE.USB_CTRLB, USB_DEVICE_CTRLB_SPDCONF_Msk, USB_DEVICE_CTRLB_SPDCONF(USB_DEVICE_CTRLB_SPDCONF_FS_Val));
    } else if(USB_SPEED_LOW == module_config->speed_mode) {
        __bits_set(module_inst->hw->DEVICE.USB_CTRLB, USB_DEVICE_CTRLB_SPDCONF_Msk, USB_DEVICE_CTRLB_SPDCONF(USB_DEVICE_CTRLB_SPDCONF_LS_Val));
    }

    memset((uint8_t *)(&usb_descriptor_table.usb_endpoint_table[0]), 0,
            sizeof(usb_descriptor_table.usb_endpoint_table));

    /* callback related init */
    for (i = 0; i < USB_HOST_CALLBACK_N; i++) {
        module_inst->host_callback[i] = NULL;
    };
    for (i = 0; i < USB_HOST_PIPE_NUMBER; i++) {
        for (j = 0; j < USB_HOST_PIPE_CALLBACK_N; j++) {
            module_inst->host_pipe_callback[i][j] = NULL;
        }
    };
    module_inst->host_registered_callback_mask = 0;
    module_inst->host_enabled_callback_mask = 0;
    for (i = 0; i < USB_HOST_PIPE_NUMBER; i++) {
        module_inst->host_pipe_registered_callback_mask[i] = 0;
        module_inst->host_pipe_enabled_callback_mask[i] = 0;
    }

    /*  device callback related */
    for (i = 0; i < USB_DEVICE_CALLBACK_N; i++) {
        module_inst->device_callback[i] = NULL;
    }
    for (i = 0; i < USB_DEVICE_ENDPOINT_NUMBER; i++) {
        for(j = 0; j < USB_DEVICE_EP_CALLBACK_N; j++) {
            module_inst->device_endpoint_callback[i][j] = NULL;
        }
    }
    module_inst->device_registered_callback_mask = 0;
    module_inst->device_enabled_callback_mask = 0;
    for (j = 0; j < USB_DEVICE_ENDPOINT_NUMBER; j++) {
        module_inst->device_endpoint_registered_callback_mask[j] = 0;
        module_inst->device_endpoint_enabled_callback_mask[j] = 0;
    }

    return STATUS_OK;
}
