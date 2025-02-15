/**
 * \file
 *
 * \brief SAM SERCOM USART Driver
 *
 * Copyright (C) 2012-2015 Atmel Corporation. All rights reserved.
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
#include "sercom_drv.h"
#if USART_CALLBACK_MODE == true
#  include "usart_interrupt.h"
#endif

/**
 * \internal
 * Set Configuration of the USART module
 */
static enum status_code _usart_set_config(
    struct usart_module *const module,
    const struct usart_config *const config)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    /* Get a pointer to the hardware module instance */
    sercom_usart_int_registers_t *const usart_hw = &module->hw->USART_INT;

    /* Index for generic clock */
    uint32_t sercom_index = _sercom_get_sercom_inst_index(module->hw);
    uint32_t chan_index[] = {
        GCLK_PCHCTRL_ID_7_GCLK_SERCOM0_CORE,
        GCLK_PCHCTRL_ID_8_GCLK_SERCOM1_CORE,
        GCLK_PCHCTRL_ID_23_GCLK_SERCOM2_CORE,
        GCLK_PCHCTRL_ID_24_GCLK_SERCOM3_CORE,
        GCLK_PCHCTRL_ID_34_GCLK_SERCOM4_CORE,
        GCLK_PCHCTRL_ID_35_GCLK_SERCOM5_CORE,
        GCLK_PCHCTRL_ID_36_GCLK_SERCOM6_CORE,
        GCLK_PCHCTRL_ID_37_GCLK_SERCOM7_CORE,
    };

    /* Cache new register values to minimize the number of register writes */
    uint32_t ctrla = 0;
    uint32_t ctrlb = 0;
    uint16_t baud  = 0;

#ifdef FEATURE_USART_OVER_SAMPLE
    enum sercom_asynchronous_operation_mode mode = SERCOM_ASYNC_OPERATION_MODE_ARITHMETIC;
    enum sercom_asynchronous_sample_num sample_num = SERCOM_ASYNC_SAMPLE_NUM_16;

    switch (config->sample_rate) {
        case USART_SAMPLE_RATE_16X_ARITHMETIC:
            mode = SERCOM_ASYNC_OPERATION_MODE_ARITHMETIC;
            sample_num = SERCOM_ASYNC_SAMPLE_NUM_16;
            break;
        case USART_SAMPLE_RATE_8X_ARITHMETIC:
            mode = SERCOM_ASYNC_OPERATION_MODE_ARITHMETIC;
            sample_num = SERCOM_ASYNC_SAMPLE_NUM_8;
            break;
        case USART_SAMPLE_RATE_3X_ARITHMETIC:
            mode = SERCOM_ASYNC_OPERATION_MODE_ARITHMETIC;
            sample_num = SERCOM_ASYNC_SAMPLE_NUM_3;
            break;
        case USART_SAMPLE_RATE_16X_FRACTIONAL:
            mode = SERCOM_ASYNC_OPERATION_MODE_FRACTIONAL;
            sample_num = SERCOM_ASYNC_SAMPLE_NUM_16;
            break;
        case USART_SAMPLE_RATE_8X_FRACTIONAL:
            mode = SERCOM_ASYNC_OPERATION_MODE_FRACTIONAL;
            sample_num = SERCOM_ASYNC_SAMPLE_NUM_8;
            break;
    }
#endif

    /* Set data order, internal muxing, and clock polarity */
    ctrla = (uint32_t)config->data_order |
            (uint32_t)config->mux_setting |
#ifdef FEATURE_USART_OVER_SAMPLE
            config->sample_adjustment |
            config->sample_rate |
#endif
#ifdef FEATURE_USART_IMMEDIATE_BUFFER_OVERFLOW_NOTIFICATION
            SERCOM_USART_INT_CTRLA_IBON(config->immediate_buffer_overflow_notification) |
#endif
            SERCOM_USART_INT_CTRLA_CPOL(config->clock_polarity_inverted);

    enum status_code status_code = STATUS_OK;

    /* Get baud value from mode and clock */
    switch (config->transfer_mode) {
        case USART_TRANSFER_SYNCHRONOUSLY:
            if (!config->use_external_clock) {
                status_code = _sercom_get_sync_baud_val(config->baudrate,
                        system_gclk_chan_get_hz(chan_index[sercom_index]), &baud);
            }

            break;

        case USART_TRANSFER_ASYNCHRONOUSLY:
            if (config->use_external_clock) {
                status_code =
                    _sercom_get_async_baud_val(config->baudrate,
                            config->ext_clock_freq, &baud, mode, sample_num);
            } else {
                status_code =
                    _sercom_get_async_baud_val(config->baudrate,
                            system_gclk_chan_get_hz(chan_index[sercom_index]), &baud, mode, sample_num);
            }

            break;
    }

    /* Check if calculating the baudrate failed */
    if (status_code != STATUS_OK) {
        /* Abort */
        return status_code;
    }

#ifdef FEATURE_USART_IRDA
    if(config->encoding_format_enable) {
        usart_hw->SERCOM_RXPL = config->receive_pulse_length;
    }
#endif

    /* Wait until synchronization is complete */
    _usart_wait_for_sync(module);

    /*Set baud val */
    usart_hw->SERCOM_BAUD = baud;

    /* Set sample mode */
    ctrla |= config->transfer_mode;

    if (config->use_external_clock == false) {
        ctrla |= SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK;
    } else {
        ctrla |= SERCOM_USART_INT_CTRLA_MODE_USART_EXT_CLK;
    }

    /* Set stopbits, character size and enable transceivers */
    ctrlb = (uint32_t)config->stopbits | (uint32_t)config->character_size |
#ifdef FEATURE_USART_IRDA
            SERCOM_USART_EXT_CTRLB_ENC(config->encoding_format_enable) |
#endif
#ifdef FEATURE_USART_START_FRAME_DECTION
            SERCOM_USART_EXT_CTRLB_SFDE(config->start_frame_detection_enable) |
#endif
#ifdef FEATURE_USART_COLLISION_DECTION
            SERCOM_USART_EXT_CTRLB_COLDEN(config->collision_detection_enable) |
#endif
            SERCOM_USART_EXT_CTRLB_RXEN(config->receiver_enable) |
            SERCOM_USART_EXT_CTRLB_TXEN(config->transmitter_enable);

    /* Check parity mode bits */
    if (config->parity != USART_PARITY_NONE) {
        ctrla |= SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_WITH_PARITY;
        ctrlb |= config->parity;
    } else {
#ifdef FEATURE_USART_LIN_SLAVE
        if(config->lin_slave_enable) {
            ctrla |= SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_AUTO_BAUD_NO_PARITY;
        } else {
            ctrla |= SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_NO_PARITY;
        }
#else
        ctrla |= SERCOM_USART_INT_CTRLA_FORM_USART_FRAME_NO_PARITY;
#endif
    }

#ifdef FEATURE_USART_LIN_MASTER
    usart_hw->SERCOM_CTRLC = ((usart_hw->SERCOM_CTRLC) & SERCOM_USART_CTRLC_GTIME_Msk)
                          | config->lin_header_delay
                          | config->lin_break_length;

    if (config->lin_node != LIN_INVALID_MODE) {
        ctrla &= ~SERCOM_USART_INT_CTRLA_FORM_Msk;
        ctrla |= config->lin_node;
    }
#endif

    /* Set whether module should run in standby. */
    if (config->run_in_standby || system_is_debugger_present()) {
        ctrla |= SERCOM_USART_INT_CTRLA_RUNSTDBY(1);
    }

    /* Wait until synchronization is complete */
    _usart_wait_for_sync(module);

    /* Write configuration to CTRLB */
    usart_hw->SERCOM_CTRLB = ctrlb;

    /* Wait until synchronization is complete */
    _usart_wait_for_sync(module);

    /* Write configuration to CTRLA */
    usart_hw->SERCOM_CTRLA = ctrla;

#ifdef FEATURE_USART_RS485
    usart_hw->SERCOM_CTRLC &= ~SERCOM_USART_INT_CTRLC_GTIME_Msk;
    usart_hw->SERCOM_CTRLC |= SERCOM_USART_CTRLC_GTIME(config->rs485_guard_time);
#endif

    return STATUS_OK;
}

/**
 * \brief Initializes the device
 *
 * Initializes the USART device based on the setting specified in the
 * configuration struct.
 *
 * \param[out] module  Pointer to USART device
 * \param[in]  hw      Pointer to USART hardware instance
 * \param[in]  config  Pointer to configuration struct
 *
 * \return Status of the initialization.
 *
 * \retval STATUS_OK                       The initialization was successful
 * \retval STATUS_BUSY                     The USART module is busy
 *                                         resetting
 * \retval STATUS_ERR_DENIED               The USART have not been disabled in
 *                                         advance of initialization
 * \retval STATUS_ERR_INVALID_ARG          The configuration struct contains
 *                                         invalid configuration
 * \retval STATUS_ERR_ALREADY_INITIALIZED  The SERCOM instance has already been
 *                                         initialized with different clock
 *                                         configuration
 * \retval STATUS_ERR_BAUD_UNAVAILABLE     The BAUD rate given by the
 *                                         configuration
 *                                         struct cannot be reached with
 *                                         the current clock configuration
 */
enum status_code usart_init(
    struct usart_module *const module,
    sercom_registers_t *const hw,
    const struct usart_config *const config)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(hw);
    Assert(config);

    enum status_code status_code = STATUS_OK;

    /* Assign module pointer to software instance struct */
    module->hw = hw;

    /* Get a pointer to the hardware module instance */
    sercom_usart_int_registers_t *const usart_hw = &module->hw->USART_INT;

    uint32_t sercom_index = _sercom_get_sercom_inst_index(module->hw);
    struct sercom_apb_clock {
        enum system_clock_apb_bus apb_bus;
        uint32_t mask;
    } apb_index[] = {
        { SYSTEM_CLOCK_APB_APBA, MCLK_APBAMASK_SERCOM0(1), },
        { SYSTEM_CLOCK_APB_APBA, MCLK_APBAMASK_SERCOM1(1), },
        { SYSTEM_CLOCK_APB_APBB, MCLK_APBBMASK_SERCOM2(1), },
        { SYSTEM_CLOCK_APB_APBB, MCLK_APBBMASK_SERCOM3(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_SERCOM4(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_SERCOM5(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_SERCOM6(1), },
        { SYSTEM_CLOCK_APB_APBD, MCLK_APBDMASK_SERCOM7(1), },
    };
    uint32_t chan_index[] = {
        GCLK_PCHCTRL_ID_7_GCLK_SERCOM0_CORE,
        GCLK_PCHCTRL_ID_8_GCLK_SERCOM1_CORE,
        GCLK_PCHCTRL_ID_23_GCLK_SERCOM2_CORE,
        GCLK_PCHCTRL_ID_24_GCLK_SERCOM3_CORE,
        GCLK_PCHCTRL_ID_34_GCLK_SERCOM4_CORE,
        GCLK_PCHCTRL_ID_35_GCLK_SERCOM5_CORE,
        GCLK_PCHCTRL_ID_36_GCLK_SERCOM6_CORE,
        GCLK_PCHCTRL_ID_37_GCLK_SERCOM7_CORE,
    };

    if (usart_hw->SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_SWRST(1)) {
        /* The module is busy resetting itself */
        return STATUS_BUSY;
    }

    if (usart_hw->SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_ENABLE(1)) {
        /* Check the module is enabled */
        return STATUS_ERR_DENIED;
    }

    /* Turn on module in PM */
    system_apb_clock_set_mask(apb_index[sercom_index].apb_bus, apb_index[sercom_index].mask);

    /* Set up the GCLK for the module */
    struct system_gclk_chan_config gclk_chan_conf;
    system_gclk_chan_get_config_defaults(&gclk_chan_conf);
    gclk_chan_conf.source_generator = config->generator_source;
    system_gclk_chan_set_config(chan_index[sercom_index], &gclk_chan_conf);
    system_gclk_chan_enable(chan_index[sercom_index]);
    sercom_set_gclk_generator(config->generator_source, false);

    /* Set character size */
    module->character_size = config->character_size;

    /* Set transmitter and receiver status */
    module->receiver_enabled = config->receiver_enable;
    module->transmitter_enabled = config->transmitter_enable;

#ifdef FEATURE_USART_LIN_SLAVE
    module->lin_slave_enabled = config->lin_slave_enable;
#endif
#ifdef FEATURE_USART_START_FRAME_DECTION
    module->start_frame_detection_enabled = config->start_frame_detection_enable;
#endif
    /* Set configuration according to the config struct */
    status_code = _usart_set_config(module, config);
    if(status_code != STATUS_OK) {
        return status_code;
    }

    struct system_pinmux_config pin_conf;
    system_pinmux_get_config_defaults(&pin_conf);
    pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_INPUT;
    pin_conf.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;

    uint32_t pad_pinmuxes[] = {
        config->pinmux_pad0, config->pinmux_pad1,
        config->pinmux_pad2, config->pinmux_pad3
    };

    /* Configure the SERCOM pins according to the user configuration */
    for (uint8_t pad = 0; pad < 4; pad++) {
        uint32_t current_pinmux = pad_pinmuxes[pad];

        if (current_pinmux == PINMUX_DEFAULT) {
            current_pinmux = _sercom_get_default_pad(hw, pad);
        }

        if (current_pinmux != PINMUX_UNUSED) {
            pin_conf.mux_position = current_pinmux & 0xFFFF;
            system_pinmux_pin_set_config(current_pinmux >> 16, &pin_conf);
        }
    }

#if USART_CALLBACK_MODE == true
    /* Initialize parameters */
    for (uint32_t i = 0; i < USART_CALLBACK_N; i++) {
        module->callback[i]            = NULL;
    }

    module->tx_buffer_ptr              = NULL;
    module->rx_buffer_ptr              = NULL;
    module->remaining_tx_buffer_length = 0x0000;
    module->remaining_rx_buffer_length = 0x0000;
    module->callback_reg_mask          = 0x00;
    module->callback_enable_mask       = 0x00;
    module->rx_status                  = STATUS_OK;
    module->tx_status                  = STATUS_OK;

    /* Set interrupt handler and register USART software module struct in
     * look-up table */
    uint8_t instance_index = _sercom_get_sercom_inst_index(module->hw);
    _sercom_set_handler(instance_index, _usart_interrupt_handler);
    _sercom_instances[instance_index] = module;
#endif

    return status_code;
}

/**
 * \brief Enable the module
 *
 * Enables the USART module.
 *
 * \param[in]  module  Pointer to USART software instance struct
 */
void usart_enable(
    const struct usart_module *const module)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    /* Get a pointer to the hardware module instance */
    sercom_usart_int_registers_t *const usart_hw = &module->hw->USART_INT;

#if USART_CALLBACK_MODE == true
    /* Enable Global interrupt for module */
    NVIC_EnableIRQ(_sercom_get_interrupt_vector(module->hw));
#endif

    /* Wait until synchronization is complete */
    _usart_wait_for_sync(module);

    /* Enable USART module */
    usart_hw->SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE(1);
}

/**
 * \brief Disable module
 *
 * Disables the USART module.
 *
 * \param[in]  module  Pointer to USART software instance struct
 */
void usart_disable(
    const struct usart_module *const module)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    /* Get a pointer to the hardware module instance */
    sercom_usart_int_registers_t *const usart_hw = &module->hw->USART_INT;

#if USART_CALLBACK_MODE == true
    /* Disable Global interrupt for module */
    NVIC_DisableIRQ(_sercom_get_interrupt_vector(module->hw));
#endif
    /* Wait until synchronization is complete */
    _usart_wait_for_sync(module);

    /* Disable USART module */
    usart_hw->SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE(1);
}

/**
 * \brief Resets the USART module
 *
 * Disables and resets the USART module.
 *
 * \param[in]  module  Pointer to the USART software instance struct
 */
void usart_reset(
    const struct usart_module *const module)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    /* Get a pointer to the hardware module instance */
    sercom_usart_int_registers_t *const usart_hw = &module->hw->USART_INT;

    usart_disable(module);

    /* Wait until synchronization is complete */
    _usart_wait_for_sync(module);

    /* Reset module */
    usart_hw->SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST(1);
}

/**
 * \brief Transmit a character via the USART
 *
 * This blocking function will transmit a single character via the
 * USART.
 *
 * \param[in]  module   Pointer to the software instance struct
 * \param[in]  tx_data  Data to transfer
 *
 * \return Status of the operation.
 * \retval STATUS_OK         If the operation was completed
 * \retval STATUS_BUSY       If the operation was not completed, due to the USART
 *                           module being busy
 * \retval STATUS_ERR_DENIED If the transmitter is not enabled
 */
enum status_code usart_write_wait(
    struct usart_module *const module,
    const uint16_t tx_data)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    /* Get a pointer to the hardware module instance */
    sercom_usart_int_registers_t *const usart_hw = &module->hw->USART_INT;

    /* Check that the transmitter is enabled */
    if (!(module->transmitter_enabled)) {
        return STATUS_ERR_DENIED;
    }

#if USART_CALLBACK_MODE == true
    /* Check if the USART is busy doing asynchronous operation. */
    if (module->remaining_tx_buffer_length > 0) {
        return STATUS_BUSY;
    }

#else
    /* Check if USART is ready for new data */
    if (!(usart_hw->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE(1))) {
        /* Return error code */
        return STATUS_BUSY;
    }
#endif

    /* Wait until synchronization is complete */
    _usart_wait_for_sync(module);

    /* Write data to USART module */
    usart_hw->SERCOM_DATA = tx_data;

    while (!(usart_hw->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_TXC(1))) {
        /* Wait until data is sent */
    }

    return STATUS_OK;
}

/**
 * \brief Receive a character via the USART
 *
 * This blocking function will receive a character via the USART.
 *
 * \param[in]   module   Pointer to the software instance struct
 * \param[out]  rx_data  Pointer to received data
 *
 * \return Status of the operation.
 * \retval STATUS_OK                If the operation was completed
 * \retval STATUS_BUSY              If the operation was not completed,
 *                                  due to the USART module being busy
 * \retval STATUS_ERR_BAD_FORMAT    If the operation was not completed,
 *                                  due to configuration mismatch between USART
 *                                  and the sender
 * \retval STATUS_ERR_BAD_OVERFLOW  If the operation was not completed,
 *                                  due to the baudrate being too low or the
 *                                  system frequency being too high
 * \retval STATUS_ERR_BAD_DATA      If the operation was not completed, due to
 *                                  data being corrupted
 * \retval STATUS_ERR_DENIED        If the receiver is not enabled
 */
enum status_code usart_read_wait(
    struct usart_module *const module,
    uint16_t *const rx_data)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    /* Error variable */
    uint8_t error_code;

    /* Get a pointer to the hardware module instance */
    sercom_usart_int_registers_t *const usart_hw = &module->hw->USART_INT;

    /* Check that the receiver is enabled */
    if (!(module->receiver_enabled)) {
        return STATUS_ERR_DENIED;
    }

#if USART_CALLBACK_MODE == true
    /* Check if the USART is busy doing asynchronous operation. */
    if (module->remaining_rx_buffer_length > 0) {
        return STATUS_BUSY;
    }
#endif

    /* Check if USART has new data */
    if (!(usart_hw->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC(1))) {
        /* Return error code */
        return STATUS_BUSY;
    }

    /* Wait until synchronization is complete */
    _usart_wait_for_sync(module);

    /* Read out the status code and mask away all but the 3 LSBs*/
    error_code = (uint8_t)(usart_hw->SERCOM_STATUS & SERCOM_USART_INT_STATUS_Msk);

    /* Check if an error has occurred during the receiving */
    if (error_code) {
        /* Check which error occurred */
        if (error_code & SERCOM_USART_INT_STATUS_FERR(1)) {
            /* Clear flag by writing a 1 to it and
             * return with an error code */
            usart_hw->SERCOM_STATUS = SERCOM_USART_INT_STATUS_FERR(1);

            return STATUS_ERR_BAD_FORMAT;
        } else if (error_code & SERCOM_USART_INT_STATUS_BUFOVF(1)) {
            /* Clear flag by writing a 1 to it and
             * return with an error code */
            usart_hw->SERCOM_STATUS = SERCOM_USART_INT_STATUS_BUFOVF(1);

            return STATUS_ERR_OVERFLOW;
        } else if (error_code & SERCOM_USART_INT_STATUS_PERR(1)) {
            /* Clear flag by writing a 1 to it and
             * return with an error code */
            usart_hw->SERCOM_STATUS = SERCOM_USART_INT_STATUS_PERR(1);

            return STATUS_ERR_BAD_DATA;
        }
#ifdef FEATURE_USART_LIN_SLAVE
        else if (error_code & SERCOM_USART_INT_STATUS_ISF(1)) {
            /* Clear flag by writing 1 to it  and
             *  return with an error code */
            usart_hw->SERCOM_STATUS = SERCOM_USART_INT_STATUS_ISF(1);

            return STATUS_ERR_PROTOCOL;
        }
#endif
#ifdef FEATURE_USART_COLLISION_DECTION
        else if (error_code & SERCOM_USART_INT_STATUS_COLL(1)) {
            /* Clear flag by writing 1 to it
             *  return with an error code */
            usart_hw->SERCOM_STATUS = SERCOM_USART_INT_STATUS_COLL(1);

            return STATUS_ERR_PACKET_COLLISION;
        }
#endif
    }

    /* Read data from USART module */
    *rx_data = usart_hw->SERCOM_DATA;

    return STATUS_OK;
}

/**
 * \brief Transmit a buffer of characters via the USART
 *
 * This blocking function will transmit a block of \c length characters
 * via the USART.
 *
 * \note Using this function in combination with the interrupt (\c _job) functions is
 *       not recommended as it has no functionality to check if there is an
 *       ongoing interrupt driven operation running or not.
 *
 * \param[in]  module   Pointer to USART software instance struct
 * \param[in]  tx_data  Pointer to data to transmit
 * \param[in]  length   Number of characters to transmit
 *
 * \note If using 9-bit data, the array that *tx_data point to should be defined
 *       as uint16_t array and should be casted to uint8_t* pointer. Because it
 *       is an address pointer, the highest byte is not discarded. For example:
 *   \code
          #define TX_LEN 3
          uint16_t tx_buf[TX_LEN] = {0x0111, 0x0022, 0x0133};
          usart_write_buffer_wait(&module, (uint8_t*)tx_buf, TX_LEN);
    \endcode
 *
 * \return Status of the operation.
 * \retval STATUS_OK              If operation was completed
 * \retval STATUS_ERR_INVALID_ARG If operation was not completed, due to invalid
 *                                arguments
 * \retval STATUS_ERR_TIMEOUT     If operation was not completed, due to USART
 *                                module timing out
 * \retval STATUS_ERR_DENIED      If the transmitter is not enabled
 */
enum status_code usart_write_buffer_wait(
    struct usart_module *const module,
    const uint8_t *tx_data,
    uint16_t length)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    /* Check if the buffer length is valid */
    if (length == 0) {
        return STATUS_ERR_INVALID_ARG;
    }

    /* Check that the transmitter is enabled */
    if (!(module->transmitter_enabled)) {
        return STATUS_ERR_DENIED;
    }

    /* Get a pointer to the hardware module instance */
    sercom_usart_int_registers_t *const usart_hw = &module->hw->USART_INT;

    /* Wait until synchronization is complete */
    _usart_wait_for_sync(module);

    uint16_t tx_pos = 0;

    /* Blocks while buffer is being transferred */
    while (length--) {
        /* Wait for the USART to be ready for new data and abort
        * operation if it doesn't get ready within the timeout*/
        for (uint32_t i = 0; i <= USART_TIMEOUT; i++) {
            if (usart_hw->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE(1)) {
                break;
            } else if (i == USART_TIMEOUT) {
                return STATUS_ERR_TIMEOUT;
            }
        }

        /* Data to send is at least 8 bits long */
        uint16_t data_to_send = tx_data[tx_pos++];

        /* Check if the character size exceeds 8 bit */
        if (module->character_size == USART_CHARACTER_SIZE_9BIT) {
            data_to_send |= (tx_data[tx_pos++] << 8);
        }

        /* Send the data through the USART module */
        usart_write_wait(module, data_to_send);
    }

    /* Wait until Transmit is complete or timeout */
    for (uint32_t i = 0; i <= USART_TIMEOUT; i++) {
        if (usart_hw->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_TXC(1)) {
            break;
        } else if (i == USART_TIMEOUT) {
            return STATUS_ERR_TIMEOUT;
        }
    }

    return STATUS_OK;
}

/**
 * \brief Receive a buffer of \c length characters via the USART
 *
 * This blocking function will receive a block of \c length characters
 * via the USART.
 *
 * \note Using this function in combination with the interrupt (\c *_job)
 *       functions is not recommended as it has no functionality to check if
 *       there is an ongoing interrupt driven operation running or not.
 *
 * \param[in]  module   Pointer to USART software instance struct
 * \param[out] rx_data  Pointer to receive buffer
 * \param[in]  length   Number of characters to receive
 *
 * \note If using 9-bit data, the array that *rx_data point to should be defined
 *       as uint16_t array and should be casted to uint8_t* pointer. Because it
 *       is an address pointer, the highest byte is not discarded. For example:
 *   \code
          #define RX_LEN 3
          uint16_t rx_buf[RX_LEN] = {0x0,};
          usart_read_buffer_wait(&module, (uint8_t*)rx_buf, RX_LEN);
    \endcode
 *
 * \return Status of the operation.
 * \retval STATUS_OK                If operation was completed
 * \retval STATUS_ERR_INVALID_ARG   If operation was not completed, due to an
 *                                  invalid argument being supplied
 * \retval STATUS_ERR_TIMEOUT       If operation was not completed, due
 *                                  to USART module timing out
 * \retval STATUS_ERR_BAD_FORMAT    If the operation was not completed,
 *                                  due to a configuration mismatch
 *                                  between USART and the sender
 * \retval STATUS_ERR_BAD_OVERFLOW  If the operation was not completed,
 *                                  due to the baudrate being too low or the
 *                                  system frequency being too high
 * \retval STATUS_ERR_BAD_DATA      If the operation was not completed, due
 *                                  to data being corrupted
 * \retval STATUS_ERR_DENIED        If the receiver is not enabled
 */
enum status_code usart_read_buffer_wait(
    struct usart_module *const module,
    uint8_t *rx_data,
    uint16_t length)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    /* Check if the buffer length is valid */
    if (length == 0) {
        return STATUS_ERR_INVALID_ARG;
    }

    /* Check that the receiver is enabled */
    if (!(module->receiver_enabled)) {
        return STATUS_ERR_DENIED;
    }

    /* Get a pointer to the hardware module instance */
    sercom_usart_int_registers_t *const usart_hw = &module->hw->USART_INT;

    uint16_t rx_pos = 0;

    /* Blocks while buffer is being received */
    while (length--) {
        /* Wait for the USART to have new data and abort operation if it
         * doesn't get ready within the timeout*/
        for (uint32_t i = 0; i <= USART_TIMEOUT; i++) {
            if (usart_hw->SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC(1)) {
                break;
            } else if (i == USART_TIMEOUT) {
                return STATUS_ERR_TIMEOUT;
            }
        }

        enum status_code retval;
        uint16_t received_data = 0;

        retval = usart_read_wait(module, &received_data);

        if (retval != STATUS_OK) {
            /* Overflow, abort */
            return retval;
        }

        /* Read value will be at least 8-bits long */
        rx_data[rx_pos++] = received_data;

        /* If 9-bit data, write next received byte to the buffer */
        if (module->character_size == USART_CHARACTER_SIZE_9BIT) {
            rx_data[rx_pos++] = (received_data >> 8);
        }
    }

    return STATUS_OK;
}
