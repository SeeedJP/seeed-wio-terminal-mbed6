/**
 * \file
 *
 * \brief SAM I2C Slave Driver
 *
 * Copyright (C) 2013-2015 Atmel Corporation. All rights reserved.
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

#include "cmsis.h"
#include "i2c_slave.h"
#if I2C_SLAVE_CALLBACK_MODE == true
#  include "i2c_slave_interrupt.h"
#endif
#include "sercom_drv.h"

/**
 * \internal Sets configuration to module
 *
 * \param[out] module  Pointer to software module structure
 * \param[in]  config  Configuration structure with configurations to set
 *
 * \return Status of setting configuration.
 * \retval STATUS_OK                       Module was configured correctly
 * \retval STATUS_ERR_ALREADY_INITIALIZED  If setting other GCLK generator than
 *                                         previously set
 */
static enum status_code _i2c_slave_set_config(
    struct i2c_slave_module *const module,
    const struct i2c_slave_config *const config)
{
    uint32_t tmp_ctrla;

    /* Sanity check arguments. */
    Assert(module);
    Assert(module->hw);
    Assert(config);

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);
    sercom_registers_t *const sercom_hw = module->hw;

    module->buffer_timeout = config->buffer_timeout;
    module->ten_bit_address = config->ten_bit_address;

    struct system_pinmux_config pin_conf;
    system_pinmux_get_config_defaults(&pin_conf);

    uint32_t pad0 = config->pinmux_pad0;
    uint32_t pad1 = config->pinmux_pad1;

    /* SERCOM PAD0 - SDA */
    if (pad0 == PINMUX_DEFAULT) {
        pad0 = _sercom_get_default_pad(sercom_hw, 0);
    }
    pin_conf.mux_position = pad0 & 0xFFFF;
    pin_conf.direction    = SYSTEM_PINMUX_PIN_DIR_OUTPUT_WITH_READBACK;
    system_pinmux_pin_set_config(pad0 >> 16, &pin_conf);

    /* SERCOM PAD1 - SCL */
    if (pad1 == PINMUX_DEFAULT) {
        pad1 = _sercom_get_default_pad(sercom_hw, 1);
    }
    pin_conf.mux_position = pad1 & 0xFFFF;
    pin_conf.direction    = SYSTEM_PINMUX_PIN_DIR_OUTPUT_WITH_READBACK;
    system_pinmux_pin_set_config(pad1 >> 16, &pin_conf);

    /* Prepare config to write to register CTRLA */
    if (config->run_in_standby || system_is_debugger_present()) {
        tmp_ctrla = SERCOM_I2CS_CTRLA_RUNSTDBY(1);
    } else {
        tmp_ctrla = 0;
    }

    tmp_ctrla |= ((uint32_t)config->sda_hold_time |
                  config->transfer_speed |
                  (config->scl_low_timeout << SERCOM_I2CS_CTRLA_LOWTOUTEN_Pos) |
                  (config->scl_stretch_only_after_ack_bit << SERCOM_I2CS_CTRLA_SCLSM_Pos) |
                  (config->slave_scl_low_extend_timeout << SERCOM_I2CS_CTRLA_SEXTTOEN_Pos));

    i2c_hw->SERCOM_CTRLA |= tmp_ctrla;

    /* Set CTRLB configuration */
    i2c_hw->SERCOM_CTRLB = SERCOM_I2CS_CTRLB_SMEN(1) | config->address_mode;

    i2c_hw->SERCOM_ADDR = config->address << SERCOM_I2CS_ADDR_ADDR_Pos |
                       config->address_mask << SERCOM_I2CS_ADDR_ADDRMASK_Pos |
                       config->ten_bit_address << SERCOM_I2CS_ADDR_TENBITEN_Pos |
                       config->enable_general_call_address << SERCOM_I2CS_ADDR_GENCEN_Pos;

    return STATUS_OK;
}

/**
 * \brief Initializes the requested I<SUP>2</SUP>C hardware module
 *
 * Initializes the SERCOM I<SUP>2</SUP>C Slave device requested and sets the provided
 * software module struct.  Run this function before any further use of
 * the driver.
 *
 * \param[out] module  Pointer to software module struct
 * \param[in]  hw      Pointer to the hardware instance
 * \param[in]  config  Pointer to the configuration struct
 *
 * \return Status of initialization.
 * \retval STATUS_OK                       Module initiated correctly
 * \retval STATUS_ERR_DENIED               If module is enabled
 * \retval STATUS_BUSY                     If module is busy resetting
 * \retval STATUS_ERR_ALREADY_INITIALIZED  If setting other GCLK generator than
 *                                         previously set
 */
enum status_code i2c_slave_init(
    struct i2c_slave_module *const module,
    sercom_registers_t *const hw,
    const struct i2c_slave_config *const config)
{
    /* Sanity check arguments. */
    Assert(module);
    Assert(hw);
    Assert(config);

    /* Initialize software module */
    module->hw = hw;

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);

    /* Check if module is enabled. */
    if (i2c_hw->SERCOM_CTRLA & SERCOM_I2CS_CTRLA_ENABLE(1)) {
        return STATUS_ERR_DENIED;
    }

    /* Check if reset is in progress. */
    if (i2c_hw->SERCOM_CTRLA & SERCOM_I2CS_CTRLA_SWRST(1)) {
        return STATUS_BUSY;
    }

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

    /* Turn on module in PM */
    system_apb_clock_set_mask(apb_index[sercom_index].apb_bus, apb_index[sercom_index].mask);

    /* Set up the GCLK for the module */
    struct system_gclk_chan_config gclk_chan_conf;
    system_gclk_chan_get_config_defaults(&gclk_chan_conf);
    gclk_chan_conf.source_generator = config->generator_source;
    system_gclk_chan_set_config(chan_index[sercom_index], &gclk_chan_conf);
    system_gclk_chan_enable(chan_index[sercom_index]);
    sercom_set_gclk_generator(config->generator_source, false);

#if I2C_SLAVE_CALLBACK_MODE == true
    /* Get sercom instance index. */
    uint8_t instance_index = _sercom_get_sercom_inst_index(module->hw);

    /* Save software module in interrupt handler. */
    _sercom_set_handler(instance_index, _i2c_slave_interrupt_handler);

    /* Save software module. */
    _sercom_instances[instance_index] = module;

    /* Initialize values in module. */
    module->registered_callback = 0;
    module->enabled_callback = 0;
    module->buffer_length = 0;
    module->nack_on_address = config->enable_nack_on_address;
#endif

    /* Set SERCOM module to operate in I2C slave mode. */
    i2c_hw->SERCOM_CTRLA = SERCOM_I2CS_CTRLA_MODE(0x4);

    /* Set config and return status. */
    return _i2c_slave_set_config(module, config);
}

/**
 * \brief Enables the I<SUP>2</SUP>C module
 *
 * This will enable the requested I<SUP>2</SUP>C module.
 *
 * \param[in]  module Pointer to the software module struct
 */
void i2c_slave_enable(
    const struct i2c_slave_module *const module)
{
    /* Sanity check of arguments. */
    Assert(module);
    Assert(module->hw);

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);

#if I2C_SLAVE_CALLBACK_MODE == true
    /* Enable global interrupt for module */
    NVIC_EnableIRQ(_sercom_get_interrupt_vector(module->hw));
#endif

    /* Wait for module to sync */
    _i2c_slave_wait_for_sync(module);

    /* Enable module */
    i2c_hw->SERCOM_CTRLA |= SERCOM_I2CS_CTRLA_ENABLE(1);
}


/**
 * \brief Disables the I<SUP>2</SUP>C module
 *
 * This will disable the I<SUP>2</SUP>C module specified in the provided software module
 * structure.
 *
 * \param[in]  module  Pointer to the software module struct
 */
void i2c_slave_disable(
    const struct i2c_slave_module *const module)
{
    /* Sanity check of arguments. */
    Assert(module);
    Assert(module->hw);

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);

#if I2C_SLAVE_CALLBACK_MODE == true
    /* Disable interrupts */
    i2c_hw->SERCOM_INTENCLR = SERCOM_I2CS_INTENSET_PREC(1) |
                           SERCOM_I2CS_INTENSET_AMATCH(1) | SERCOM_I2CS_INTENSET_DRDY(1);

    /* Clear interrupt flags */
    i2c_hw->SERCOM_INTFLAG = SERCOM_I2CS_INTFLAG_PREC(1) | SERCOM_I2CS_INTFLAG_AMATCH(1) |
                          SERCOM_I2CS_INTFLAG_DRDY(1);

    /* Disable global interrupt for module */
    NVIC_DisableIRQ(_sercom_get_interrupt_vector(module->hw));
#endif

    /* Wait for module to sync */
    _i2c_slave_wait_for_sync(module);

    /* Disable module */
    i2c_hw->SERCOM_CTRLA &= ~SERCOM_I2CS_CTRLA_ENABLE(1);
}

/**
 * \brief Resets the hardware module
 *
 * This will reset the module to hardware defaults.
 *
 * \param[in,out] module  Pointer to software module structure
 */
void i2c_slave_reset(
    struct i2c_slave_module *const module)
{
    /* Sanity check arguments. */
    Assert(module);
    Assert(module->hw);

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);

#if I2C_SLAVE_CALLBACK_MODE == true
    /* Reset module instance. */
    module->registered_callback = 0;
    module->enabled_callback = 0;
    module->buffer_length = 0;
    module->buffer_remaining = 0;
    module->buffer = NULL;
#endif

    /* Disable module */
    i2c_slave_disable(module);

#if I2C_SLAVE_CALLBACK_MODE == true
    /* Clear all pending interrupts. */
    system_interrupt_enter_critical_section();
    system_interrupt_clear_pending(_sercom_get_interrupt_vector(module->hw));
    system_interrupt_leave_critical_section();
#endif

    /* Wait for sync. */
    _i2c_slave_wait_for_sync(module);

    /* Reset module. */
    i2c_hw->SERCOM_CTRLA = SERCOM_I2CS_CTRLA_SWRST(1);
}

/**
 * \internal Waits for answer on bus
 *
 * \param[in]  module  Pointer to software module structure
 *
 * \return Status of bus.
 * \retval STATUS_OK           If given response from slave device
 * \retval STATUS_ERR_TIMEOUT  If no response was given within specified timeout
 *                             period
 */
static enum status_code _i2c_slave_wait_for_bus(
    struct i2c_slave_module *const module)
{
    /* Sanity check arguments. */
    Assert(module);
    Assert(module->hw);

    sercom_i2cs_registers_t *const i2c_module = &(module->hw->I2CS);

    /* Wait for reply. */
    uint16_t timeout_counter = 0;
    while ((!(i2c_module->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_DRDY(1))) &&
            (!(i2c_module->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_PREC(1))) &&
            (!(i2c_module->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_AMATCH(1)))) {

        /* Check timeout condition. */
        if (++timeout_counter >= module->buffer_timeout) {
            return STATUS_ERR_TIMEOUT;
        }
    }
    return STATUS_OK;
}

/**
 * \brief Writes a packet to the master
 *
 * Writes a packet to the master. This will wait for the master to issue
 * a request.
 *
 * \param[in]  module  Pointer to software module structure
 * \param[in]  packet  Packet to write to master
 *
 * \return Status of packet write.
 * \retval STATUS_OK                Packet was written successfully
 * \retval STATUS_ERR_DENIED        Start condition not received, another
 *                                  interrupt flag is set
 * \retval STATUS_ERR_IO            There was an error in the previous transfer
 * \retval STATUS_ERR_BAD_FORMAT    Master wants to write data
 * \retval STATUS_ERR_INVALID_ARG   Invalid argument(s) was provided
 * \retval STATUS_ERR_BUSY          The I<SUP>2</SUP>C module is busy with a job
 * \retval STATUS_ERR_ERR_OVERFLOW  Master NACKed before entire packet was
 *                                  transferred
 * \retval STATUS_ERR_TIMEOUT       No response was given within the timeout
 *                                  period
 */
enum status_code i2c_slave_write_packet_wait(
    struct i2c_slave_module *const module,
    struct i2c_slave_packet *const packet)
{
    /* Sanity check arguments. */
    Assert(module);
    Assert(module->hw);
    Assert(packet);

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);

    uint16_t length = packet->data_length;

    if (length == 0) {
        return STATUS_ERR_INVALID_ARG;
    }

#if I2C_SLAVE_CALLBACK_MODE == true
    /* Check if the module is busy with a job or AMATCH is enabled */
    if (module->buffer_remaining > 0 ||
            (i2c_hw->SERCOM_INTENSET & SERCOM_I2CS_INTFLAG_AMATCH(1))) {
        return STATUS_BUSY;
    }
#endif

    enum status_code status;
    /* Wait for master to send address packet */
    status = _i2c_slave_wait_for_bus(module);

    if (status != STATUS_OK) {
        /* Timeout, return */
        return status;
    }
    if (!(i2c_hw->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_AMATCH(1))) {
        /* Not address interrupt, something is wrong */
        return STATUS_ERR_DENIED;
    }

    if (module->ten_bit_address) {
        /* ACK the first address */
        i2c_hw->SERCOM_CTRLB &= ~SERCOM_I2CS_CTRLB_ACKACT(1);
        i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_CMD(0x3);

        /* Wait for address interrupt */
        status = _i2c_slave_wait_for_bus(module);

        if (status != STATUS_OK) {
            /* Timeout, return */
            return STATUS_ERR_TIMEOUT;
        }

        if (!(i2c_hw->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_AMATCH(1))) {
            /* Not address interrupt, something is wrong */
            return STATUS_ERR_DENIED;
        }
    }

    /* Check if there was an error in last transfer */
    if (i2c_hw->SERCOM_STATUS & (SERCOM_I2CS_STATUS_BUSERR(1) |
                              SERCOM_I2CS_STATUS_COLL(1) | SERCOM_I2CS_STATUS_LOWTOUT(1))) {
        return STATUS_ERR_IO;
    }

    /* Check direction */
    if (!(i2c_hw->SERCOM_STATUS & SERCOM_I2CS_STATUS_DIR(1))) {
        /* Write request from master, send NACK and return */
        i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_ACKACT(1);
        i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_CMD(0x3);
        return STATUS_ERR_BAD_FORMAT;
    }

    /* Read request from master, ACK address */
    i2c_hw->SERCOM_CTRLB &= ~SERCOM_I2CS_CTRLB_ACKACT(1);
    i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_CMD(0x3);

    uint16_t i = 0;

    /* Wait for data interrupt */
    status = _i2c_slave_wait_for_bus(module);
    if (status != STATUS_OK) {
        /* Timeout, return */
        return status;
    }

    while (length--) {
        /* Write data */
        _i2c_slave_wait_for_sync(module);
        i2c_hw->SERCOM_DATA = packet->data[i++];

        /* Wait for response from master */
        status = _i2c_slave_wait_for_bus(module);

        if (status != STATUS_OK) {
            /* Timeout, return */
            return status;
        }

        if (i2c_hw->SERCOM_STATUS & SERCOM_I2CS_STATUS_RXNACK(1) &&
                length !=0) {
            /* NACK from master, abort */
            /* Release line */
            i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_CMD(0x02);

            return STATUS_ERR_OVERFLOW;
        }
        /* ACK from master, continue writing */
    }

    /* Release line */
    i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_CMD(0x02);

    return STATUS_OK;
}

/**
 * \brief Reads a packet from the master
 *
 * Reads a packet from the master. This will wait for the master to issue a
 * request.
 *
 * \param[in]  module  Pointer to software module structure
 * \param[out] packet  Packet to read from master
 *
 * \return Status of packet read.
 * \retval STATUS_OK                Packet was read successfully
 * \retval STATUS_ABORTED           Master sent stop condition or repeated
 *                                  start before specified length of bytes
 *                                  was received
 * \retval STATUS_ERR_IO            There was an error in the previous transfer
 * \retval STATUS_ERR_DENIED        Start condition not received, another
 *                                  interrupt flag is set
 * \retval STATUS_ERR_INVALID_ARG   Invalid argument(s) was provided
 * \retval STATUS_ERR_BUSY          The I<SUP>2</SUP>C module is busy with a job
 * \retval STATUS_ERR_BAD_FORMAT    Master wants to read data
 * \retval STATUS_ERR_ERR_OVERFLOW  Last byte received overflows buffer
 */
enum status_code i2c_slave_read_packet_wait(
    struct i2c_slave_module *const module,
    struct i2c_slave_packet *const packet)
{
    /* Sanity check arguments. */
    Assert(module);
    Assert(module->hw);
    Assert(packet);

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);

    uint16_t length = packet->data_length;

    if (length == 0) {
        return STATUS_ERR_INVALID_ARG;
    }

#if I2C_SLAVE_CALLBACK_MODE == true
    /* Check if the module is busy with a job or AMATCH is enabled */
    if (module->buffer_remaining > 0 ||
            (i2c_hw->SERCOM_INTENSET & SERCOM_I2CS_INTFLAG_AMATCH(1))) {
        return STATUS_BUSY;
    }
#endif

    enum status_code status;

    /* Wait for master to send address packet */
    status = _i2c_slave_wait_for_bus(module);
    if (status != STATUS_OK) {
        /* Timeout, return */
        return status;
    }

    if (!(i2c_hw->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_AMATCH(1))) {
        /* Not address interrupt, something is wrong */
        return STATUS_ERR_DENIED;
    }

    /* Check if there was an error in the last transfer */
    if (i2c_hw->SERCOM_STATUS & (SERCOM_I2CS_STATUS_BUSERR(1) |
                              SERCOM_I2CS_STATUS_COLL(1) | SERCOM_I2CS_STATUS_LOWTOUT(1))) {
        return STATUS_ERR_IO;
    }
    /* Check direction */
    if ((i2c_hw->SERCOM_STATUS & SERCOM_I2CS_STATUS_DIR(1))) {
        /* Read request from master, send NACK and return */
        i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_ACKACT(1);
        i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_CMD(0x3);
        return STATUS_ERR_BAD_FORMAT;
    }

    /* Write request from master, ACK address */
    i2c_hw->SERCOM_CTRLB &= ~SERCOM_I2CS_CTRLB_ACKACT(1);
    i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_CMD(0x3);

    uint16_t i = 0;
    while (length--) {

        /* Wait for next byte or stop condition */
        status = _i2c_slave_wait_for_bus(module);
        if (status != STATUS_OK) {
            /* Timeout, return */
            return status;
        }

        if ((i2c_hw->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_PREC(1)) ||
                i2c_hw->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_AMATCH(1)) {
            /* Master sent stop condition, or repeated start, read done */
            /* Clear stop flag */
            i2c_hw->SERCOM_INTFLAG = SERCOM_I2CS_INTFLAG_PREC(1);
            return STATUS_ABORTED;
        }

        /* Read data */
        _i2c_slave_wait_for_sync(module);
        packet->data[i++] = i2c_hw->SERCOM_DATA;

    }

    /* Packet read done, wait for packet to NACK, Stop or repeated start */
    status = _i2c_slave_wait_for_bus(module);

    if (i2c_hw->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_DRDY(1)) {
        /* Buffer is full, send NACK */
        i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_ACKACT(1);
        i2c_hw->SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_CMD(0x2);
    }
    if (i2c_hw->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_PREC(1)) {
        /* Clear stop flag */
        i2c_hw->SERCOM_INTFLAG = SERCOM_I2CS_INTFLAG_PREC(1);
    }
    return STATUS_OK;
}

/**
 * \brief Waits for a start condition on the bus
 *
 * \note This function is only available for 7-bit slave addressing.
 *
 * Waits for the master to issue a start condition on the bus.
 * Note that this function does not check for errors in the last transfer,
 * this will be discovered when reading or writing.
 *
 * \param[in]  module  Pointer to software module structure
 *
 * \return Direction of the current transfer, when in slave mode.
 * \retval I2C_SLAVE_DIRECTION_NONE   No request from master within timeout
 *                                    period
 * \retval I2C_SLAVE_DIRECTION_READ   Write request from master
 * \retval I2C_SLAVE_DIRECTION_WRITE  Read request from master
 */
enum i2c_slave_direction i2c_slave_get_direction_wait(
    struct i2c_slave_module *const module)
{
    /* Sanity check arguments. */
    Assert(module);
    Assert(module->hw);

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);

    enum status_code status;

    /* Wait for address interrupt */
    status = _i2c_slave_wait_for_bus(module);

    if (status != STATUS_OK) {
        /* Timeout, return */
        return I2C_SLAVE_DIRECTION_NONE;
    }

    if (!(i2c_hw->SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_AMATCH(1))) {
        /* Not address interrupt, something is wrong */
        return I2C_SLAVE_DIRECTION_NONE;
    }

    /* Check direction */
    if ((i2c_hw->SERCOM_STATUS & SERCOM_I2CS_STATUS_DIR(1))) {
        /* Read request from master */
        return I2C_SLAVE_DIRECTION_WRITE;
    } else {
        /* Write request from master */
        return I2C_SLAVE_DIRECTION_READ;
    }
}

/**
 * \brief Retrieves the current module status
 *
 * Checks the status of the module and returns it as a bitmask of status
 * flags.
 *
 * \param[in] module      Pointer to the I<SUP>2</SUP>C slave software device struct
 *
 * \return Bitmask of status flags.
 *
 * \retval I2C_SLAVE_STATUS_ADDRESS_MATCH   A valid address has been received
 * \retval I2C_SLAVE_STATUS_DATA_READY      A I<SUP>2</SUP>C slave byte transmission is
 *                                          successfully completed
 * \retval I2C_SLAVE_STATUS_STOP_RECEIVED   A stop condition is detected for a
 *                                          transaction being processed
 * \retval I2C_SLAVE_STATUS_CLOCK_HOLD      The slave is holding the SCL line
 *                                          low
 * \retval I2C_SLAVE_STATUS_SCL_LOW_TIMEOUT An SCL low time-out has occurred
 * \retval I2C_SLAVE_STATUS_REPEATED_START  Indicates a repeated start, only
 *                                          valid if \ref
 *                                          I2C_SLAVE_STATUS_ADDRESS_MATCH is
 *                                          set
 * \retval I2C_SLAVE_STATUS_RECEIVED_NACK   The last data packet sent was not
 *                                          acknowledged
 * \retval I2C_SLAVE_STATUS_COLLISION       The I<SUP>2</SUP>C slave was not able to
 *                                          transmit a high data or NACK bit
 * \retval I2C_SLAVE_STATUS_BUS_ERROR       An illegal bus condition has
 *                                          occurred on the bus
 */
uint32_t i2c_slave_get_status(
    struct i2c_slave_module *const module)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);

    uint8_t intflags = i2c_hw->SERCOM_INTFLAG;
    uint8_t status = i2c_hw->SERCOM_STATUS;
    uint32_t status_flags = 0;

    /* Check Address Match flag */
    if (intflags & SERCOM_I2CS_INTFLAG_AMATCH(1)) {
        status_flags |= I2C_SLAVE_STATUS_ADDRESS_MATCH;
    }
    /* Check Data Ready flag */
    if (intflags & SERCOM_I2CS_INTFLAG_DRDY(1)) {
        status_flags |= I2C_SLAVE_STATUS_DATA_READY;
    }
    /* Check Stop flag */
    if (intflags & SERCOM_I2CS_INTFLAG_PREC(1)) {
        status_flags |= I2C_SLAVE_STATUS_STOP_RECEIVED;
    }
    /* Check Clock Hold */
    if (status & SERCOM_I2CS_STATUS_CLKHOLD(1)) {
        status_flags |= I2C_SLAVE_STATUS_CLOCK_HOLD;
    }
    /* Check SCL Low Timeout */
    if (status & SERCOM_I2CS_STATUS_LOWTOUT(1)) {
        status_flags |= I2C_SLAVE_STATUS_SCL_LOW_TIMEOUT;
    }
    /* Check Repeated Start */
    if (status & SERCOM_I2CS_STATUS_SR(1)) {
        status_flags |= I2C_SLAVE_STATUS_REPEATED_START;
    }
    /* Check Received Not Acknowledge */
    if (status & SERCOM_I2CS_STATUS_RXNACK(1)) {
        status_flags |= I2C_SLAVE_STATUS_RECEIVED_NACK;
    }
    /* Check Transmit Collision */
    if (status & SERCOM_I2CS_STATUS_COLL(1)) {
        status_flags |= I2C_SLAVE_STATUS_COLLISION;
    }
    /* Check Bus Error */
    if (status & SERCOM_I2CS_STATUS_BUSERR(1)) {
        status_flags |= I2C_SLAVE_STATUS_BUS_ERROR;
    }

    return status_flags;
}

/**
 * \brief Clears a module status flag
 *
 * Clears the given status flag of the module.
 *
 * \note Not all status flags can be cleared.
 *
 * \param[in] module         Pointer to the I<SUP>2</SUP>C software device struct
 * \param[in] status_flags   Bit mask of status flags to clear
 *
 */
void i2c_slave_clear_status(
    struct i2c_slave_module *const module,
    uint32_t status_flags)
{
    /* Sanity check arguments */
    Assert(module);
    Assert(module->hw);

    sercom_i2cs_registers_t *const i2c_hw = &(module->hw->I2CS);

    /* Clear Address Match flag */
    if (status_flags & I2C_SLAVE_STATUS_ADDRESS_MATCH) {
        i2c_hw->SERCOM_INTFLAG = SERCOM_I2CS_INTFLAG_AMATCH(1);
    }
    /* Clear Data Ready flag */
    if (status_flags & I2C_SLAVE_STATUS_DATA_READY) {
        i2c_hw->SERCOM_INTFLAG = SERCOM_I2CS_INTFLAG_DRDY(1);
    }
    /* Clear Stop flag */
    if (status_flags & I2C_SLAVE_STATUS_STOP_RECEIVED) {
        i2c_hw->SERCOM_INTFLAG = SERCOM_I2CS_INTFLAG_PREC(1);
    }
    /* Clear SCL Low Timeout */
    if (status_flags & I2C_SLAVE_STATUS_SCL_LOW_TIMEOUT) {
        i2c_hw->SERCOM_STATUS = SERCOM_I2CS_STATUS_LOWTOUT(1);
    }
    /* Clear Transmit Collision */
    if (status_flags & I2C_SLAVE_STATUS_COLLISION) {
        i2c_hw->SERCOM_STATUS = SERCOM_I2CS_STATUS_COLL(1);
    }
    /* Clear Bus Error */
    if (status_flags & I2C_SLAVE_STATUS_BUS_ERROR) {
        i2c_hw->SERCOM_STATUS = SERCOM_I2CS_STATUS_BUSERR(1);
    }
}
