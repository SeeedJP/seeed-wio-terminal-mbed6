/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
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
#ifdef DEVICE_SPI
#include "mbed_assert.h"
#include "spi_api.h"

#include "cmsis.h"
#include "pinmap.h"
#include "sercom_drv.h"

#include "pinmap_function.h"

#define SPI_MOSI_INDEX	0
#define SPI_MISO_INDEX	1
#define SPI_SCLK_INDEX	2
#define SPI_SSEL_INDEX	3

/**
 * \brief SPI modes enum
 *
 * SPI mode selection.
 */
enum spi_mode {
    /** Master mode. */
    SPI_MODE_MASTER         = 1,
    /** Slave mode. */
    SPI_MODE_SLAVE          = 0,
};

#if DEVICE_SPI_ASYNCH
#define pSPI_S(obj)			(&obj->spi)
#define pSPI_SERCOM(obj)	obj->spi.spi
#else
#define pSPI_S(obj)			(obj)
#define pSPI_SERCOM(obj)	(obj->spi)
#endif
#define _SPI(obj)			pSPI_SERCOM(obj)->SPIM

/** SPI default baud rate. */
#define SPI_DEFAULT_BAUD	100000


/** SPI timeout value. */
#  define SPI_TIMEOUT 10000

extern uint8_t g_sys_init;
uint16_t dummy_fill_word = 0xFFFF;


static inline bool spi_is_syncing(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Return synchronization status */
    return (_SPI(obj).SERCOM_SYNCBUSY);
}

static inline void spi_enable(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Enable SPI */
    _SPI(obj).SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_ENABLE(1);

    /* Wait until the synchronization is complete */
    while (spi_is_syncing(obj));
}

static inline void spi_disable(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Disable SPI */
    _SPI(obj).SERCOM_CTRLA &= ~SERCOM_SPIM_CTRLA_ENABLE(1);

    /* Wait until the synchronization is complete */
    while (spi_is_syncing(obj));
}

static inline bool spi_is_write_complete(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Check interrupt flag */
    return (_SPI(obj).SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_TXC(1));
}

static inline bool spi_is_ready_to_write(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Check interrupt flag */
    return (_SPI(obj).SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_DRE(1));
}

static inline bool spi_is_ready_to_read(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Check interrupt flag */
    return (_SPI(obj).SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC(1));
}

static inline bool spi_write(spi_t *obj, uint16_t tx_data)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Check if the data register has been copied to the shift register */
    if (!spi_is_ready_to_write(obj)) {
        /* Data register has not been copied to the shift register, return */
        return false;
    }

    /* Write the character to the DATA register */
    _SPI(obj).SERCOM_DATA = tx_data & SERCOM_SPIM_DATA_DATA_Msk;

    return true;
}

static inline bool spi_read(spi_t *obj, uint16_t *rx_data)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Check if data is ready to be read */
    if (!spi_is_ready_to_read(obj)) {
        /* No data has been received, return */
        return false;
    }

    /* Check if data is overflown */
    if (_SPI(obj).SERCOM_STATUS & SERCOM_SPIM_STATUS_BUFOVF(1)) {
        /* Clear overflow flag */
        _SPI(obj).SERCOM_STATUS |= SERCOM_SPIM_STATUS_BUFOVF(1);
    }

    /* Read the character from the DATA register */
    if ((_SPI(obj).SERCOM_CTRLB & SERCOM_SPIM_CTRLB_CHSIZE_Msk) == SERCOM_SPIM_CTRLB_CHSIZE_9_BIT) {
        *rx_data = _SPI(obj).SERCOM_DATA & SERCOM_SPIM_DATA_DATA_Msk;
    } else {
        *rx_data = (uint8_t)_SPI(obj).SERCOM_DATA;
    }

    return true;
}

static uint32_t spi_find_mux_settings(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    uint8_t i_dipo;
    uint8_t i_dopo;
    uint32_t dipo = 0;
    uint32_t dopo = 0;
    uint32_t mux_pad;

    uint32_t mux_settings = 0;

    uint32_t sercom_index = _sercom_get_sercom_inst_index(pSPI_SERCOM(obj));

    if ((_SPI(obj).SERCOM_CTRLA & SERCOM_SPIM_CTRLA_MODE_Msk) == SERCOM_SPIM_CTRLA_MODE_SPI_MASTER) {
        i_dipo = SPI_MISO_INDEX;
        i_dopo = SPI_MOSI_INDEX;
    } else {
        i_dipo = SPI_MOSI_INDEX;
        i_dopo = SPI_MISO_INDEX;
    }

    /* Find MUX setting */
    if (pSPI_S(obj)->pins[i_dipo] != NC) {
        /* Set Data input MUX padding for master */
        mux_pad = pinmap_pad_sercom(pSPI_S(obj)->pins[i_dipo], sercom_index);
        if (mux_pad != (uint32_t)NC) {
            /* MUX pad value is same as DIPO value */
            dipo = mux_pad;
            mux_settings |= SERCOM_SPIM_CTRLA_DIPO(dipo);
        }
    }

    if (pSPI_S(obj)->pins[i_dopo] != NC) {
        /* Set Data output MUX padding for master */
        mux_pad = pinmap_pad_sercom(pSPI_S(obj)->pins[i_dopo], sercom_index);
        if (mux_pad != (uint32_t)NC) {
            if (mux_pad != 0) {
                dopo = mux_pad - 1;
            } else {
                if (3 == pinmap_pad_sercom(pSPI_S(obj)->pins[SPI_SCLK_INDEX], sercom_index)) {
                    dopo = 3;
                } else {
                    dopo = 0;
                }
            }
            mux_settings |= SERCOM_SPIM_CTRLA_DOPO(dopo);
        }
    }

    return mux_settings;
}

/**
 * \defgroup GeneralSPI SPI Configuration Functions
 * @{
 */

/** Initialize the SPI peripheral
 *
 * Configures the pins used by SPI, sets a default format and frequency, and enables the peripheral
 * @param[out] obj  The SPI object to initialize
 * @param[in]  mosi The pin to use for MOSI
 * @param[in]  miso The pin to use for MISO
 * @param[in]  sclk The pin to use for SCLK
 * @param[in]  ssel The pin to use for SSEL
 */
void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    MBED_ASSERT(sclk != NC);

    uint16_t baud = 0;
    uint32_t ctrla = 0;
    uint32_t ctrlb = 0;
    enum status_code error_code;

    if (g_sys_init == 0) {
        system_init();
        g_sys_init = 1;
    }

    /* Calculate SERCOM instance from pins */
    uint32_t sercom_index = pinmap_find_sercom(mosi, miso, sclk, ssel);
    pSPI_SERCOM(obj) = (sercom_registers_t*)pinmap_peripheral_sercom(NC, sercom_index);

    /* Disable SPI */
    spi_disable(obj);

    /* Check if reset is in progress. */
    if (_SPI(obj).SERCOM_CTRLA & SERCOM_SPIM_CTRLA_SWRST(1)) {
        return;
    }

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
    gclk_chan_conf.source_generator = GCLK_GENERATOR_1;
    system_gclk_chan_set_config(chan_index[sercom_index], &gclk_chan_conf);
    system_gclk_chan_enable(chan_index[sercom_index]);
    sercom_set_gclk_generator(GCLK_GENERATOR_0, false);

    /* Set the SERCOM in SPI master mode */
    _SPI(obj).SERCOM_CTRLA |= SERCOM_I2CS_CTRLA_MODE_SPI_MASTER;
    pSPI_S(obj)->mode = SPI_MODE_MASTER;

    /* TODO: Do pin muxing here */
    struct system_pinmux_config pin_conf;
    system_pinmux_get_config_defaults(&pin_conf);
    pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_INPUT;

    pSPI_S(obj)->pins[SPI_MOSI_INDEX] = mosi;
    pSPI_S(obj)->pins[SPI_MISO_INDEX] = miso;
    pSPI_S(obj)->pins[SPI_SCLK_INDEX] = sclk;
    pSPI_S(obj)->pins[SPI_SSEL_INDEX] = ssel;
    /* Configure the SERCOM pins according to the user configuration */
    for (uint8_t pad = 0; pad < 4; pad++) {
        uint32_t current_pin = pSPI_S(obj)->pins[pad];
        if (current_pin != (uint32_t)NC) {
            pin_conf.mux_position = pinmap_function_sercom((PinName)current_pin, sercom_index);
            if ((uint8_t)NC != pin_conf.mux_position) {
                system_pinmux_pin_set_config(current_pin, &pin_conf);
            }
        }
    }

    /* Get baud value, based on baudrate and the internal clock frequency */
    uint32_t internal_clock = system_gclk_chan_get_hz(chan_index[sercom_index]);
    //internal_clock = 8000000;
    error_code = _sercom_get_sync_baud_val(SPI_DEFAULT_BAUD, internal_clock, &baud);
    if (error_code != STATUS_OK) {
        /* Baud rate calculation error */
        return;
    }
    _SPI(obj).SERCOM_BAUD = (uint8_t)baud;

    /* TODO: Find MUX settings */
    ctrla |= spi_find_mux_settings(obj);

    /* Set SPI character size */
    ctrlb |= SERCOM_SPIM_CTRLB_CHSIZE_8_BIT;

    /* Enable receiver */
    ctrlb |= SERCOM_SPIM_CTRLB_RXEN(1);

    /* Write CTRLA register */
    _SPI(obj).SERCOM_CTRLA |= ctrla;

    /* Write CTRLB register */
    _SPI(obj).SERCOM_CTRLB |= ctrlb;

    /* Wait until the synchronization is complete */
    while (spi_is_syncing(obj));

    /* Enable SPI */
    spi_enable(obj);
}

/** Release a SPI object
 *
 * TODO: spi_free is currently unimplemented
 * This will require reference counting at the C++ level to be safe
 *
 * Return the pins owned by the SPI object to their reset state
 * Disable the SPI peripheral
 * Disable the SPI clock
 * @param[in] obj The SPI object to deinitialize
 */
void spi_free(spi_t *obj)
{
    // [TODO]
}

/** Configure the SPI format
 *
 * Set the number of bits per frame, configure clock polarity and phase, shift order and master/slave mode
 * @param[in,out] obj   The SPI object to configure
 * @param[in]     bits  The number of bits per frame
 * @param[in]     mode  The SPI mode (clock polarity, phase, and shift direction)
 * @param[in]     slave Zero for master mode or non-zero for slave mode
 */
void spi_format(spi_t *obj, int bits, int mode, int slave)
{
    PinMode pull_mode;
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Disable SPI */
    spi_disable(obj);


    if (slave) {
        /* Set the SERCOM in SPI mode */
        _SPI(obj).SERCOM_CTRLA &= ~SERCOM_SPIS_CTRLA_MODE_Msk;
        _SPI(obj).SERCOM_CTRLA |= SERCOM_SPIS_CTRLA_MODE_SPI_SLAVE;
        pSPI_S(obj)->mode = SPI_MODE_SLAVE;
        pull_mode = PullNone;
        /* Enable PLOADEN to avoid sending dummy character by slave */
        _SPI(obj).SERCOM_CTRLB |= SERCOM_SPIS_CTRLB_PLOADEN(1);
        /* Wait until the synchronization is complete */
        while (spi_is_syncing(obj));
    } else {
        /* Set the SERCOM in SPI mode */
        _SPI(obj).SERCOM_CTRLA &= ~SERCOM_SPIM_CTRLA_MODE_Msk;
        _SPI(obj).SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_MODE_SPI_MASTER;
        pSPI_S(obj)->mode = SPI_MODE_MASTER;
        pull_mode = PullUp;
    }

    /* Change pull mode of pins */
    for (uint8_t pad = 0; pad < 4; pad++) {
        if (pSPI_S(obj)->pins[pad] != NC) {
            pin_mode(pSPI_S(obj)->pins[pad], pull_mode);
        }
    }

    /* Change MUX settings */
    uint32_t ctrla = _SPI(obj).SERCOM_CTRLA;
    ctrla &= ~(SERCOM_SPIM_CTRLA_DIPO_Msk | SERCOM_SPIM_CTRLA_DOPO_Msk);
    ctrla |= spi_find_mux_settings(obj);
    _SPI(obj).SERCOM_CTRLA = ctrla;

    /* Set SPI Frame size - only 8-bit and 9-bit supported now */
    _SPI(obj).SERCOM_CTRLB |= SERCOM_SPIM_CTRLB_CHSIZE((bits > 8)? 1 : 0);
    /* Wait until the synchronization is complete */
    while (spi_is_syncing(obj));

    /* Set SPI Clock Phase */
    _SPI(obj).SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_CPHA((mode & 0x01)? 1 : 0);

    /* Set SPI Clock Polarity */
    _SPI(obj).SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_CPOL((mode & 0x02)? 1 : 0);

    /* Enable SPI */
    spi_enable(obj);
}

/** Set the SPI baud rate
 *
 * Actual frequency may differ from the desired frequency due to available dividers and bus clock
 * Configures the SPI peripheral's baud rate
 * @param[in,out] obj The SPI object to configure
 * @param[in]     hz  The baud rate in Hz
 */
void spi_frequency(spi_t *obj, int hz)
{
    uint16_t baud = 0;
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Disable SPI */
    spi_disable(obj);

    /* Find frequency of the internal SERCOMi_GCLK_ID_CORE */
    uint32_t sercom_index = _sercom_get_sercom_inst_index(pSPI_SERCOM(obj));
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
    uint32_t internal_clock = system_gclk_chan_get_hz(chan_index[sercom_index]);

    /* Get baud value, based on baudrate and the internal clock frequency */
    enum status_code error_code = _sercom_get_sync_baud_val(hz, internal_clock, &baud);

    if (error_code != STATUS_OK) {
        /* Baud rate calculation error, return status code */
        /* Enable SPI */
        spi_enable(obj);
        return;
    }

    _SPI(obj).SERCOM_BAUD = (uint8_t)baud;

    /* Enable SPI */
    spi_enable(obj);
}

/**@}*/
/**
 * \defgroup SynchSPI Synchronous SPI Hardware Abstraction Layer
 * @{
 */

/** Write a byte out in master mode and receive a value
 *
 * @param[in] obj   The SPI peripheral to use for sending
 * @param[in] value The value to send
 * @return Returns the value received during send
 */
int spi_master_write(spi_t *obj, int value)
{
    uint16_t rx_data = 0;

    /* Sanity check arguments */
    MBED_ASSERT(obj);

#if DEVICE_SPI_ASYNCH
    if (obj->spi.status == STATUS_BUSY) {
        /* Check if the SPI module is busy with a job */
        return 0;
    }
#endif

    /* Wait until the module is ready to write the character */
    while (!spi_is_ready_to_write(obj));

    /* Write data */
    spi_write(obj, value);

    if (!(_SPI(obj).SERCOM_CTRLB & SERCOM_SPIM_CTRLB_RXEN(1))) {
        return 0;
    }

    /* Wait until the module is ready to read the character */
    while (!spi_is_ready_to_read(obj));

    /* Read data */
    spi_read(obj, &rx_data);

    return rx_data;
}

int spi_master_block_write(spi_t *obj, const char *tx_buffer, int tx_length,
                           char *rx_buffer, int rx_length, char write_fill) {
    int total = (tx_length > rx_length) ? tx_length : rx_length;

    for (int i = 0; i < total; i++) {
        char out = (i < tx_length) ? tx_buffer[i] : write_fill;
        char in = spi_master_write(obj, out);
        if (i < rx_length) {
            rx_buffer[i] = in;
        }
    }

    return total;
}

/** Check if a value is available to read
 *
 * @param[in] obj The SPI peripheral to check
 * @return non-zero if a value is available
 */
int spi_slave_receive(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    return spi_is_ready_to_read(obj);
}

/** Get a received value out of the SPI receive buffer in slave mode
 *
 * Blocks until a value is available
 * @param[in] obj The SPI peripheral to read
 * @return The value received
 */
int spi_slave_read(spi_t *obj)
{
    int i;
    uint16_t rx_data = 0;

    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Check for timeout period */
    for (i = 0; i < SPI_TIMEOUT; i++) {
        if (spi_is_ready_to_read(obj)) {
            break;
        }
    }
    if (i == SPI_TIMEOUT) {
        /* Not ready to read data within timeout period */
        return 0;
    }

    /* Read data */
    spi_read(obj, &rx_data);

    return rx_data;
}

/** Write a value to the SPI peripheral in slave mode
 *
 * Blocks until the SPI peripheral can be written to
 * @param[in] obj   The SPI peripheral to write
 * @param[in] value The value to write
 */
void spi_slave_write(spi_t *obj, int value)
{
    int i;

    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Check for timeout period */
    for (i = 0; i < SPI_TIMEOUT; i++) {
        if (spi_is_ready_to_write(obj)) {
            break;
        }
    }
    if (i == SPI_TIMEOUT) {
        /* Not ready to write data within timeout period */
        return;
    }

    /* Write data */
    spi_write(obj, value);
}

/** Checks if the specified SPI peripheral is in use
 *
 * @param[in] obj The SPI peripheral to check
 * @return non-zero if the peripheral is currently transmitting
 */
int spi_busy(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    return spi_is_write_complete(obj);
}

/** Get the module number
 *
 * @param[in] obj The SPI peripheral to check
 * @return The module number
 */
uint8_t spi_get_module(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    return _sercom_get_sercom_inst_index(pSPI_SERCOM(obj));
}

const PinMap *spi_master_mosi_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *spi_master_miso_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *spi_master_clk_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *spi_master_cs_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *spi_slave_mosi_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *spi_slave_miso_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *spi_slave_clk_pinmap()
{
    return PinMap_SERCOM;
}

const PinMap *spi_slave_cs_pinmap()
{
    return PinMap_SERCOM;
}


#if DEVICE_SPI_ASYNCH
/**
 * \defgroup AsynchSPI Asynchronous SPI Hardware Abstraction Layer
 * @{
 */


/**
 * \internal
 * Writes a character from the TX buffer to the Data register.
 *
 * \param[in,out]  module  Pointer to SPI software instance struct
 */
static void _spi_write_async(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    uint16_t data_to_send;
    uint8_t *tx_buffer = obj->tx_buff.buffer;

    /* Do nothing if we are at the end of buffer */
    if (obj->tx_buff.pos < obj->tx_buff.length) {
        /* Write value will be at least 8-bits long */
        if (tx_buffer) {
            data_to_send = tx_buffer[obj->tx_buff.pos];
        } else {
            data_to_send = dummy_fill_word;
        }
        /* Increment 8-bit index */
        obj->tx_buff.pos++;

        if (_SPI(obj).SERCOM_CTRLB & SERCOM_SPIS_CTRLB_CHSIZE_9_BIT) {
            if (tx_buffer)
                data_to_send |= (tx_buffer[obj->tx_buff.pos] << 8);
            /* Increment 8-bit index */
            obj->tx_buff.pos++;
        }
    } else {
        /* Write a dummy packet */
        /* TODO: Current implementation do not enter this condition, remove if not needed */
        data_to_send = dummy_fill_word;
    }

    /* Write the data to send*/
    _SPI(obj).SERCOM_DATA = data_to_send & SERCOM_SPIM_DATA_Msk;

    /* Check for error */
    if ((_SPI(obj).SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_ERROR(1)) && (obj->spi.mask & SPI_EVENT_ERROR)) {
        obj->spi.event |= SPI_EVENT_ERROR;
    }
}

/**
 * \internal
 * Reads a character from the Data register to the RX buffer.
 *
 * \param[in,out]  module  Pointer to SPI software instance struct
 */
static void _spi_read_async(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    uint8_t *rx_buffer = obj->rx_buff.buffer;

    /* Check if data is overflown */
    if (_SPI(obj).SERCOM_STATUS & SERCOM_SPIM_STATUS_BUFOVF(1)) {
        /* Clear overflow flag */
        _SPI(obj).SERCOM_STATUS |= SERCOM_SPIM_STATUS_BUFOVF(1);
        if (obj->spi.mask & SPI_EVENT_RX_OVERFLOW) {
            /* Set overflow error */
            obj->spi.event |= SPI_EVENT_RX_OVERFLOW;
            return;
        }
    }

    /* Read data, either valid, or dummy */
    uint16_t received_data = (_SPI(obj).SERCOM_DATA & SERCOM_SPIM_DATA_Msk);

    /* Do nothing if we are at the end of buffer */
    if ((obj->rx_buff.pos >= obj->rx_buff.length) && rx_buffer) {
        return;
    }

    /* Read value will be at least 8-bits long */
    rx_buffer[obj->rx_buff.pos] = received_data;
    /* Increment 8-bit index */
    obj->rx_buff.pos++;

    if (_SPI(obj).SERCOM_CTRLB & SERCOM_SPIS_CTRLB_CHSIZE_9_BIT) {
        /* 9-bit data, write next received byte to the buffer */
        rx_buffer[obj->rx_buff.pos] = (received_data >> 8);
        /* Increment 8-bit index */
        obj->rx_buff.pos++;
    }

    /* Check for error */
    if ((_SPI(obj).SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_ERROR(1)) && (obj->spi.mask & SPI_EVENT_ERROR)) {
        obj->spi.event |= SPI_EVENT_ERROR;
    }
}

static void (*irq_handler[SERCOM_INST_NUM])(void);

void spi_irq(const uint8_t instance, const uint8_t offset)
{
    irq_handler[instance]();
}

static IRQn_Type get_serial_irq_num (const uint8_t sercom_index)
{
    IRQn_Type irq_n_0[] = {
        SERCOM0_0_IRQn,
        SERCOM1_0_IRQn,
        SERCOM2_0_IRQn,
        SERCOM3_0_IRQn,
        SERCOM4_0_IRQn,
        SERCOM5_0_IRQn,
        SERCOM6_0_IRQn,
        SERCOM7_0_IRQn,
    };
    return irq_n_0[sercom_index];
}

/**
 * \internal
 * Clears all interrupt flags of SPI
 *
 * \param[in,out]  module  Pointer to SPI software instance struct
 */
static void _spi_clear_interrupts(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    uint8_t sercom_index = _sercom_get_sercom_inst_index(obj->spi.spi);
    IRQn_Type irq_n_0 = get_serial_irq_num(sercom_index);

    /* Clear all interrupts */
    _SPI(obj).SERCOM_INTENCLR =
        SERCOM_SPIM_INTFLAG_DRE(1) |
        SERCOM_SPIM_INTFLAG_TXC(1) |
        SERCOM_SPIM_INTFLAG_RXC(1) |
        SERCOM_SPIM_INTFLAG_ERROR(1);

    for (IRQn_Type i = 0; i < 4; i++) {
        NVIC_DisableIRQ(irq_n_0 + i);
    }
}

/**
 * \internal
 * Starts transceive of buffers with a given length
 *
 * \param[in,out]  obj   Pointer to SPI software instance struct
 *
 */
static enum status_code _spi_transceive_buffer(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    uint16_t interrupt_status = _SPI(obj).SERCOM_INTFLAG;
    interrupt_status &= _SPI(obj).SERCOM_INTENSET;

    if (interrupt_status & SERCOM_SPIM_INTFLAG_DRE(1)) {
        /* Clear DRE interrupt */
        _SPI(obj).SERCOM_INTENCLR = SERCOM_SPIM_INTFLAG_DRE(1);
        /* Write data */
        _spi_write_async(obj);
        /* Set TXC interrupt */
        _SPI(obj).SERCOM_INTENSET |= SERCOM_SPIM_INTFLAG_TXC(1);
    }
    if (interrupt_status & SERCOM_SPIM_INTFLAG_TXC(1)) {
        /* Clear TXC interrupt */
        _SPI(obj).SERCOM_INTENCLR = SERCOM_SPIM_INTFLAG_TXC(1);
        if ((obj->rx_buff.buffer) && (obj->rx_buff.pos < obj->rx_buff.length)) {
            while (!spi_is_ready_to_read(obj));
            _spi_read_async(obj);
            if ((obj->tx_buff.pos >= obj->tx_buff.length) && (obj->tx_buff.length < obj->rx_buff.length)) {
                obj->tx_buff.length = obj->rx_buff.length;
                obj->tx_buff.buffer = 0;
            }
        }
        if (obj->tx_buff.pos < obj->tx_buff.length) {
            /* Set DRE interrupt */
            _SPI(obj).SERCOM_INTENSET |= SERCOM_SPIM_INTFLAG_DRE(1);
        }
    }

    if (obj->spi.event & (SPI_EVENT_ERROR | SPI_EVENT_RX_OVERFLOW) || (interrupt_status & SERCOM_SPIM_INTFLAG_ERROR(1))) {
        /* Clear all interrupts */
        _spi_clear_interrupts(obj);

        if (interrupt_status & SERCOM_SPIM_INTFLAG_ERROR(1)) {
            obj->spi.event = STATUS_ERR_BAD_DATA;
        }

        /* Transfer interrupted, invoke the callback function */
        if (obj->spi.event & SPI_EVENT_RX_OVERFLOW) {
            obj->spi.status = STATUS_ERR_OVERFLOW;
        } else {
            obj->spi.status = STATUS_ERR_BAD_DATA;
        }
        return (enum status_code)obj->spi.status;
    }

    if ((obj->tx_buff.pos >= obj->tx_buff.length) && (obj->rx_buff.pos >= obj->rx_buff.length) && (interrupt_status & SERCOM_SPIM_INTFLAG_TXC(1))) {
        /* Clear all interrupts */
        _spi_clear_interrupts(obj);

        /* Transfer complete, invoke the callback function */
        obj->spi.event = SPI_EVENT_INTERNAL_TRANSFER_COMPLETE;
        obj->spi.status = STATUS_OK;
    }

    return (enum status_code)(obj->spi.status);
}

/** Begin the SPI transfer. Buffer pointers and lengths are specified in tx_buff and rx_buff
 *
 * @param[in] obj       The SPI object which holds the transfer information
 * @param[in] tx        The buffer to send
 * @param[in] tx_length The number of words to transmit
 * @param[out]rx        The buffer to receive
 * @param[in] rx_length The number of words to receive
 * @param[in] bit_width The bit width of buffer words
 * @param[in] event     The logical OR of events to be registered
 * @param[in] handler   SPI interrupt handler
 * @param[in] hint      A suggestion for how to use DMA with this transfer **< DMA currently not implemented >**
 */
void spi_master_transfer(spi_t *obj, const void *tx, size_t tx_length, void *rx, size_t rx_length, uint8_t bit_width, uint32_t handler, uint32_t event, DMAUsage hint)
{
    uint16_t dummy_read;
    (void) dummy_read;
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    uint8_t sercom_index = _sercom_get_sercom_inst_index(obj->spi.spi);
    IRQn_Type irq_n_0 = get_serial_irq_num(sercom_index);

    obj->spi.tx_buffer = (void *)tx;
    obj->tx_buff.buffer =(void *)tx;
    obj->tx_buff.pos = 0;
    if (tx) {
        /* Only two bit rates supported now */
        obj->tx_buff.length = tx_length * ((bit_width > 8)? 2 : 1);
    } else {
        if (rx) {
            obj->tx_buff.length = rx_length * ((bit_width > 8)? 2 : 1);
        } else {
            /* Nothing to transfer */
            return;
        }
    }

    obj->spi.rx_buffer = rx;
    obj->rx_buff.buffer = rx;
    obj->rx_buff.pos = 0;
    if (rx) {
        /* Only two bit rates supported now */
        obj->rx_buff.length = rx_length * ((bit_width > 8)? 2 : 1);
    } else {
        /* Disable RXEN */
        spi_disable(obj);
        _SPI(obj).SERCOM_CTRLB &= ~SERCOM_SPIM_CTRLB_RXEN(1);
        /* Wait until the synchronization is complete */
        while (spi_is_syncing(obj));
        spi_enable(obj);
        obj->rx_buff.length = 0;
    }

    /* Clear data buffer if there is anything pending to read */
    while (spi_is_ready_to_read(obj)) {
        dummy_read = _SPI(obj).SERCOM_DATA;
    }

    obj->spi.mask = event;

    obj->spi.dma_usage = hint;

    /*if (hint == DMA_USAGE_NEVER) {** TEMP: Commented as DMA is not implemented now */
    /* Use irq method */
    uint16_t irq_mask = 0;
    obj->spi.status = STATUS_BUSY;

    /* Enable interrupt */
    irq_handler[sercom_index] = (void (*)())handler;
    _sercom_instances[sercom_index] = obj;
    _sercom_set_handler(sercom_index, spi_irq);

    NVIC_EnableIRQ(irq_n_0 + 1); /* TXC */
    NVIC_EnableIRQ(irq_n_0 + 3); /* OTHER */

    /* Clear all interrupts */
    _SPI(obj).SERCOM_INTENCLR = SERCOM_SPIM_INTFLAG_TXC(1) | SERCOM_SPIM_INTFLAG_RXC(1) | SERCOM_SPIM_INTFLAG_ERROR(1);
    _SPI(obj).SERCOM_INTFLAG =  SERCOM_SPIM_INTFLAG_TXC(1) | SERCOM_SPIM_INTFLAG_ERROR(1);
    _SPI(obj).SERCOM_STATUS |=  SERCOM_SPIM_STATUS_BUFOVF(1);

    /* Set SPI interrupts */
    /* Set DRE flag to kick start transmission */
    irq_mask |= SERCOM_SPIM_INTFLAG_DRE(1);

    if (event & SPI_EVENT_ERROR) {
        irq_mask |= SERCOM_SPIM_INTFLAG_ERROR(1);
    }
    _SPI(obj).SERCOM_INTENSET = irq_mask;
    /*} ** TEMP: Commented as DMA is not implemented now */
}

/** The asynchronous IRQ handler
 *
 * Reads the received values out of the RX FIFO, writes values into the TX FIFO and checks for transfer termination
 * conditions, such as buffer overflows or transfer complete.
 * @param[in] obj     The SPI object which holds the transfer information
 * @return event flags if a transfer termination condition was met or 0 otherwise.
 */
uint32_t spi_irq_handler_asynch(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);
    enum status_code tmp_status;

    uint32_t transfer_event = 0;

    /*if (obj->spi.dma_usage == DMA_USAGE_NEVER) {** TEMP: Commented as DMA is not implemented now */
    /* IRQ method */
    tmp_status = _spi_transceive_buffer(obj);
    if (STATUS_BUSY != tmp_status) {
        if ((obj->spi.event & SPI_EVENT_INTERNAL_TRANSFER_COMPLETE) && (tmp_status == STATUS_OK)) {
            obj->spi.event |= SPI_EVENT_COMPLETE;
        }
        transfer_event = obj->spi.event & (obj->spi.mask | SPI_EVENT_INTERNAL_TRANSFER_COMPLETE);
    }
    /*}** TEMP: Commented as DMA is not implemented now */
    return transfer_event;
}

/** Attempts to determine if the SPI peripheral is already in use.
 * @param[in] obj The SPI object to check for activity
 * @return non-zero if the SPI port is active or zero if it is not.
 */
uint8_t spi_active(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    /* Check if the SPI module is busy with a job */
    return (obj->spi.status == STATUS_BUSY);
}

/** Abort an SPI transfer
 *
 * @param obj The SPI peripheral to stop
 */
void spi_abort_asynch(spi_t *obj)
{
    /* Sanity check arguments */
    MBED_ASSERT(obj);

    uint8_t sercom_index = _sercom_get_sercom_inst_index(obj->spi.spi);
    IRQn_Type irq_n_0 = get_serial_irq_num(sercom_index);

    /* Clear all interrupts */
    _SPI(obj).SERCOM_INTENCLR =
        SERCOM_SPIM_INTFLAG_DRE(1) |
        SERCOM_SPIM_INTFLAG_TXC(1) |
        SERCOM_SPIM_INTFLAG_RXC(1) |
        SERCOM_SPIM_INTFLAG_ERROR(1);

    // TODO: Disable and remove irq handler
    for (IRQn_Type i = 0; i < 4; i++) {
        NVIC_DisableIRQ(irq_n_0 + i);
    }

    obj->spi.status = STATUS_ABORTED;
}

#endif /* DEVICE_SPI_ASYNCH */

#endif
