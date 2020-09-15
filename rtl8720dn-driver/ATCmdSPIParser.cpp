/* Copyright (c) 2017 ARM Limited
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
 *
 * @section DESCRIPTION
 *
 * Parser for the AT command syntax
 *
 */

#include "ATCmdSPIParser.h"
#include "mbed_poll.h"
#include "mbed_debug.h"
#include "rtos/ThisThread.h"
#include "platform/mbed_chrono.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef LF
#undef LF
#define LF  10
#else
#define LF  10
#endif

#ifdef CR
#undef CR
#define CR  13
#else
#define CR  13
#endif

#define SPT_TAG_PRE (0x55)
#define SPT_TAG_WR  (0x80)
#define SPT_TAG_RD  (0x00)
#define SPT_TAG_DMY (0xff)

using namespace mbed;
using namespace std::chrono;
using std::milli;

ATCmdSPIParser::ATCmdSPIParser(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName sync, PinName enable, const char *output_delimiter,
    int buffer_size, int timeout, bool debug)
    : _spi(mosi, miso, sclk),
      _spi_cs(cs),
      _spi_sync(sync),
      _rtl8720_enable(enable),
      _buffer_size(buffer_size),
      _oob_cb_count(0),
      _in_prev(0),
      _aborted(false),
      _oobs(NULL)
{
    _buffer = new char[buffer_size];
    set_timeout(timeout);
    set_delimiter(output_delimiter);
    debug_on(debug);
}

static bool __initialized = false;
void ATCmdSPIParser::__init()
{
    if (__initialized == true)
        return;
    __initialized = true;

    // RTL8720DN Initialize
    _rtl8720_enable = 0;
    _spi.format(8, 0);
    _spi.frequency();
    _spi_sync.mode(PullUp);
    //_spi_irq.mode(PullDown);
    _spi_cs = 1;
    rtos::ThisThread::sleep_for(20ms);
    _rtl8720_enable = 1;
    rtos::ThisThread::sleep_for(500ms);
    char buf[10];
    read(buf, 10);
    buf[9] = 0;
}

void ATCmdSPIParser::set_chip_select(bool high)
{
    if (high == true) {
        _spi_cs = 1;
    } else {
        _spi_cs = 0;
    }
}

void ATCmdSPIParser::wait_for_mosi_direction()
{
    while (_spi_sync == 0) {
        rtos::ThisThread::sleep_for(1ms);
    }
}

void ATCmdSPIParser::wait_for_miso_direction()
{
    while (_spi_sync == 1) {
        rtos::ThisThread::sleep_for(1ms);
    }
}

// getc/putc handling with timeouts
int ATCmdSPIParser::putc(char c)
{
    _spi.write((int)c);
    return 0;
}

int ATCmdSPIParser::getc()
{
    return _spi.write(SPT_TAG_DMY);
}

void ATCmdSPIParser::flush()
{
}

// read/write handling with timeouts
int ATCmdSPIParser::write(const char *data, int size)
{
    wait_for_mosi_direction();

    set_chip_select(false);
    putc(SPT_TAG_PRE);
    putc(SPT_TAG_WR);
    putc((size >> 8) & 0xff);
    putc(size & 0xff);
    set_chip_select(true);

    wait_for_miso_direction();

    set_chip_select(false);
    uint8_t v;
    v = getc();
    if (v != 0xbe) {
        debug_if(_dbg_on, "ACK = %02x\n", v);
    }
    v = getc();
    if (v != 0x00) {
        debug_if(_dbg_on, "STATUS = %02x\n", v);
    }
    uint16_t length;
    length = getc() << 8;
    length |= getc();
    if (size != length) {
        debug_if(_dbg_on, "Length is not match: %d, %d\n", size, length);
    }
    set_chip_select(true);

    wait_for_mosi_direction();

    set_chip_select(false);
    int i = 0;
    for (; i < size; i++) {
        if (putc(data[i]) < 0) {
            return -1;
        }
    }
    set_chip_select(true);

    if (size) {
        write(NULL, 0);
    }

    return i;
}

int ATCmdSPIParser::raw_read(char *data, int size)
{
    wait_for_mosi_direction();

    set_chip_select(false);
    putc(SPT_TAG_PRE);
    putc(SPT_TAG_RD);
    putc((size >> 8) & 0xff);
    putc(size & 0xff);
    set_chip_select(true);

    wait_for_miso_direction();

    set_chip_select(false);
    uint8_t v;
    v = getc();
    if (v != 0xbe) {
        debug_if(_dbg_on, "ACK = %02x\n", v);
    }
    v = getc();
    if (v != 0x00) {
        debug_if(_dbg_on, "STATUS = %02x\n", v);
    }
    uint16_t length;
    length = getc() << 8;
    length |= getc();

    int i = 0;
    for (; i < length; i++) {
        int c = getc();
        if (c < 0) {
            return -1;
        }
        data[i] = c;
    }
    set_chip_select(true);

    return i;
}

int ATCmdSPIParser::read(char *data, int size)
{
    for (int i = 0; i < _timeout; i++) {
        int ret = raw_read(data, size);
        if (ret) {
            return ret;
        }
        rtos::ThisThread::sleep_for(1ms);
    }
    return -1; //Timeout
}

// printf/scanf handling
int ATCmdSPIParser::vprintf(const char *format, std::va_list args)
{
    if (vsprintf(_buffer, format, args) < 0) {
        return false;
    }

    int i = 0;
    for (; _buffer[i]; i++) {
        if (putc(_buffer[i]) < 0) {
            return -1;
        }
    }
    return i;
}

// Command parsing with line handling
bool ATCmdSPIParser::vsend(const char *command, std::va_list args)
{
    __init();

    // Create and send command
    if (vsprintf(_buffer, command, args) < 0) {
        return false;
    }

    uint16_t length = strlen(_buffer);
    for (int i = 0; _output_delimiter[i]; i++) {
        _buffer[length++] = _output_delimiter[i];
    }
    //_buffer[length] = '\0';

    if (write(_buffer, length) < 0) {
        return false;
    }

    debug_if(_dbg_on, "AT> %s\n", _buffer);
    return true;

}

int ATCmdSPIParser::vrecvscanf(const char *response, std::va_list args, bool multiline)
{
    bool oob_received = false;

restart:
    _aborted = false;
    // Iterate through each line in the expected response
    // response being NULL means we just want to check for OOBs
    while (!response || response[0]) {
        // Since response is const, we need to copy it into our buffer to
        // add the line's null terminator and clobber value-matches with asterisks.
        //
        // We just use the beginning of the buffer to avoid unnecessary allocations.
        int i = 0;
        int offset = 0;
        bool whole_line_wanted = false;

        while (response && response[i]) {
            if (response[i] == '%' && response[i + 1] != '%' && response[i + 1] != '*') {
                if ((offset + 2) > _buffer_size) {
                    return -1;
                }
                _buffer[offset++] = '%';
                _buffer[offset++] = '*';
                i++;
            } else {
                if ((offset + 1) > _buffer_size) {
                    return -1;
                }
                _buffer[offset++] = response[i++];
                // Find linebreaks, taking care not to be fooled if they're in a %[^\n] conversion specification
                if (response[i - 1] == '\n' && !(i >= 3 && response[i - 3] == '[' && response[i - 2] == '^')) {
                    whole_line_wanted = true;
                    break;
                }
            }
        }

        // Scanf has very poor support for catching errors
        // fortunately, we can abuse the %n specifier to determine
        // if the entire string was matched.
        if ((offset + 3) > _buffer_size) {
            return -1;
        }
        _buffer[offset++] = '%';
        _buffer[offset++] = 'n';
        _buffer[offset++] = 0;

        debug_if(_dbg_on, "AT? %s\n", _buffer);
        // To workaround scanf's lack of error reporting, we actually
        // make two passes. One checks the validity with the modified
        // format string that only stores the matched characters (%n).
        // The other reads in the actual matched values.
        //
        // We keep trying the match until we succeed or some other error
        // derails us.
        int j = 0;

        while (true) {
            // Ran out of space
            if (j + 1 >= _buffer_size - offset) {
                return -1;
            }

#if 0
            // If just peeking for OOBs, and at start of line, check
            // readability
            if (!response && j == 0 && !_fh->readable()) {
                return -1;
            }
#endif

            if (!response && j == 0 && oob_received == true) {
                return -1;
            }

            // Receive next character
            char ch;
            if (read(&ch, 1) < 0) {
                debug_if(_dbg_on, "AT(Timeout)\n");
                return -1;
            }
            int c = (int)ch;

            // Simplify newlines (borrowed from retarget.cpp)
            if ((c == CR && _in_prev != LF) ||
                    (c == LF && _in_prev != CR)) {
                _in_prev = c;
                c = '\n';
            } else if ((c == CR && _in_prev == LF) ||
                       (c == LF && _in_prev == CR)) {
                _in_prev = c;
                // onto next character
                continue;
            } else {
                _in_prev = c;
            }

            if ((offset + j + 1) > _buffer_size) {
                return -1;
            }
            _buffer[offset + j++] = c;
            _buffer[offset + j] = 0;

            // Check for oob data
            if (multiline) {
                for (struct oob *oob = _oobs; oob; oob = oob->next) {
                    if ((unsigned)j == oob->len && memcmp(
                                oob->prefix, _buffer + offset, oob->len) == 0) {
                        debug_if(_dbg_on, "AT! %s\n", oob->prefix);
                        _oob_cb_count++;
                        oob->cb();

                        if (_aborted) {
                            debug_if(_dbg_on, "AT(Aborted)\n");
                            return false;
                        }
                        // oob may have corrupted non-reentrant buffer,
                        // so we need to set it up again
                        oob_received = true;
                        goto restart;
                    }
                }
            }

            // Check for match
            int count = -1;
            if (whole_line_wanted && c != '\n') {
                // Don't attempt scanning until we get delimiter if they included it in format
                // This allows recv("Foo: %s\n") to work, and not match with just the first character of a string
            } else if (response) {
                sscanf(_buffer + offset, _buffer, &count);
                //debug_if(_dbg_on, "count = %d\n", count);
            }

            // We only succeed if all characters in the response are matched
            if (count == j) {
                debug_if(_dbg_on, "AT= %s\n", _buffer + offset);
                // Reuse the front end of the buffer
                memcpy(_buffer, response, i);
                _buffer[i] = 0;

                // Store the found results
                vsscanf(_buffer + offset, _buffer, args);

                if (!multiline) {
                    return j;
                }

                // Jump to next line and continue parsing
                response += i;
                break;
            }

            // Clear the buffer when we hit a newline or ran out of space
            // running out of space usually means we ran into binary data
            if (c == '\n' || j + 1 >= _buffer_size - offset) {
                debug_if(_dbg_on, "AT< %s", _buffer + offset);
                j = 0;
            }
        }
    }

    return 1;
}

int ATCmdSPIParser::vscanf(const char *format, std::va_list args)
{
    return vrecvscanf(format, args, false);
}

bool ATCmdSPIParser::vrecv(const char *response, std::va_list args)
{
    return (vrecvscanf(response, args, true)) > 0 ? true : false;
}

// Mapping to vararg functions
int ATCmdSPIParser::printf(const char *format, ...)
{
    std::va_list args;
    va_start(args, format);
    int res = vprintf(format, args);
    va_end(args);
    return res;
}

int ATCmdSPIParser::scanf(const char *format, ...)
{
    std::va_list args;
    va_start(args, format);
    int res = vrecvscanf(format, args, false);
    va_end(args);
    return res;
}

bool ATCmdSPIParser::send(const char *command, ...)
{
    std::va_list args;
    va_start(args, command);
    bool res = vsend(command, args);
    va_end(args);
    return res;
}

bool ATCmdSPIParser::recv(const char *response, ...)
{
    std::va_list args;
    va_start(args, response);
    int res = vrecvscanf(response, args, true);
    va_end(args);
    return (res > 0) ? true : false;
}

// oob registration
void ATCmdSPIParser::oob(const char *prefix, Callback<void()> cb)
{
    struct oob *oob = new struct oob;
    oob->len = strlen(prefix);
    oob->prefix = prefix;
    oob->cb = cb;
    oob->next = _oobs;
    _oobs = oob;
}

void ATCmdSPIParser::remove_oob(const char *prefix)
{
    struct oob *prev = NULL;
    struct oob *oob = _oobs;
    while (oob) {
        if (memcmp(oob->prefix, prefix, strlen(prefix)) == 0) {
            if (prev) {
                prev->next = oob->next;
            } else {
                _oobs = oob->next;
            }
            delete oob;
            return;
        }
        prev = oob;
        oob = oob->next;
    }
}

void ATCmdSPIParser::abort()
{
    _aborted = true;
}

bool ATCmdSPIParser::process_oob()
{
    int pre_count = _oob_cb_count;
    static_cast<void>(recv(NULL));
    return _oob_cb_count != pre_count;
}
