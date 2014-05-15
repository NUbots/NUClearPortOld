/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "UART.h"

#include <sys/ioctl.h>
#include <fcntl.h>

namespace Darwin {
    uint8_t calculateChecksum(void* command) {

        uint8_t* data = static_cast<uint8_t*>(command);
        uint8_t checksum = 0x00;
        // Skip over the magic numbers and checksum the rest of the packet
        for (int i = 2; i < data[Packet::LENGTH] + 3; ++i) {
            checksum += data[i];
        }
        return (~checksum);
    }

    uint8_t calculateChecksum(const CommandResult& result) {

        uint8_t checksum = 0x00;

        checksum += result.header.id;
        checksum += result.header.length;
        checksum += result.header.errorcode;

        for (size_t i = 0; i < result.data.size(); ++i) {
            checksum += result.data[i];
        }

        return (~checksum);
    }

    UART::UART(const char* name) {

        double baud = 1000000;  // (1mb/s)

        fd = open(name, O_RDWR|O_NOCTTY|O_NONBLOCK);

        // If we have a valid file handle, and were able to configure it correctly (custom baud)
        if (fd < 0 || !configure(baud)) {
            // There was an exception connecting
            throw std::runtime_error("There was an error setting up the serial connection to the CM730");
        }
    }

    bool UART::configure(double baud) {

        // Do our setup for the tio settings, you must set BS38400 in order to set custom baud using "baud rate aliasing"
        // http://stackoverflow.com/questions/4968529/how-to-set-baud-rate-to-307200-on-linux
        termios tio;
        memset(&tio, 0, sizeof(tio));
        // B38400 for alising, CS8 (8bit,no parity,1 stopbit), CLOCAL (local connection, no modem contol), CREAD (enable receiving characters)
        tio.c_cflag      = B38400|CS8|CLOCAL|CREAD;
        // IGNPAR (ignore incoming parity bits as we don't have parity)
        tio.c_iflag      = IGNPAR;
        // 0 means raw output
        tio.c_oflag      = 0;
        // No ICANON so we read immediantly rather then line by line
        tio.c_lflag     &= ~ICANON;
        tio.c_cc[VTIME]  = 0;
        tio.c_cc[VMIN]   = 0;
        // Set the settings
        tcsetattr(fd, TCSANOW, &tio);

        // Here we do the baud rate aliasing in order to set the custom baud rate
        serial_struct serinfo;

        // Get our serial_info from the system
        if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
            return false;
        }

        // Set the speed flags to "Custom Speed" (clear the existing speed, and set the custom speed flags)
        serinfo.flags &= ~ASYNC_SPD_MASK;
        serinfo.flags |= ASYNC_SPD_CUST;

        // Set our custom divsor for our speed
        serinfo.custom_divisor = serinfo.baud_base / baud;

        // Set our custom speed in the system
        if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
            return false;
        }

        // Flush our connection to remove all existing data
        tcflush(fd, TCIFLUSH);

        return true;
    }

    CommandResult UART::readPacket() {

        // We will wait this long for an initial packet header
        int PACKET_WAIT = 20000;
        // We will only wait a maximum of 1000 microseconds between bytes in a packet (assumes baud of 1000000bps)
        int BYTE_WAIT = 1000;

        // Our result
        CommandResult result;

        // Clear our connection set and put in our serial device
        fd_set connectionset;
        timeval timeout;
        timeout.tv_sec = 0;
        FD_ZERO(&connectionset);
        FD_SET(fd, &connectionset);

        // First we find the packet magic number in order to sync with the channel
        timeout.tv_usec = PACKET_WAIT;
        for (int sync = 0; sync < 2;) {
            if (select(fd + 1, &connectionset, nullptr, nullptr, &timeout) == 1) {

                uint8_t byte;
                int bytesRead = read(fd, &byte, 1);

                assert(bytesRead == 1);
                (void) bytesRead; // Make the compiler happy when NDEBUG is set

                sync = byte == 0xFF ? sync + 1 : 0;
            }
            else {
                // The result is pre initialized as a timeout
                return result;
            }
        }

        // We now are now waiting for 4 bytes
        timeout.tv_usec = BYTE_WAIT * sizeof(Header);
        uint8_t* headerBytes = reinterpret_cast<uint8_t*>(&result.header);
        for (size_t done = 0; done < sizeof(Header);) {
            if (select(fd + 1, &connectionset, nullptr, nullptr, &timeout) == 1) {

                done += read(fd, &headerBytes[done], sizeof(Header) - done);
            }
            else {
                // The result is pre initialized as a timeout
                return result;
            }
        }

        // Here we adjust our "length" to mean the length of the payload rather then the length of bytes after the length
        int length = result.header.length - 2;

        // We now are now waiting for our data
        timeout.tv_usec = BYTE_WAIT * length;
        result.data.resize(length);
        for (int done = 0; done < length;) {
            if (select(fd + 1, &connectionset, nullptr, nullptr, &timeout) == 1) {

                done += read(fd, &result.data[done], length - done);
            }
            else {
                // Set our packet header to timeout and return it
                result.header.errorcode = ErrorCode::NO_RESPONSE;
                return result;
            }
        }

        // We just read the checksum now
        timeout.tv_usec = 2000;
        if (select(fd + 1, &connectionset, nullptr, nullptr, &timeout) == 1) {

            size_t bytesRead = read(fd, &result.checksum, 1);
            assert(bytesRead == 1);
            (void)bytesRead; // Make the compiler happy when NDEBUG is set
        }
        else {
            // If all we are missing is the checksum, just assume the data is corrupt
            result.header.errorcode |= ErrorCode::CORRUPT_DATA;
            return result;
        }

        // Validate our checksum
        if (result.checksum != calculateChecksum(result)) {
            CommandResult result;
            result.checksum = 0; // GCC doesn't like that this isn't initalized
            result.header.errorcode |= ErrorCode::CORRUPT_DATA;
            return result;
        }

        // Return the packet we recieved
        return result;
    }

    std::vector<CommandResult> UART::executeBulk(const std::vector<uint8_t>& command) {

        // We can work out how many responses to expect based on our packets length
        int responses = (command[Packet::LENGTH]-3) / 3;
        std::vector<CommandResult> results(responses);

        // Lock our mutex
        std::lock_guard<std::mutex> lock(mutex);

        // We flush our buffer, just in case there was anything random in it
        tcflush(fd, TCIFLUSH);

        // Write the command as usual
        size_t written = write(fd, command.data(), command.size());
        assert(written == command.size());
        (void)written; // Keep the compiler happy when NDEBUG is set

        // Read our responses for each of the packets
        for (int i = 0; i < responses; ++i) {
            results[i] = readPacket();

            // If we get a timeout don't wait for other packets (other errors are fine)
            if(results[i].header.errorcode == ErrorCode::NO_RESPONSE) break;
        }

        return results;
    }

    void UART::executeBroadcast(const std::vector<uint8_t>& command) {

        // Lock our mutex
        std::lock_guard<std::mutex> lock(mutex);

        // We flush our buffer, just in case there was anything random in it
        tcflush(fd,TCIFLUSH);

        // Write the command as usual
        size_t written = write(fd, command.data(), command.size());
        assert(written == command.size());
        (void) written; // Make the compiler happy when NDEBUG is set

        // There are no responses for broadcast commands
    }
}