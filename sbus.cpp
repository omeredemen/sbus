#include "sbus.h"

namespace sbus_bridge{

    bool SBusSerialPort::connectSerialPort() {
        // Open serial port
        // O_RDWR - Read and write
        // O_NOCTTY - Ignore special chars like CTRL-C
        serial_port_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  
        if (serial_port_fd_ == -1) {
            return false;
        }
        
        if (!configureSerialPortForSBus()) {
            close(serial_port_fd_);
            return false;
        } 
        return true;
    }


    bool SBusSerialPort::configureSerialPortForSBus() const{
        // clear config
        fcntl(serial_port_fd_, F_SETFL, 0);
        // read non blocking
        fcntl(serial_port_fd_, F_SETFL, FNDELAY);

        struct termios2 uart_config;
        /* Fill the struct for the new configuration */
        ioctl(serial_port_fd_, TCGETS2, &uart_config);

        // Output flags - Turn off output processing
        // no CR to NL translation, no NL to CR-NL translation,
        // no NL to CR translation, no column 0 CR suppression,
        // no Ctrl-D suppression, no fill characters, no case mapping,
        // no local output processing
        //
        uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

        // Input flags - Turn off input processing
        // convert break to null byte, no CR to NL translation,
        // no NL to CR translation, don't mark parity errors or breaks
        // no input parity check, don't strip high bit off,
        // no XON/XOFF software flow control
        //
        uart_config.c_iflag &=
            ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

        //
        // No line processing:
        // echo off
        // echo newline off
        // canonical mode off,
        // extended input processing off
        // signal chars off
        //
        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

        // Turn off character processing
        // Turn off odd parity
        uart_config.c_cflag &= ~(CSIZE | PARODD | CBAUD);

        // Enable parity generation on output and parity checking for input.
        uart_config.c_cflag |= PARENB;
        // Set two stop bits, rather than one.
        uart_config.c_cflag |= CSTOPB;
        // No output processing, force 8 bit input
        uart_config.c_cflag |= CS8;
        // Enable a non standard baud rate
        uart_config.c_cflag |= BOTHER;

        // Set custom baud rate of 100'000 bits/s necessary for sbus
        const speed_t spd = 100000;
        uart_config.c_ispeed = spd;
        uart_config.c_ospeed = spd;

        if (ioctl(serial_port_fd_, TCSETS2, &uart_config) < 0) {
            printf("could not set configuration of serial port");
            return false;
        }

        return true;
    }


    void SBusSerialPort::transmitSerialSBusMessage() const {
        static uint8_t buffer[kSbusFrameLength_];

        // SBUS header
        buffer[0] = kSbusHeaderByte_;

        // 16 channels of 11 bit data
        buffer[1] = (uint8_t)((this->channels[0] & 0x07FF));
        buffer[2] = (uint8_t)((this->channels[0] & 0x07FF) >> 8 |
                        (this->channels[1] & 0x07FF) << 3);
        buffer[3] = (uint8_t)((this->channels[1] & 0x07FF) >> 5 |
                        (this->channels[2] & 0x07FF) << 6);
        buffer[4] = (uint8_t)((this->channels[2] & 0x07FF) >> 2);
        buffer[5] = (uint8_t)((this->channels[2] & 0x07FF) >> 10 |
                        (this->channels[3] & 0x07FF) << 1);
        buffer[6] = (uint8_t)((this->channels[3] & 0x07FF) >> 7 |
                        (this->channels[4] & 0x07FF) << 4);
        buffer[7] = (uint8_t)((this->channels[4] & 0x07FF) >> 4 |
                        (this->channels[5] & 0x07FF) << 7);
        buffer[8] = (uint8_t)((this->channels[5] & 0x07FF) >> 1);
        buffer[9] = (uint8_t)((this->channels[5] & 0x07FF) >> 9 |
                        (this->channels[6] & 0x07FF) << 2);
        buffer[10] = (uint8_t)((this->channels[6] & 0x07FF) >> 6 |
                         (this->channels[7] & 0x07FF) << 5);
        buffer[11] = (uint8_t)((this->channels[7] & 0x07FF) >> 3);
        buffer[12] = (uint8_t)((this->channels[8] & 0x07FF));
        buffer[13] = (uint8_t)((this->channels[8] & 0x07FF) >> 8 |
                         (this->channels[9] & 0x07FF) << 3);
        buffer[14] = (uint8_t)((this->channels[9] & 0x07FF) >> 5 |
                         (this->channels[10] & 0x07FF) << 6);
        buffer[15] = (uint8_t)((this->channels[10] & 0x07FF) >> 2);
        buffer[16] = (uint8_t)((this->channels[10] & 0x07FF) >> 10 |
                         (this->channels[11] & 0x07FF) << 1);
        buffer[17] = (uint8_t)((this->channels[11] & 0x07FF) >> 7 |
                         (this->channels[12] & 0x07FF) << 4);
        buffer[18] = (uint8_t)((this->channels[12] & 0x07FF) >> 4 |
                         (this->channels[13] & 0x07FF) << 7);
        buffer[19] = (uint8_t)((this->channels[13] & 0x07FF) >> 1);
        buffer[20] = (uint8_t)((this->channels[13] & 0x07FF) >> 9 |
                         (this->channels[14] & 0x07FF) << 2);
        buffer[21] = (uint8_t)((this->channels[14] & 0x07FF) >> 6 |
                         (this->channels[15] & 0x07FF) << 5);
        buffer[22] = (uint8_t)((this->channels[15] & 0x07FF) >> 3);

        // SBUS flags
        // (bit0 = least significant bit)
        // bit0 = ch17 = digital channel (0x01)
        // bit1 = ch18 = digital channel (0x02)
        // bit2 = Frame lost, equivalent red LED on receiver (0x04)
        // bit3 = Failsafe activated (0x08)
        // bit4 = n/a
        // bit5 = n/a
        // bit6 = n/a
        // bit7 = n/a
        buffer[23] = 0x08;

        // SBUS footer
        buffer[24] = kSbusFooterByte_;

        const int written = write(serial_port_fd_, (char*)buffer, kSbusFrameLength_);
        // tcflush(serial_port_fd_, TCOFLUSH); // There were rumors that this might
        // not work on Odroids...
        if (written != kSbusFrameLength_) {
            printf(" Wrote %d bytes but should have written %d ", written, kSbusFrameLength_);
        }
        else{
            printf("Wrote %d bytes \n", written);
        }
    }
}
