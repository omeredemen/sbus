#ifndef SBUS_H
#define SBUS_H

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <deque>
#include <unistd.h> 
#include <iostream>

namespace sbus_bridge {

class SBusSerialPort {
 public:

  bool connectSerialPort();
  void disconnectSerialPort();

  void transmitSerialSBusMessage() const;
  bool configureSerialPortForSBus() const;
  

 private:
  static constexpr int kSbusFrameLength_ = 25;
  static constexpr uint8_t kSbusHeaderByte_ = 0x0F;
  static constexpr uint8_t kSbusFooterByte_ = 0x00;

    uint16_t channels[16] = {1502, 1683, 1792, 1397, 
                             1258, 2000, 1127, 1654,
                             1000, 1200, 1743, 1320,
                             1500, 1543, 1539, 1959};

   // Digital channels (ch17 and ch18)
    bool digital_channel_1;
    bool digital_channel_2;

    // Flags
    bool frame_lost;
    bool failsafe;
    
    int serial_port_fd_;
};

}  // namespace sbus_bridge
#endif
