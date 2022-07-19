    #include "sbus.h"

    #include "rclcpp/rclcpp.hpp" 
    #include "sensor_msgs/msg/joy.hpp"  

    namespace SBUS{
        bool SBusSerialPort::connectSerialPort() {
            // Open serial port
            // O_RDWR - Read and write
            // O_NOCTTY - Ignore special chars like CTRL-C
            serial_port_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    
            if (serial_port_fd_ == -1) {
                disconnectSerialPort();
                printf("Serial Port Couldn't Open");
                return false;
            }
            
            if (!configureSerialPortForSBus()) {
                disconnectSerialPort();
                printf("Serial Port Couldn't Configure");
                return false;
            } 
            return true;
        }
        
        void SBusSerialPort::disconnectSerialPort() {
            close(serial_port_fd_);
        }

        //SBUS mesajı için serial portu konfigüre eder.
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

        //SBUS mesajlarını 25 byte ayırarak serial port üzerinden gönderir. 
        int* SBusSerialPort::transmitSerialSBusMessage(int channels[16]) const {
            static uint8_t buffer[kSbusFrameLength_];

            // SBUS header
            buffer[0] = kSbusHeaderByte_;

            // 16 channels of 11 bit data
            buffer[1] = (uint8_t)((channels[0] & 0x07FF));
            buffer[2] = (uint8_t)((channels[0] & 0x07FF) >> 8 |
                                (channels[1] & 0x07FF) << 3);
            buffer[3] = (uint8_t)((channels[1] & 0x07FF) >> 5 |
                                (channels[2] & 0x07FF) << 6);
            buffer[4] = (uint8_t)((channels[2] & 0x07FF) >> 2);
            buffer[5] = (uint8_t)((channels[2] & 0x07FF) >> 10 |
                                (channels[3] & 0x07FF) << 1);
            buffer[6] = (uint8_t)((channels[3] & 0x07FF) >> 7 |
                                (channels[4] & 0x07FF) << 4);
            buffer[7] = (uint8_t)((channels[4] & 0x07FF) >> 4 |
                                (channels[5] & 0x07FF) << 7);
            buffer[8] = (uint8_t)((channels[5] & 0x07FF) >> 1);
            buffer[9] = (uint8_t)((channels[5] & 0x07FF) >> 9 |
                                (channels[6] & 0x07FF) << 2);
            buffer[10] = (uint8_t)((channels[6] & 0x07FF) >> 6 |
                                (channels[7] & 0x07FF) << 5);
            buffer[11] = (uint8_t)((channels[7] & 0x07FF) >> 3);
            buffer[12] = (uint8_t)((channels[8] & 0x07FF));
            buffer[13] = (uint8_t)((channels[8] & 0x07FF) >> 8 |
                                (channels[9] & 0x07FF) << 3);
            buffer[14] = (uint8_t)((channels[9] & 0x07FF) >> 5 |
                                (channels[10] & 0x07FF) << 6);
            buffer[15] = (uint8_t)((channels[10] & 0x07FF) >> 2);
            buffer[16] = (uint8_t)((channels[10] & 0x07FF) >> 10 |
                                (channels[11] & 0x07FF) << 1);
            buffer[17] = (uint8_t)((channels[11] & 0x07FF) >> 7 |
                                (channels[12] & 0x07FF) << 4);
            buffer[18] = (uint8_t)((channels[12] & 0x07FF) >> 4 |
                                (channels[13] & 0x07FF) << 7);
            buffer[19] = (uint8_t)((channels[13] & 0x07FF) >> 1);
            buffer[20] = (uint8_t)((channels[13] & 0x07FF) >> 9 |
                                (channels[14] & 0x07FF) << 2);
            buffer[21] = (uint8_t)((channels[14] & 0x07FF) >> 6 |
                                (channels[15] & 0x07FF) << 5);
            buffer[22] = (uint8_t)((channels[15] & 0x07FF) >> 3);

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
            buffer[23] = 0x00;
            /*if (digital_channel_1) {
                buffer[23] |= 0x01;
            }
            if (digital_channel_2) {
                buffer[23] |= 0x02;
            }
            if (frame_lost) {
                buffer[23] |= 0x04;
            }
            if (failsafe) {
                buffer[23] |= 0x08;
            }*/

            // SBUS footer
            buffer[24] = kSbusFooterByte_;

            const int written = write(serial_port_fd_, buffer, kSbusFrameLength_);
            // tcflush(serial_port_fd_, TCOFLUSH); // There were rumors that this might
            // not work on Odroids...
            if (written != kSbusFrameLength_) {
                printf(" Wrote %d bytes but should have written %d \n", written, kSbusFrameLength_);
            }
            else{
                printf("Wrote %d bytes \n", written);
            }
            return channels;
        }
    }

    SBUS::SBusSerialPort sbus;

    class sbus_node: public rclcpp::Node 
    {
    public:
        sbus_node() : Node("sbus_node") 
        {
            //Subscriber
            sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
            std::bind(&sbus_node::getJoyMsg,this,std::placeholders::_1));

            //Belirlediğimiz frekansta SBUS mesajlarını seri port üzerinden gönderir.
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/freq),
            std::bind(&sbus_node::sendSbusMsg, this));
        }
        
    private:
        void getJoyMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
            //Joystickten eksen ve butonların değerlerini float 
            //tipinde joystick adlı bir diziye yerleştiririz.

            //Eksenler 
            //Roll ve Yaw eksenleri en sağda 1972 çıkışı 
            //vermesi için -1 ile çarpıldı.
            joystick[0]  = -joy_msg->axes[3];  //Roll
            joystick[1]  = joy_msg->axes[4];   //Pitch
            joystick[2]  = joy_msg->axes[1];   //Throttle 
            joystick[3]  = -joy_msg->axes[0];  //Yaw

            //R2 ve L2 hem eksen hem de buton olark kullanılabilir. 
            //Joy node'u bu eksenleri varsayılan olarak 1 olarak yayınlar .
            //"-1" ile çarpılır. Aksi takdirde R2 ve L2 eksenleri
            //normal halde dururken 1972 çıkışı alırız.
            joystick[4]  = -joy_msg->axes[2]; 
            joystick[5]  = -joy_msg->axes[5];  

            //Bu iki eksen yön tuşlarıdır. Ve sağ-sol yön tuşu "-1"
            //ile çarpılıp sağ yön tuşuna basıldığında 1972 olacak 
            //şekilde çıkış verilmesi sağlandı.
            joystick[6]  = -joy_msg->axes[6];  
            joystick[7]  = joy_msg->axes[7];   

            //Butonlar
            joystick[8]  = joy_msg->buttons[0]; // X butonu
            joystick[9]  = joy_msg->buttons[1]; // Daire butonu
            joystick[10] = joy_msg->buttons[2]; // Üçgen butonu
            joystick[11] = joy_msg->buttons[3]; // Kare butonu
            joystick[12] = joy_msg->buttons[4]; // L1 butonu
            joystick[13] = joy_msg->buttons[5]; // R1 butonu
            joystick[14] = joy_msg->buttons[6]; // L2 butonu default 0
            joystick[15] = joy_msg->buttons[7]; // R2 butonu default 0 
            
            //Linear Interpolation  y = y0 + ((y1-y0)/(x1-x0)) * (x - x0);
            //Eksenleri ve butonları 192 ile 1972 arasına yerleştirme denklemleri 
            for(int j = 0; j<16; j++){
                if(j<8){
                    joystick[j] = min_sbus + ((max_sbus-min_sbus)/(max_axes-min_axes)) * (joystick[j]-min_axes); 
                }
                else{
                    joystick[j] = min_sbus +((max_sbus-min_sbus)/(max_button-min_button)) * (joystick[j]-min_button); 
                }
                sbusChannels[j] = int(joystick[j]); //Joy mesajları float , SBUS kanalları integer
                                                    //veri tipinde olduğu için dönüştürme gereklidir. 
            }
        }

        //SBUS mesajları Serial Port üzerinden gönderir.
        void sendSbusMsg(){
            int* p;
            p = sbus.transmitSerialSBusMessage(sbusChannels);
            for(int i =0; i<16; i++){
                RCLCPP_INFO(this->get_logger()," channels[%d] = %d", i, *(p+i)); //SBUS kanallarındaki değerleri konsola yazdırır.
            }
        }
        
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        int freq = 125;         //RC kumandanın SBUS frekansı yaklaşık 125 Hz'dir. (T = 8 ms)

        float joystick[16];
        int   sbusChannels[16]; //Joystick dizisindeki float değerleri integer olarak sbusChannels dizisine kaydedilir.      

        //interpolasyon
        bool min_button = 0;
        bool max_button = 1;        

        int min_axes = -1;
        int max_axes =  1;

        int min_sbus = 192;  //FC tarafından 1000 olarak algılanır.  
        int max_sbus = 1792; //FC tarafından 2000 olarak algılanır.  

    };
        
    int main(int argc, char **argv)
    {
        if (!sbus.connectSerialPort())
        {
            return 0;
        }
        
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<sbus_node>());
        rclcpp::shutdown();
        return 0;
    }
    
    
    
    
    
    
    
    
    
    
    #ifndef SBUS_H_
#define SBUS_H_

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <deque>
#include <unistd.h> 
#include <chrono>

namespace SBUS{

class SBusSerialPort {
public:
   bool connectSerialPort();
   int* transmitSerialSBusMessage(int channels[16]) const;

private:
   static constexpr int kSbusFrameLength_ = 25;
   static constexpr uint8_t kSbusHeaderByte_ = 0x0F;
   static constexpr uint8_t kSbusFooterByte_ = 0x00;

   // Digital channels (ch17 and ch18)
   bool digital_channel_1;
   bool digital_channel_2;

   // Flags
   bool frame_lost;
   bool failsafe;
    
   int serial_port_fd_;

   void disconnectSerialPort();
   bool configureSerialPortForSBus() const;
   
};
}

#endif









<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sbus</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="omer@todo.todo">omer</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>









cmake_minimum_required(VERSION 3.8)
project(sbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs)

add_executable(sbus_node src/sbus.cpp)
ament_target_dependencies(sbus_node rclcpp sensor_msgs)
target_include_directories(sbus_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  sbus_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

