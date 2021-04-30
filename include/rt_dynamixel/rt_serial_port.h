
#ifndef RT_SERIAL_PORT_H
#define RT_SERIAL_PORT_H

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h>

class rt_serial_port
{
private:
    char port_name[20];
    int baudrate_;
    double tx_time_per_byte;

    int GetBaud(int baud);
    bool SetupSerialPort(int baud);
    bool SetBaudRate(int baud);
    bool SetCustomBaudRate(int speed);

    int getCFlagBaud(const int baudrate);
    
public:
    
    int SocketFD;
    static const int DEFAULT_BAUDRATE = 57600;
    bool DEBUG_PRINT;

    rt_serial_port();
    rt_serial_port(const char* port_name);
    virtual ~rt_serial_port();

    bool OpenPort();
    void ClosePort();
    void SetPortName(const char* port_name);

    int WritePort(unsigned char* packet, int packet_len);
    int ReadPort(unsigned char* packet, int packet_len);
};

#endif /* RT_SERIAL_PORT_H */
