#include "rt_serial_port.h"

struct termios2 {
  tcflag_t c_iflag;       /* input mode flags */
  tcflag_t c_oflag;       /* output mode flags */
  tcflag_t c_cflag;       /* control mode flags */
  tcflag_t c_lflag;       /* local mode flags */
  cc_t c_line;            /* line discipline */
  cc_t c_cc[19];          /* control characters */
  speed_t c_ispeed;       /* input speed */
  speed_t c_ospeed;       /* output speed */
};

#ifndef TCGETS2
#define TCGETS2     _IOR('T', 0x2A, struct termios2)
#endif
#ifndef TCSETS2
#define TCSETS2     _IOW('T', 0x2B, struct termios2)
#endif
#ifndef BOTHER
#define BOTHER      0010000
#endif

rt_serial_port::rt_serial_port() {
    DEBUG_PRINT = false;
    SocketFD = -1;
    SetPortName("");
}

rt_serial_port::rt_serial_port(const char* port_name_) {
    DEBUG_PRINT = false;
    SocketFD = -1;
    baudrate_ = DEFAULT_BAUDRATE;
    SetPortName(port_name_);
}

rt_serial_port::~rt_serial_port() {
    ClosePort();
}

void rt_serial_port::SetPortName(const char* port_name_)
{
    strcpy(port_name, port_name_);
}


bool rt_serial_port::OpenPort()
{
    return SetBaudRate(baudrate_);
}

bool rt_serial_port::SetBaudRate(const int baudrate)
{
    int baud = getCFlagBaud(baudrate);

    ClosePort();

    if(baud <= 0)
    {
        SetupSerialPort(B38400);
        baudrate_ = baudrate;
        return SetCustomBaudRate(baudrate);
    }
    else
    {
        baudrate_ = baudrate;
        return SetupSerialPort(baud);
    }
}

bool rt_serial_port::SetCustomBaudRate(int speed)
{
    struct termios2 options;

    if (ioctl(SocketFD, TCGETS2, &options) != 01)
    {
        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = speed;
        options.c_ospeed = speed;

        if (ioctl(SocketFD, TCSETS2, &options) != -1)
            return true;
    }

    // try to set a custom divisor
    struct serial_struct ss;
    if(ioctl(SocketFD, TIOCGSERIAL, &ss) != 0)
    {
        printf("[rt_serial_port::SetCustomBaudRate] TIOCGSERIAL failed!\t rt_serial_port.cpp:90\n");
        return false;
    }

    ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
    int closest_speed = ss.baud_base / ss.custom_divisor;

    if(closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100)
    {
        printf("[rt_serial_port::SetCustomBaudRate] Cannot set speed to %d, closest is %d\t rt_serial_port.cpp:100 \n", speed, closest_speed);
        return false;
    }

    if(ioctl(SocketFD, TIOCSSERIAL, &ss) < 0)
    {
        printf("[rt_serial_port::SetCustomBaudRate] TIOCSSERIAL failed!\t rt_serial_port.cpp:106\n");
        return false;
    }

    tx_time_per_byte = (1000.0 / (double)speed) * 10.0;
    return true;
}

bool rt_serial_port::SetupSerialPort(int cflag_baud)
{
    struct termios newtio;

    SocketFD = open(port_name, O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(SocketFD < 0)
    {
        printf("[rt_serial_port::SetupSerialPort] Error opening serial port!\n");
        return false;
    }

    bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

    newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;

    // clean the buffer and activate the settings for the port
    tcflush(SocketFD, TCIFLUSH);
    tcsetattr(SocketFD, TCSANOW, &newtio);

    tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
    return true;
}

int rt_serial_port::getCFlagBaud(const int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}

void rt_serial_port::ClosePort()
{
    if(SocketFD != -1)
    {
      close(SocketFD);
    }
    SocketFD = -1;
}

int rt_serial_port::WritePort(unsigned char* packet, int packet_len)
{
    return write(SocketFD, packet, packet_len);
}

int rt_serial_port::ReadPort(unsigned char* packet, int packet_len)
{
    return read(SocketFD, packet, packet_len);
}