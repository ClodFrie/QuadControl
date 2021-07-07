#include <fcntl.h>
#include <stdio.h>
#include <strings.h>  // included for bzero
#include <termio.h>
#include <unistd.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/crc.h"
#include "../include/time_helper.h"

void configurePort(int fd);

int openPort(char portNo) {
    // get port number
    char port[] = "/dev/ttyUSBX";
    // read(0, &portNo, 1);  // read port number
    port[11] = portNo + '0';  // assign port number
    // open serial port
    int fd = open(port, O_RDWR | O_NOCTTY);
    usleep(1000);
    if (fd == -1) {
        printf("ERROR: Can't open ");
        printf("%s", port);
        printf("\n");
        return -1;
    } else {
        printf("%s", port);
        printf(" opened correctly.\n");
        configurePort(fd);
        return fd;
    }

    return -1;
}

int sendCommand(int fd) {
    char sendCmd[] = ">*>c";
    write(fd, sendCmd, sizeof(sendCmd));
    tcdrain(fd);
    unsigned char crc0 = crc8(0, NULL, 0);
    ctrl.CRC = crc8(crc0, (unsigned char*)(&ctrl), sizeof(ctrl) - 1);

    unsigned char* ptr = (unsigned char*)&ctrl;
    int bytesLeft = sizeof(ctrl);
    while (bytesLeft > 0) {
        if (bytesLeft > FIFO_BUFFER_SIZE) {
            write(fd, ptr + sizeof(ctrl) - bytesLeft, FIFO_BUFFER_SIZE);  // send 15 bytes
            tcdrain(fd);
            bytesLeft -= FIFO_BUFFER_SIZE;  // decrement remaining bytes
        } else if (bytesLeft > 0 && bytesLeft <= FIFO_BUFFER_SIZE) {
            write(fd, ptr + sizeof(ctrl) - bytesLeft, bytesLeft);  // send remaining bytes
            tcdrain(fd);
            bytesLeft = 0;
        }
    }
    usleep(1000);
    char answer[] = "NK";
    int receivedBytes = 0;
    int tmpRec = 0;
    double endTime = get_time_ms() + MS_TIMEOUT;
    while (receivedBytes < sizeof(answer)) {
        tmpRec = read(fd, answer + receivedBytes, FIFO_BUFFER_SIZE);  // read serial data
        receivedBytes += tmpRec;
        if (get_time_ms() > endTime) {
            return -1;
        }
    }
}

int requestData_ser(int fd) {
    char reqData[] = ">*>d";

    int receivedBytes = 0;
    int tmpRec = 0;
    double endTime = get_time_ms() + MS_TIMEOUT;
    write(fd, reqData, sizeof(reqData));
    tcdrain(fd);
    usleep(1000);
    unsigned char* rptr = (unsigned char*)&data;

    bzero(&data, sizeof(data));  // clear data

    while (receivedBytes < sizeof(data) && get_time_ms() < endTime) {
        tmpRec = read(fd, rptr + receivedBytes, FIFO_BUFFER_SIZE);  // read serial data
        receivedBytes += tmpRec;
        double actTime = get_time_ms();
        if (actTime > endTime) {
            bzero(&data, sizeof(data));  // clear already received data
        }
    }

    unsigned char crc0 = crc8(0, NULL, 0);
    unsigned char crc = crc8(crc0, (unsigned char*)(&data), sizeof(data) - 1);
    if (data.CRC != crc) {
        fprintf(stderr, "[CRC ERROR] Receiving side\n");
        bzero(&data, sizeof(data));  // delete corrupted data
    }
}
int sendParameters(int fd) {
    char sendParam[] = ">*>p";
    write(fd, sendParam, sizeof(sendParam));
    tcdrain(fd);

    unsigned char crc0 = crc8(0, NULL, 0);
    ctrl.CRC = crc8(crc0, (unsigned char*)(&params), sizeof(params) - 1);

    unsigned char* ptr = (unsigned char*)&params;
    int bytesLeft = sizeof(params);
    while (bytesLeft > 0) {
        if (bytesLeft > FIFO_BUFFER_SIZE) {
            write(fd, ptr + sizeof(params) - bytesLeft, FIFO_BUFFER_SIZE);  // send 15 bytes
            tcdrain(fd);
            bytesLeft -= FIFO_BUFFER_SIZE;  // decrement remaining bytes
        } else if (bytesLeft > 0 && bytesLeft <= FIFO_BUFFER_SIZE) {
            write(fd, ptr + sizeof(params) - bytesLeft, bytesLeft);  // send remaining bytes
            tcdrain(fd);
            bytesLeft = 0;
        }
    }

    
    usleep(1000);
    char answer[] = "NK";
    int receivedBytes = 0;
    int tmpRec = 0;
    double endTime = get_time_ms() + MS_TIMEOUT;
    while (receivedBytes < sizeof(answer)) {
        tmpRec = read(fd, answer + receivedBytes, FIFO_BUFFER_SIZE);  // read serial data
        receivedBytes += tmpRec;
        if (get_time_ms() > endTime) {
            return -1;
        }
    }

    if (bcmp(answer, "OK", (int)receivedBytes) != 0) {
        fprintf(stderr, "%s", answer);
        return 0;
    }
    return -1;
}
void configurePort(int fd) {
    // for information about port settings refer to
    // http://tldp.org/HOWTO/Serial-Programming-HOWTO/x56.html, or
    // https://www.cmrr.umn.edu/~strupp/serial.html.

    struct termios port_settings;  // structure to store the port settings in

    bzero(&port_settings, sizeof(port_settings));  // empty port_settings

    port_settings.c_cflag = B115200 /*| CRTSCTS*/ | CS8 | CLOCAL | CREAD;
    port_settings.c_iflag = IGNPAR;
    port_settings.c_oflag = 0;
    /*set input mode (non-canonical, no echo, ...)*/
    port_settings.c_lflag = 0;

    port_settings.c_cc[VTIME] = 0;  // inter-character timer unused
    port_settings.c_cc[VMIN] = 0;   // read non blocking

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &port_settings);  // apply settings to the port
}
