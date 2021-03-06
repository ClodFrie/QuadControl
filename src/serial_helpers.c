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
    char sendCmd[] = {'>', '*', '>', 'c'};
    write(fd, sendCmd, sizeof(sendCmd));
    tcdrain(fd);
    ctrl.CRC = crc8(0, (unsigned char*)(&ctrl), sizeof(ctrl) - 1);

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
        tmpRec = read(fd, answer + receivedBytes, sizeof(answer) - receivedBytes);  // read serial data
        receivedBytes += tmpRec;
        if (get_time_ms() > endTime) {
            return -1;
        }
    }

    if (bcmp(answer, "OK", sizeof(answer)) == 0) {
        // it worked
        return 0;
    } else {
        // error
        printf("[CMD] %s\n", answer);
        return -1;
    }
}

int requestData_ser(int fd) {
    char reqData[] = {'>', '*', '>', 'd'};

    int receivedBytes = 0;
    double endTime = get_time_ms() + /*MS_TIMEOUT*/ 20;
    write(fd, reqData, sizeof(reqData));
    tcdrain(fd);

    unsigned char* rptr = (unsigned char*)&data;

    bzero(&data, sizeof(data));  // clear data

    while (receivedBytes < sizeof(data)) {
        if (sizeof(data) - receivedBytes > FIFO_BUFFER_SIZE) {
            receivedBytes += read(fd, rptr + receivedBytes, FIFO_BUFFER_SIZE);  // read serial data
        } else {
            receivedBytes += read(fd, rptr + receivedBytes, sizeof(data) - receivedBytes);  // read serial data
        }
        if (get_time_ms() > endTime) {
            printf("[DATA] read timed out\n");
            return 1;
        }
    }
    usleep(1000);
    // tcflush(fd,TCIOFLUSH);

    unsigned char crc = crc8(0, (unsigned char*)(&data), sizeof(data) - 1);
    if (data.CRC != crc) {
        fprintf(stderr, "[CRC ERROR] Receiving side\n");
        bzero(&data, sizeof(data));  // delete corrupted data
        return 1;
    }
    return 0;
}
int sendParameters(int fd) {
    char sendParam[] = {'>', '*', '>', 'p'};
    write(fd, sendParam, sizeof(sendParam));
    tcdrain(fd);

    ctrl.CRC = crc8(0, (unsigned char*)(&params), sizeof(params) - 1);
    double endTime = get_time_ms() + MS_TIMEOUT;

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
        if (get_time_ms() > endTime) {
            fprintf(stderr, "[PARAM] read timed out %lf \n", get_time_ms() - endTime);
            return -1;
        }
    }

    usleep(1000);
    char answer[] = "NK";
    int receivedBytes = 0;
    int tmpRec = 0;
    while (receivedBytes < sizeof(answer)) {
        tmpRec = read(fd, answer + receivedBytes, sizeof(answer));  // read serial data
        receivedBytes += tmpRec;
        if (get_time_ms() > endTime) {
            printf("[PARAM] read timed out %lf \n", get_time_ms() - endTime);
            return -1;
        }
    }

    if (bcmp(answer, "OK", sizeof(answer)) == 0) {
        // it worked
        printf("[PARAM] %s\n", answer);
        return 0;
    } else {
        // error
        printf("[PARAM] %s\n", answer);
        return -1;
    }
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
