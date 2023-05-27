#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h> // write(), read(), close()
#include <termios.h> // Contains POSIX terminal control definitions
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> //Error integer and strerror() function

#include "servo/servo_command.hpp"

// For list return, we use pointer
int * read_temp(int servo_count) {
 // 1. Open Serial Port
 int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
 if (fd < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
 }

 // 2. Configure Serial Port: Baud rate && Parameters set
 struct termios oldtio = { 0 };
 struct termios newtio = { 0 };
 tcgetattr(fd, &oldtio);

 // Set the baud rate(115200) and The other parameters
 newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
 newtio.c_cflag |= CSTOPB; // Set stop field, two stop bits used in communication
 newtio.c_iflag = 0;
 newtio.c_oflag = 0;
 newtio.c_lflag = 0;
 newtio.c_cc[VTIME] = 0;
 newtio.c_cc[VMIN] = 1;
 tcflush(fd, TCIOFLUSH);
 tcsetattr(fd, TCSANOW, &newtio);
 // Set to non-blocking mode, this will be used when reading the serial port
 fcntl(fd, F_SETFL, O_NONBLOCK);


 int * temp_result = (int *)malloc(sizeof(int) * servo_count);
 unsigned char checksum = 0;

 // Transmittion data (3rd item: servo ID)
 for (int i= 1; i <= servo_count; i++){
  unsigned char j = (unsigned char) i;
  unsigned char buffer[6] = {0x55,0x55,j,0x03,0x26,0x00};

  for (int i=2; i<6;i++){
    checksum += buffer[i];
  }
  // 6. Buffer Writing
  checksum = ~ checksum;
  buffer[5] = checksum;

  // 7. Transmission code
  int n = write(fd, buffer, sizeof(buffer));
  if (n != sizeof(buffer)){
    printf("Failed to Send \n");
  }
  else{

    // 8. Read Temperature
    int read_buffer[4] = {0,};
    int ret = read(fd, read_buffer, sizeof(read_buffer));

    if (ret > 0){
      temp_result[i-1] = read_buffer[3];
    }
  }
 }

 return temp_result;
}




// For list return, we use pointer
unsigned short int * read_angle(int servo_count){
 // 1. Open Serial Port
 int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
 if (fd < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
 }

 // 2. Configure Serial Port: Baud rate && Parameters set
 struct termios oldtio = { 0 };
 struct termios newtio = { 0 };
 tcgetattr(fd, &oldtio);

 // Set the baud rate and The other parameters
 newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
 newtio.c_cflag |= CSTOPB; // Set stop field, two stop bits used in communication
 //newtio.c_cflag = CS8 | CLOCAL | CREAD;
 newtio.c_iflag = 0;
 newtio.c_oflag = 0;
 newtio.c_lflag = 0;
 newtio.c_cc[VTIME] = 0;
 newtio.c_cc[VMIN] = 1;
 //cfsetispeed(&newtio, B115200);
 //cfsetospeed(&newtio, B115200);
 tcflush(fd, TCIOFLUSH);
 tcsetattr(fd, TCSANOW, &newtio);
 // Set to non-blocking mode, this will be used when reading the serial port
 fcntl(fd, F_SETFL, O_NONBLOCK);


 unsigned short int * angle_result = (unsigned short int *)malloc(sizeof(unsigned short int) * servo_count);
 unsigned char checksum = 0;

 // Transmittion data (3rd item: servo ID)
 for (int i= 1; i <= servo_count; i++){
  unsigned char j = (unsigned char) i;
  unsigned char buffer[6] = {0x55,0x55,j,0x03,0x28,0x00};

  for (int i=2; i<6;i++){
    checksum += buffer[i];
  }
  // 6. Buffer Writing
  checksum = ~ checksum;
  buffer[5] = checksum;

  // 7. Transmission code
  int n = write(fd, buffer, sizeof(buffer));
  if (n != sizeof(buffer)){
    printf("Failed to Send \n");
  }
  else{

    // 8. Read Temperature
    unsigned char read_buffer[5] = {0};
    int ret = read(fd, read_buffer, sizeof(read_buffer));

    if (ret > 0){
      // 2byte -> int
      angle_result[i-1] = read_buffer[5] << 8 | read_buffer[4];
    }
  }
 }
  return angle_result;
}



void write_angle(int servo_id, int angle)
{
  // 1. Open Serial Port
  int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  if (fd < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }

  // 2. Configure Serial Port: Baud rate && Parameters set
  struct termios oldtio = { 0 };
  struct termios newtio = { 0 };
  tcgetattr(fd, &oldtio);

  // Set the baud rate and The other parameters
  newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  newtio.c_cflag |= CSTOPB; // Set stop field, two stop bits used in communication
  //newtio.c_cflag = CS8 | CLOCAL | CREAD;
  newtio.c_iflag = 0;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 1;
  //cfsetispeed(&newtio, B115200);
  //cfsetospeed(&newtio, B115200);
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);
  // Set to non-blocking mode, this will be used when reading the serial port
  fcntl(fd, F_SETFL, O_NONBLOCK);


  // There is Maybe an error because it is discarded below the decimal point.
  angle = angle * 1000 / 240;

  // 3. Find value for first digit. ex) 1000(decimal) -> 3E8(hexa): Find 3 digits
  unsigned char servo_num;
  servo_num = (unsigned char) servo_id;

  int hexa_base = 16;
  int angle_1,angle_2;
  unsigned char angle_low, angle_high;

  angle_1 = angle / (hexa_base * hexa_base);
  angle_2 = angle - angle_1 * (hexa_base * hexa_base);

  angle_low = (unsigned char) angle_2;
  angle_high = (unsigned char) angle_1;

	// 4. Find checksum
	unsigned char checksum = 0;
	unsigned char buffer[10] = {0x55,0x55,servo_num,0x07,0x01,
                                angle_low,angle_high, 0x0a, 0x00, 0x00};
	for (int i=2; i<10;i++){
		checksum += buffer[i];
  }
	// 5. Buffer Writing
  checksum = ~ checksum;
  buffer[9] = checksum;

	// 6. Transmission code
  int n = write(fd, buffer, sizeof(buffer));
  if (n != sizeof(buffer)){
    printf("Failed to Send \n");
  }
 close(fd);
}

