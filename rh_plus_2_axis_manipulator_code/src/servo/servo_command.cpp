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

 int * temp_result = (int *)malloc(sizeof(int) * servo_count);
 for (int j=1;j<=servo_count;j++)
  temp_result[j] = 0;

 for (int i= 1; i <= servo_count; i++){

  int serial_port = open("/dev/ttyUSB0", O_RDWR);
  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

 unsigned char checksum = 0;

 // Transmittion data (3rd item: servo ID)
  unsigned char j = (unsigned char) i;
  unsigned char msg[] = { 0x55,0x55,j,0x03,0x1A,0x00};

  for (int i=2; i<6;i++){
    checksum += msg[i];
  }
  // 6. Buffer Writing
  checksum = ~ checksum;
  msg[5] = checksum;

  // 7. Transmission code
  write(serial_port, msg, sizeof(msg));
  // Allocate memory for read buffer, set size according to your needs
  char read_buf [7];

  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
  }

  while(1){
    if (read_buf[0] == 85 && read_buf[1] == 85 && read_buf[2] == i && read_buf[4] == 26 && read_buf[5] < 100){
      temp_result[i-1] = read_buf[5];
      break;
    }
    else{
      write(serial_port, msg, sizeof(msg));
      num_bytes = read(serial_port,&read_buf,sizeof(read_buf));
    }
  }

  close(serial_port);

 }

 return temp_result;
}




// For list return, we use pointer
unsigned short int * read_angle(int servo_count){

  unsigned short int * angle_result = (unsigned short int *)malloc(sizeof(unsigned short int) * servo_count);

 for (int j=1;j<=servo_count;j++)
  angle_result[j] = 0;


  for (int i= 1; i <= servo_count; i++){

  int serial_port = open("/dev/ttyUSB0", O_RDWR);
  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  unsigned char checksum = 0;

  // Transmittion data (3rd item: servo ID)
  unsigned char j = (unsigned char) i;
  unsigned char msg[] = {0x55,0x55,j,0x03,0x1C,0x00};

    for (int i=2; i<6;i++){
      checksum += msg[i];
    }
    // 6. Buffer Writing
    checksum = ~ checksum;
    msg[5] = checksum;

    // 7. Transmission code
    write(serial_port, msg, sizeof(msg));

    // Allocate memory for read buffer, set size according to your needs
    char read_buf [8] = {0,};

    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
    }

    while(1){
      if (read_buf[0] == 85 && read_buf[1] == 85 && read_buf[2] == i && read_buf[4] == 28 && read_buf[6] < 4){
        // 2byte -> int
        unsigned char result1, result2;
        result1 = read_buf[5];  result2 = read_buf[6];
        angle_result[i-1] = result2 * 256 + result1;
        break;
      }
      else{
        write(serial_port, msg, sizeof(msg));
        num_bytes = read(serial_port,&read_buf,sizeof(read_buf));
      }
    }

    close(serial_port);
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
  // angle = angle * 1000 / 240;

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
