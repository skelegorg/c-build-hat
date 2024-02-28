#include "serial.h"

#define FIRMWARE_PATH "data/firmware.bin"
#define SIGNATURE_PATH "data/signature.bin"

/* 
 *    Low level functions
 */

int initialize_serial(char *port) {
  int serial_port = open(port, O_RDWR);
  if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }

  struct termios tty;
  if (tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  // 8n1 serial settings
  tty.c_cflag &= ~PARENB;                 // clear parity bit
  tty.c_cflag &= ~CSTOPB;                 // one stop bit
  tty.c_cflag &= ~CSIZE;                  // clear sizebit
  tty.c_cflag |= CS8;                     // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;                // hardware flow control off
  tty.c_cflag |= CREAD | CLOCAL;          // turn off modem specific stuff
  tty.c_lflag &= ~ICANON;                 // disable canonical mode
  tty.c_lflag &= ~ECHO;                   // disable echo
  tty.c_lflag &= ~ECHOE;                  // disable erasure
  tty.c_lflag &= ~ECHONL;                 // disable new-line echo
  tty.c_lflag &= ~ISIG;                   // get rid of signal chars
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // software control off
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL);                // all weird input handling stuff
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  tty.c_cc[VTIME] = 10;                   // 1 second response timeout
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  // check if firmware or bootloader is present

  unsigned char msg[] = {'v', 'e', 'r', 's', 'i', 'o', 'n', '\r'};
  write(serial_port, msg, sizeof(msg));
  usleep(10000);

  char res[256];
  memset(&res, '\0', sizeof(res));
  int n = read(serial_port, &res, sizeof(res));

  if (n < 0) {
    printf("Error reading: %s\n", strerror(errno));
    return 1;
  }

  // look for 'bootloader' in the version string
  int bootloader = 0;
  for (int i = 0; i < (n - 4); i++) {
    if (res[i] == 'b' && res[i + 1] == 'o' && res[i + 2] == 'o' &&
        res[i + 3] == 't') {
      bootloader = 1;
      break;
    }
  }

  if (bootloader) {
    if (load_firmware(serial_port) != 0) {
      printf("Error writing firmware\n");
      return -1;
    }
  } 

  usleep(10000);

  return serial_port;
}

// this routine is pulled directly from
//    https://github.com/RaspberryPiFoundation/python-build-hat
int calc_checksum(unsigned char data[], int len) {
  int u = 1;
  for (int i = 0; i < len; i++) {
    if ((u & 0x80000000) != 0) {
      u = (u << 1) ^ 0x1d872b41;
    } else {
      u = u << 1;
    }
    u = (u ^ data[i]) & 0xffffffff;
  }
  return u;
}

int load_firmware(int serial_port) {
  // the last 0 in RP2040 means "no onboard nonvolatile storage" :(

  // clear anything sitting in the serial buffer
  int bytes;
  ioctl(serial_port, FIONREAD, &bytes);
  int trashbuffer[bytes];
  read(serial_port, &trashbuffer, bytes);

  // read firmware and signature files
  FILE *firmware_file = fopen(FIRMWARE_PATH, "rb");
  fseek(firmware_file, 0L, SEEK_END);
  int firmware_length = ftell(firmware_file);
  unsigned char firmware[firmware_length];
  memset(&firmware, '\0', sizeof(firmware));
  rewind(firmware_file);
  fread(firmware, sizeof(unsigned char), sizeof(firmware), firmware_file);
  fclose(firmware_file);
  int checksum = calc_checksum(firmware, firmware_length);

  FILE *signature_file = fopen(SIGNATURE_PATH, "rb");
  fseek(signature_file, 0L, SEEK_END);
  int signature_length = ftell(signature_file);
  unsigned char signature[signature_length];
  memset(&signature, '\0', sizeof(signature));
  rewind(signature_file);
  fread(signature, sizeof(unsigned char), sizeof(signature), signature_file);
  fclose(signature_file);

  char msg[80]; // 80 is just a comfortable number
  sprintf(msg, "load %i %i\r", firmware_length, checksum);
  int n = write(serial_port, &msg, strlen(msg));

  usleep(100000); // 0.1 second delay
  unsigned char header = 0x02;
  unsigned char footer[] = {0x03, '\r'};
  n = write(serial_port, &header, sizeof(header));

  // write data
  n = write(serial_port, &firmware, sizeof(firmware));

  n = write(serial_port, &footer, sizeof(footer));
  usleep(100000); // 0.1 seconds

  // wait for the prompt
  char res[256];
  memset(&res, '\0', sizeof(res));
  usleep(10000);
  n = read(serial_port, &res, sizeof(res));

  if (n < 0) {
    printf("Error reading: %s\n", strerror(errno));
    return 1;
  }
  usleep(10000);

  // "signature <len>\r"
  char sigmsg[80];
  sprintf(sigmsg, "signature %i\r", signature_length);
  write(serial_port, &sigmsg, strlen(sigmsg));

  // "0x02<sig>0x03\r"
  write(serial_port, &header, sizeof(header));

  write(serial_port, &signature, sizeof(signature));

  write(serial_port, &footer, sizeof(footer));

  // prompt
  memset(&res, '\0', sizeof(res));
  usleep(10000);
  n = read(serial_port, &res, sizeof(res));

  unsigned char rebmsg[7] = {'r', 'e', 'b', 'o', 'o', 't', '\r'};
  write(serial_port, &rebmsg, sizeof(rebmsg));
  n = read(serial_port, &res, sizeof(res));

  return 0;
}

int send_serial(int serial_port, char* msg, int bytes) {
  // check if user added a \r to the end, if not add it
  unsigned char msg_buffer[bytes];
  for(int i = 0; i < bytes; i++) {
    msg_buffer[i] = msg[i];
  }
  int n = write(serial_port, &msg_buffer, bytes);
  usleep(1000);
  if(n != strlen(msg)) {
    perror("Did not write all bytes to serial");
  }

  return 0;
}

// DOESNT WORK FOR SOME STUFF???
void read_serial(int serial_port, char* response, int bytes, bool debug) {
  int n = read(serial_port, response, bytes);
  response[n] = 0;
  if(n < 0) {
    perror("Error reading serial");
  }
  if(debug) {
    printf("Read: \n");
    for(int i = 0; i < bytes; i++){
      printf("%c", response[i]);
    }
    printf("\n");
  }
}

void clear_serial(int serial_port) {
  int bytes;
  ioctl(serial_port, FIONREAD, &bytes);
  read(serial_port, NULL, bytes);
}

/* 
 *    High level functions
 */


float read_volts(int serial_port) {
  send_serial(serial_port, "vin\r", 4);
  char res[15];
  read_serial(serial_port, res, 15, false);
  return atof(res);
}


int set_leds(int serial_port, int mode) {

  /*
   * -1: LED's lit depend on the voltage on input 
   *  power jack (default)
   *
   *  0: LEDs off
   *  1: orange
   *  2: green
   *  3: orange and green together
  */

  if(mode < -1 || mode > 3) {
    return -1;
  }

  char msg[12];
  sprintf(msg, "ledmode %i\r", mode);
  send_serial(serial_port, msg, (mode >= 0) ? 10 : 11);
  clear_serial(serial_port);
  return 0;
}

void clear_faults(int serial_port) {
  send_serial(serial_port, "clear_faults\r", 13);
}

int select_port(int serial_port, char port) {
  // for if an int is passed (0, 1, 2, 3)
  if(port < 4) {
    port += 48;
  }
  if(port > 90) {
    // if lowercase is passed
    port -= 45;
  } else if(port > 64) {
    // if uppercase is passed
    port -= 11;
  }
  // if not in allowed range
  if(port < 48 || port > 51) {
    return -1;
  }

  char msg[7] = {'p', 'o', 'r', 't', ' ', port, '\r'};
  printf("Selecting port %i\n", port);
  send_serial(serial_port, msg, 7);
  return 0;
}


/*
 *    giga high level functions
*/


Motor initialize_motor(int serial_port, char port) {
  Motor motor;
  //send_serial(serial_port, "plimit 1; pwm\r", 14);
  motor.serial_port = serial_port;
  motor.port = port;
  return motor;
}

void set_motor_speed(Motor motor, float speed) {
  if(speed > 100 || speed < -100) {
    perror("Disallowed motor speed (must be between 100 and -100)");
  }
  select_port(motor.serial_port, motor.port);
  char cmd[256];
  memset(&cmd, '\0', sizeof(cmd));
  // scale ranges 
  sprintf(cmd, "plimit 1; pwm; set %f\r", speed);
  send_serial(motor.serial_port, cmd, strlen(cmd));
}

Sensor initialize_sensor(int serial_port, char port) {
  Sensor sensor;
  sensor.serial_port = serial_port;
  sensor.port = port;
  return sensor;
}

void enable_sensor(Sensor sensor) {
  select_port(sensor.serial_port, sensor.port);
  send_serial(sensor.serial_port, "on\r", 3);
}

void disable_sensor(Sensor sensor) {
  select_port(sensor.serial_port, sensor.port);
  send_serial(sensor.serial_port, "off\r", 4);
}

// ultrasonic sensor
int read_ultrasonic(Sensor sensor) {
  select_port(sensor.serial_port, sensor.port);
  send_serial(sensor.serial_port, "plimit 1; set -1; selonce 1\r", 28);
  char res[256];
  memset(&res, '\0', sizeof(res));

  // this mess is because the sensor can randomly take way more or less
  //    time to return a reading.
  int bytes = 0;
  bytes += read(sensor.serial_port, &res, sizeof(res));
  while(bytes < 7) {
    usleep(1000);
    bytes += read(sensor.serial_port, &res, sizeof(res) - bytes);
  }
  // response formatting to get distance

  // comes back in format "PnMn: +<mm length>\n"
  //    sometimes comes back as just "nMn: +<mm len>\n" ????
  //    change offset to 7 if you're losing a sig fig of millimeters
  //
  // IF BROKEN REMOVE THE + bytes AND SET OFFSET TO 6
  int offset = 6;
  int reslen = strlen(res);
  int distancelen = reslen - (offset + 1);
  char distance_string[distancelen];
  for(int i = offset; i < offset + distancelen; i++) {
    distance_string[i - offset] = res[i];
  }
  return atoi(distance_string);
}

// color sensor
int read_color(Sensor sensor) {
  select_port(sensor.serial_port, sensor.port);
  write(sensor.serial_port, "plimit 1; set -1; selonce 0\r", 28);
  char res[256];
  memset(&res, '\0', sizeof(res));

  int bytes = 0;
  bytes += read(sensor.serial_port, &res, sizeof(res));
  while(bytes < 7) {
    usleep(1000);
    bytes += read(sensor.serial_port, &res, sizeof(res) - bytes);
  }

  // P0M0:  0
  //      ^^ 2 SPACES OR 1 SPACE + A NEGATIVE
  int offset = 5;
  int reslen = strlen(res);
  int distancelen = reslen - (offset + 1);
  char color_string[distancelen];
  for(int i = offset; i < offset + distancelen; i++) {
    color_string[i - offset] = res[i];
  }

  return atoi(color_string);
}
