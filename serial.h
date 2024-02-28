// linux headers
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

// clib headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// structures
typedef struct Sensor {
  int serial_port;
  char port;
} Sensor;

typedef struct Motor {
  int serial_port;
  char port;
} Motor;

// low level functions
int load_firmware(int serial_port);

int initialize_serial(char *port);

int calc_checksum(unsigned char data[], int len);

int send_serial(int serial_port, char* msg, int bytes);

void read_serial(int serial_port, char* response, int bytes, bool debug);

void clear_serial(int serial_port);

// high level functions

float read_volts(int serial_port);

int set_leds(int serial_port, int mode);

void clear_faults(int serial_port);

int select_port(int serial_port, char port);


// giga high level functions
Motor initialize_motor(int serial_port, char port);

void set_motor_speed(Motor motor, float speed);


Sensor initialize_sensor(int serial_port, char port);

int read_ultrasonic(Sensor sensor);

int read_color(Sensor sensor);
