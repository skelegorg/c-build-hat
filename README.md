# c-build-hat
Library for controlling the Raspberry Pi BUILD Hat through C.

## Documentation
Examples at the bottom.

To use this library in your program, use `#include "serial.h"`.

**Note:** pretty much all the high level functions and the send_serial function are broken. I recommend using the direct `read`/`write` syscalls to talk through the file descriptor returned by `initialize_serial` in the way described by the BuildHAT serial protocol PDF on the Pi Foundation's website. The Python library is kind of convoluted to reverse.

### Low Level Functions

`int initialize_serial(char *port)`: this function initializes and returns a file descriptor of the serial interface. `char* port` passes the file path of the serial interface. This is probably `/dev/serial0` or `/dev/ttyS0`. This function creates a file descriptor for the serial interface, automatically detects and deals with the firmware uploading process required at every boot.

`int load_firmware(int serial_port)`: this function directly uploads the firmware to the serial port.

`int calc_checksum(unsigned char data[], int len);`: this function shouldn't be used, |it's only called by the `load_firmware` function. It calculates the checksum that is required on firmware upload.

`int send_serial(int serial_port, char* msg, int bytes);`: tries to send a command. I suspect that this function is broken, it will be fixed soon.|

### High Level Functions

`float read_volts(int serial_port)`: returns the voltage coming through the dc power jack on the BuildHAT itself.

`int set_leds(int serial_port, int mode)`: sets the two on-board LEDs to preconfigured states (determined by the board, sorry)

| mode | LED states|
-------|------------
| 0    | both off  |
| 1    | orange    |
| 2    | green     |
| 3    | both      |

`int select_port(int serial_port, char port)`: pass the serial port and a character describing the port (capital and lowercase accepted) which changes the active port of the board. sensor and motor functions all implicitly call this function.

### Peripheral Functions
`Motor` struct: this struct has an `int` member for a serial port and a `char` member for the motor's port.

`Motor initialize_motor(int serial_port, char port)`: creates and returns a `Motor` struct.

`void set_motor_speed(Motor motor, float speed)`: sets a motor to a certain speed. Speeds must be between -1.00 and 1.00 (values like 0.50 are allowed).

`Sensor` struct: identical to the `Motor` struct. Future changes planned.

`int read_ultrasonic(Sensor sensor)`: Returns the distance in front of the distance sensor in millimeters

`int read_color(Sensor sensor)`: Returns the integer associated with the color sensor (these are determined by the board.)

| value | color |
--------|--------
| -1 | error/no object|
|0|black|
|1|magenta|
|2|undocumented by lego (??) |
|3|blue|
|4|turquoise|
|5|green|
|6|undocumented by lego (again??) |
|7|yellow|
|8|somehow also undocumented|
|9|red|Library for controlling the Raspberry Pi BUILD Hat through C.
|10|white|

These colors are drawn from [LEGO's website](https://community.legoeducation.com/blogs/31/220). You can also find the exact RGB values from here. Future update will add an RGB reading function.

## Examples
This example creates an interface, drives a motor from full speed to negative, then reads an ultrasonic and color sensor.

```c
#include "serial.h"

int main() {
  int serial_port = initialize_serial("/dev/serial0");

  Motor motor = initialize_motor(serial_port, 'a');
  for(float i = 1.0; i > -1.0; i--) {
    set_motor_speed(motor, i);
  }
  set_motor_speed(motor, 0);

  Sensor ultrasonic = initialize_sensor(serial_port, 'b');
  printf("Object is %imm away\n", read_ultrasonic(ultrasonic));

  Sensor color = initialize_sensor(serial_port, 'c');
  printf("Color ID: %i\n", read_color(color));

  close(serial_port);
  return 0;
}
```
