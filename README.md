# Arduino RS485 library for ESP32
Arduino library implementing RS485 half duplex serial interface for ESP32. Automatic detects of transmission start and complete events to change the state of the transmission direction pin.

# Usage

```
#include <RS485.h>

RS485 rs485(0); // Interface object
uint8_t MSG_IN[82]; // Message buffer
int i = -1; // Message buffer position
bool received = false; // Message buffer not empty 

void setup(void)
{
  rs485.begin(57600, SERIAL_8N1, Pin_RS485_RX, Pin_RS485_TX, Pin_RS485_DIR);
}

void loop(void)
{
  // Read from RS485
  if (rs485.available())
  {
    MSG_IN[++i] = rs485.read();
    received = true;
  }
  // Echo not empty line
  if (received == true && (MSG_IN[i] < 32 || i == 79))
  {
    if (i > 0)
    {
      MSG_IN[++i] = 13;
      MSG_IN[++i] = 10;
      rs485.write(MSG_IN, i + 1);
    }
    received = false;
    i = -1;
  }

  do_something();
}
```
