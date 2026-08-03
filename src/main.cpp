#include <Arduino.h>

// Delegate setup()/loop() to master or slave implementation
// selected at compile time via -DMASTER_MODE=1 in platformio.ini.

#ifdef MASTER_MODE
  #include "master.h"
  void setup() { master_setup(); }
  void loop()  { master_loop();  }
#else
  #include "slave.h"
  void setup() { slave_setup(); }
  void loop()  { slave_loop();  }
#endif
