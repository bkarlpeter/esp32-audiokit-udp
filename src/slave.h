#pragma once

// Entry points called from main.cpp when compiled without -DMASTER_MODE
void slave_setup();
void slave_loop();
