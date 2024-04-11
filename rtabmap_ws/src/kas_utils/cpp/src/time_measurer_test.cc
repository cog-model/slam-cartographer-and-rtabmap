#include "kas_utils/time_measurer.h"

#include <unistd.h>
#include <iostream>

void long_operation(unsigned int n) {
  usleep(n);
}

int main() {
  kas_utils::TimeMeasurer long_operation_time_measurer("long_operation", true);
  int num = 10;
  unsigned int operation_duration = 100000;
  for (int i = 0; i < num; i++) {
    long_operation_time_measurer.start();
    long_operation(operation_duration);
    long_operation_time_measurer.stop();
  }
}
