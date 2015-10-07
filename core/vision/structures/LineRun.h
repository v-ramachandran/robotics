#ifndef LINE_RUN_H
#define LINE_RUN_H

#include <inttypes.h>

struct LineRun {
  uint16_t posStart;
  uint16_t posEnd;
  uint16_t rowNum;

  LineRun(uint16_t start, uint16_t end, uint16_t row);
};

bool isAbove(struct LineRun &l);

#endif
