#include <vision/structures/LineRun.h>

LineRun::LineRun(uint16_t start, uint16_t end, uint16_t row) {
  posStart = start;
  posEnd = end;
  rowNum = row;
}
