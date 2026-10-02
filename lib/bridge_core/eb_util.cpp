#include "eb_util.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

namespace eb {

void Out::printf(const char* fmt, ...) {
  char buf[200];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, args);
  va_end(args);
  if (n <= 0) return;
  if ((size_t)n >= sizeof buf) n = (int)sizeof buf - 1;
  write(buf, (size_t)n);
}

bool parseLong(const char* text, long& value) {
  if (!text || !*text) return false;
  // Decimal, or hex with an explicit 0x prefix. A bare leading zero is not
  // treated as octal, which would silently turn "010" into 8.
  const char* digits = (*text == '-' || *text == '+') ? text + 1 : text;
  const bool hex = digits[0] == '0' && (digits[1] == 'x' || digits[1] == 'X');
  char* end = nullptr;
  const long v = strtol(text, &end, hex ? 16 : 10);
  if (end == text || *end != '\0') return false;
  value = v;
  return true;
}

bool parseDouble(const char* text, double& value) {
  if (!text || !*text) return false;
  char* end = nullptr;
  const double v = strtod(text, &end);
  if (end == text || *end != '\0') return false;
  if (isnan(v) || isinf(v)) return false;
  value = v;
  return true;
}

bool parseFloat(const char* text, float& value) {
  double d;
  if (!parseDouble(text, d)) return false;
  value = (float)d;
  return true;
}

}  // namespace eb
