// Small helpers shared by the bridge core. Portable C++11, no heap.
#pragma once

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace eb {

template <typename T>
inline T clampT(T v, T lo, T hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Integer division rounding half away from zero. `den` must be positive.
inline int32_t divRound(int32_t num, int32_t den) {
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}

// Fixed-capacity byte FIFO over caller-provided storage. push() is
// all-or-nothing so a frame is never queued in part.
class ByteQueue {
 public:
  ByteQueue(uint8_t* storage, size_t capacity)
      : buf_(storage), cap_(capacity), head_(0), size_(0) {}

  size_t size() const { return size_; }
  size_t free() const { return cap_ - size_; }
  void clear() {
    head_ = 0;
    size_ = 0;
  }

  bool push(const uint8_t* data, size_t len) {
    if (len > free()) return false;
    size_t tail = (head_ + size_) % cap_;
    const size_t first = len < cap_ - tail ? len : cap_ - tail;
    memcpy(buf_ + tail, data, first);
    if (len > first) memcpy(buf_, data + first, len - first);
    size_ += len;
    return true;
  }

  size_t pop(uint8_t* out, size_t max) {
    const size_t len = max < size_ ? max : size_;
    const size_t first = len < cap_ - head_ ? len : cap_ - head_;
    memcpy(out, buf_ + head_, first);
    if (len > first) memcpy(out + first, buf_, len - first);
    head_ = (head_ + len) % cap_;
    size_ -= len;
    return len;
  }

 private:
  uint8_t* buf_;
  size_t cap_;
  size_t head_;
  size_t size_;
};

// Text sink used by the CLI so the same command code can answer on a serial
// terminal or inside binary reply frames. Lines end with '\n'; each sink
// translates that as its transport requires.
class Out {
 public:
  virtual ~Out() {}
  virtual void write(const char* text, size_t len) = 0;
  void print(const char* text) { write(text, strlen(text)); }
  void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};

// Strict number parsing: the whole token must be consumed.
bool parseLong(const char* text, long& value);
bool parseFloat(const char* text, float& value);
bool parseDouble(const char* text, double& value);

}  // namespace eb
