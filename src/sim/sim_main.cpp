// Desktop build of the bridge, for development and automated tests without
// any hardware. It runs the same bridge core as the ESP32 firmware and
// exposes its three serial ports as TCP sockets on localhost:
//
//   --crsf PORT   the receiver UART: connect here and play the ELRS receiver
//   --host PORT   the host UART (binary stream by default)
//   --usb  PORT   the USB serial port (CLI by default)
//
// Settings are saved to a text file instead of flash, and 'reboot' restarts
// the bridge in place.
//
//   pio run -e sim
//   .pio/build/sim/program --crsf 5760 --host 5761 --usb 5762
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <mmsystem.h>
typedef SOCKET sock_t;
typedef int socklen_type;
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int sock_t;
typedef socklen_t socklen_type;
#define INVALID_SOCKET (-1)
#endif

#include "eb_bridge.h"

namespace {

void closeSocket(sock_t s) {
#ifdef _WIN32
  closesocket(s);
#else
  close(s);
#endif
}

bool wouldBlock() {
#ifdef _WIN32
  return WSAGetLastError() == WSAEWOULDBLOCK;
#else
  return errno == EWOULDBLOCK || errno == EAGAIN;
#endif
}

void setNonBlocking(sock_t s) {
#ifdef _WIN32
  u_long on = 1;
  ioctlsocket(s, FIONBIO, &on);
#else
  fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
#endif
}

// One serial port of the bridge, served to a single TCP client at a time.
struct TcpPort {
  const char* name;
  int port;
  sock_t listener;
  sock_t client;
};

bool openListener(TcpPort& p) {
  p.client = INVALID_SOCKET;
  p.listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (p.listener == INVALID_SOCKET) return false;
  int yes = 1;
  setsockopt(p.listener, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof yes);
  sockaddr_in addr;
  memset(&addr, 0, sizeof addr);
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons((uint16_t)p.port);
  if (bind(p.listener, (sockaddr*)&addr, sizeof addr) != 0) return false;
  if (listen(p.listener, 1) != 0) return false;
  setNonBlocking(p.listener);
  return true;
}

void dropClient(TcpPort& p) {
  if (p.client == INVALID_SOCKET) return;
  closeSocket(p.client);
  p.client = INVALID_SOCKET;
  fprintf(stderr, "[sim] %s: client disconnected\n", p.name);
}

void acceptClient(TcpPort& p) {
  sockaddr_in addr;
  socklen_type len = sizeof addr;
  const sock_t s = accept(p.listener, (sockaddr*)&addr, &len);
  if (s == INVALID_SOCKET) return;
  // A new connection replaces the old one, like re-plugging a cable.
  dropClient(p);
  p.client = s;
  setNonBlocking(s);
  int yes = 1;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&yes, sizeof yes);
  fprintf(stderr, "[sim] %s: client connected\n", p.name);
}

// Returns bytes read, or 0 when nothing is waiting.
size_t receive(TcpPort& p, uint8_t* buf, size_t cap) {
  if (p.client == INVALID_SOCKET) return 0;
  const int n = recv(p.client, (char*)buf, (int)cap, 0);
  if (n > 0) return (size_t)n;
  if (n == 0 || !wouldBlock()) dropClient(p);
  return 0;
}

void transmit(TcpPort& p, const uint8_t* buf, size_t len) {
  size_t sent = 0;
  int stalls = 0;
  while (p.client != INVALID_SOCKET && sent < len) {
    const int n = send(p.client, (const char*)buf + sent, (int)(len - sent), 0);
    if (n > 0) {
      sent += (size_t)n;
      continue;
    }
    if (!wouldBlock() || ++stalls > 100) {
      dropClient(p);
      return;
    }
    fd_set writable;
    FD_ZERO(&writable);
    FD_SET(p.client, &writable);
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    select((int)p.client + 1, nullptr, &writable, nullptr, &tv);
  }
}

class SimPlatform : public eb::Platform {
 public:
  explicit SimPlatform(const std::string& path) : path_(path), rebootRequested(false) {}

  const char* boardName() override { return "sim"; }
  const char* resetReason() override { return "power-on"; }
  void boardDefaults(eb::Config&) override {}

  // Mirror the classic ESP32's reserved pins so pin validation can be
  // exercised without hardware.
  bool pinUsable(int pin, eb::PinUse use) override {
    if (pin < 0 || pin > 39) return false;
    if (pin == 1 || pin == 3) return false;
    if (pin >= 6 && pin <= 11) return false;
    if (use == eb::kPinOutput && pin >= 34) return false;
    if (use == eb::kPinAnalog && pin < 32) return false;
    return true;
  }

  bool loadConfigText(char* buf, size_t cap, size_t& len) override {
    FILE* f = fopen(path_.c_str(), "rb");
    if (!f) return false;
    len = fread(buf, 1, cap, f);
    const bool ok = feof(f) != 0;
    fclose(f);
    return ok;
  }

  bool saveConfigText(const char* buf, size_t len) override {
    FILE* f = fopen(path_.c_str(), "wb");
    if (!f) return false;
    const size_t written = fwrite(buf, 1, len, f);
    fclose(f);
    return written == len;
  }

  void requestReboot() override { rebootRequested = true; }

 private:
  std::string path_;

 public:
  bool rebootRequested;
};

uint32_t millisSince(const std::chrono::steady_clock::time_point& start) {
  return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start)
      .count();
}

}  // namespace

int main(int argc, char** argv) {
  TcpPort crsfPort = {"crsf", 5760, INVALID_SOCKET, INVALID_SOCKET};
  TcpPort hostPort = {"host", 5761, INVALID_SOCKET, INVALID_SOCKET};
  TcpPort usbPort = {"usb", 5762, INVALID_SOCKET, INVALID_SOCKET};
  std::string configPath = "sim_settings.txt";

  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    if (key == "--crsf") crsfPort.port = atoi(argv[i + 1]);
    else if (key == "--host") hostPort.port = atoi(argv[i + 1]);
    else if (key == "--usb") usbPort.port = atoi(argv[i + 1]);
    else if (key == "--config") configPath = argv[i + 1];
    else {
      fprintf(stderr, "usage: %s [--crsf PORT] [--host PORT] [--usb PORT] [--config FILE]\n",
              argv[0]);
      return 2;
    }
  }

#ifdef _WIN32
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    fprintf(stderr, "[sim] WSAStartup failed\n");
    return 1;
  }
  // Windows otherwise rounds the 1 ms waits below up to ~15 ms, which would
  // make the bridge's timers visibly coarser than on the real hardware.
  timeBeginPeriod(1);
#endif

  TcpPort* ports[3] = {&crsfPort, &hostPort, &usbPort};
  for (TcpPort* p : ports) {
    if (!openListener(*p)) {
      fprintf(stderr, "[sim] cannot listen on 127.0.0.1:%d for %s\n", p->port, p->name);
      return 1;
    }
  }
  fprintf(stderr, "[sim] listening: crsf=%d host=%d usb=%d settings=%s\n", crsfPort.port,
          hostPort.port, usbPort.port, configPath.c_str());

  SimPlatform platform(configPath);
  // The bridge carries several kilobytes of queues; keep it off the stack.
  eb::Bridge* bridge = new eb::Bridge(platform);
  const auto start = std::chrono::steady_clock::now();
  bridge->begin(millisSince(start));

  uint8_t buf[1024];
  for (;;) {
    // Sleep until something arrives, but never longer than one millisecond
    // so the bridge's timers keep running.
    fd_set readable;
    FD_ZERO(&readable);
    int highest = 0;
    for (TcpPort* p : ports) {
      FD_SET(p->listener, &readable);
      if ((int)p->listener > highest) highest = (int)p->listener;
      if (p->client != INVALID_SOCKET) {
        FD_SET(p->client, &readable);
        if ((int)p->client > highest) highest = (int)p->client;
      }
    }
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    select(highest + 1, &readable, nullptr, nullptr, &tv);

    for (TcpPort* p : ports) acceptClient(*p);

    const uint32_t now = millisSince(start);
    size_t n;
    while ((n = receive(crsfPort, buf, sizeof buf)) > 0) bridge->feedCrsf(buf, n, now);
    while ((n = receive(hostPort, buf, sizeof buf)) > 0) {
      bridge->feedPort(eb::Bridge::kPortHost, buf, n, now);
    }
    while ((n = receive(usbPort, buf, sizeof buf)) > 0) {
      bridge->feedPort(eb::Bridge::kPortUsb, buf, n, now);
    }

    bridge->tick(now);

    // Output with nobody connected is discarded, as on an unplugged UART.
    while ((n = bridge->readCrsfTx(buf, sizeof buf)) > 0) transmit(crsfPort, buf, n);
    while ((n = bridge->readPortTx(eb::Bridge::kPortHost, buf, sizeof buf)) > 0) {
      transmit(hostPort, buf, n);
    }
    while ((n = bridge->readPortTx(eb::Bridge::kPortUsb, buf, sizeof buf)) > 0) {
      transmit(usbPort, buf, n);
    }

    if (platform.rebootRequested) {
      platform.rebootRequested = false;
      fprintf(stderr, "[sim] reboot\n");
      bridge->~Bridge();
      new (bridge) eb::Bridge(platform);
      bridge->begin(millisSince(start));
    }
  }
}
