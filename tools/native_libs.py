# PlatformIO pre-build script for the desktop (native) environment.
#
# On Windows the simulator needs the Winsock and multimedia-timer libraries,
# and linking statically lets the executable run without the compiler's DLLs
# on PATH.
import sys

Import("env")  # noqa: F821 - provided by PlatformIO

if sys.platform.startswith("win"):
    env.Append(LIBS=["ws2_32", "winmm"], LINKFLAGS=["-static"])  # noqa: F821
