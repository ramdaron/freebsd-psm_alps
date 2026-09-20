KMOD=   psm_alps

SRCS=   psm.c

SRCS+=  opt_isa.h
SRCS+=  opt_psm.h
SRCS+=  opt_evdev.h
SRCS+=  device_if.h
SRCS+=  bus_if.h
SRCS+=  opt_kbd.h

CFLAGS+="-DEVDEV_SUPPORT"

.include <bsd.kmod.mk>
