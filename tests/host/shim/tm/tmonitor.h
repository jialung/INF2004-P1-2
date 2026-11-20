#ifndef SHIM_TMONITOR_H
#define SHIM_TMONITOR_H
#include <stdio.h>
#define tm_printf(...) printf((const char *)__VA_ARGS__)
#endif
