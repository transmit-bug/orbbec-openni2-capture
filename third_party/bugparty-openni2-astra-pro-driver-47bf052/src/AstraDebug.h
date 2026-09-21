#pragma once
#ifdef ASTRA_DEBUG
#  include <cstdio>
#  define ASTRA_DLOG(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
#else
#  define ASTRA_DLOG(fmt, ...) do {} while (0)
#endif
