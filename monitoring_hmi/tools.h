#ifndef _TOOLS_H
#define _TOOLS_H

#ifndef MAX
#   define MAX(a, b) ((a)>(b)?(a):(b))
#endif

#ifndef MIN
#   define MIN(a, b) ((a)>(b)?(b):(a))
#endif

#ifndef MAX3
#   define MAX3(a, b, c) MAX(a, MAX(b, c))
#endif

#ifndef MIN3
#   define MIN3(a, b, c) MIN(a, MIN(b, c))
#endif

#ifndef CLAMP
#   define CLAMP(m, v, M) MAX(m, MIN(v, M))
#endif

#endif

