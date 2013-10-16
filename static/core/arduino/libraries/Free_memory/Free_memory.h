

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void __bss_end;
extern void *__brkval;

int get_free_memory();


#ifdef __cplusplus
}
#endif
