#ifndef _ERROR_H
#define _ERROR_H

#include <stdio.h>

/*
 * Functions dealing with errors handling.
 *   - functions return a non-zero integer value on error: ERROR return type
 *   - scalable error reporting system: add easily your new errors
 *   - error displayed with failed condition, file, line, function and type of error as soon as it occurs  [TODO]
 *
 * 2013/04/10: Created by Ludovic Lacoste
 */

typedef int ERROR;

enum eErrType {
	ERR_OK,
	ERR_BADPAR,
	ERR_MEM,
	ERR_NOTIMPL,
	NB_ERR
};

#if 0
#define RET_(e) return(-(int)(e))
#else
#define RET_(e) do { fprintf(stderr, "ERROR! in %s:%u@%s: %u\n", __FILE__, __LINE__, __FUNCTION__, (e)); return(-(int)(e)); } while(0)
#endif

#define RET_IF_NOT_(c, e) if(!(c)) RET_(e)

#define RET_IF_ERR_(c) RET_IF_NOT_(!(c), c);

#endif

