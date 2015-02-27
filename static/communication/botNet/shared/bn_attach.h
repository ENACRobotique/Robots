/*
 * bn_attach.h
 *
 *  Created on: 28 déc. 2014
 *      Author: ludo6431
 */

#ifndef LIB_BOTNET_SHARED_BN_ATTACH_H_
#define LIB_BOTNET_SHARED_BN_ATTACH_H_

#include <messages.h>
#include "botNet_core.h"

#ifdef __cplusplus
extern "C" {
#endif

//function pointer type for attach function
typedef void(*pfvpm)(sMsg*);

//bn_attach structure
typedef struct sAttachdef{
    E_TYPE type;
    pfvpm func;
    struct sAttachdef *next;
} sAttach;

//bn_attach first element
extern sAttach *firstAttach;

int bn_attach(E_TYPE type,pfvpm ptr);
int bn_detach(E_TYPE type);

#ifdef __cplusplus
}
#endif

#endif /* LIB_BOTNET_SHARED_BN_ATTACH_H_ */
