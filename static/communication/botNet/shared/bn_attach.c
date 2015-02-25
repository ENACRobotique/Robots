/*
 * bn_attach.c
 *
 *  Created on: 28 déc. 2014
 *      Author: ludo6431
 */

#include <stdlib.h>

#include "../../../global_errors.h"

#include "bn_attach.h"

//bn_attach first element
sAttach *firstAttach=NULL;

/* bn_attach(E_TYPE type,pfvpm ptr);
 * Arguments :
 *      type : type of the message to attach to.
 *      ptr : pointer to the function to attach.
 * Return value :
 *      0 if assignment correct
 *      -1 if wrong type
 *      -2 if type has already been assigned. (in this case, the previous attachment remains unmodified. see bn_deattach).
 *      -3 if memory allocation fails.
 *  Set an automatic call to function upon reception of a message of type "type".
 *  Warning : after a call to bn_attach, any message of type "type" received by this node WILL NOT be given to the user (won't pop with bn_receive)
 */
int bn_attach(E_TYPE type,pfvpm ptr){
    sAttach *elem=firstAttach, *prev=NULL, *new;

    //checks if the type is correct (should be within the E_TYPE enum range)
    if (type>=E_TYPE_COUNT) return -ERR_BN_TYPE_TOO_HIGH;

    // TODO check if enough free space before allocating
    //looking for already existing occurrence of this type while searching for the last element of the chain
    while ( elem!=NULL){
        if ( elem->type == type ) return -ERR_BN_TYPE_ALREADY_ATTACHED;
        else {
            prev=elem;
            elem=elem->next;
        }
    }

    //create new entry
    if ( (new = (sAttach *)malloc(sizeof(sAttach))) == NULL ) return -ERR_INSUFFICIENT_MEMORY;

    //updates anchor
    if ( firstAttach==NULL) firstAttach=new;
    else prev->next=new;

    new->next=NULL;
    new->type=type;
    new->func=ptr;

    return 0;
}


/* bn_detach(E_TYPE type);
 * Unsets an automatic call to function upon reception of a message of type "type".
 * Arguments :
 *      type : type of the message to remove the attachment from.
 * Return value :
 *      0 if everything went fine
 *      <0 if :
 *          wrong type
 *          type not found (not previously attached, or already de-attached)
 *  Warning : after a call to bn_attach, any message of type "type" received by this node WILL be given to the user (won't pop with bn_receive)
 */
int bn_detach(E_TYPE type){
    sAttach *elem=firstAttach,*nextElem;

    //checks if the type is correct (should be within the E_TYPE enum range)
    if (type>=E_TYPE_COUNT) return -ERR_BN_TYPE_TOO_HIGH;

    if (firstAttach==NULL) return -ERR_NOT_FOUND;

    //looking for already existing occurrence of this type
    //first element
    if (elem->type==type){
        firstAttach=elem->next;
        free(elem);
        return 0;
    }

    //rest of the chain
    while ( elem->next != NULL ){
        if ( elem->next->type == type ) {
            nextElem=elem->next->next;
            free(elem->next);
            elem->next=nextElem;
            return 0;
        }
        else elem=elem->next;
    }

    return -ERR_NOT_FOUND;
}
