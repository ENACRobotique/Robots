#ifndef _GV_MEDIA_H
#define _GV_MEDIA_H

void _media_RawDump();

typedef void (*gv_destroy)(void *, void *);
int     gv_media_new    (char *name, char *desc, unsigned int width, unsigned int height, GtkSignalFunc handler, void *handler_data);
void    gv_media_update (int mid, unsigned char *data, unsigned int width, unsigned int height, gv_destroy destroy, void *destroy_data);
void    gv_media_del    (int mid);

#endif

