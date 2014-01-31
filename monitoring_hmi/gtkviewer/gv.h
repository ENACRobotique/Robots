#ifndef _GTKVIEWER_H
#define _GTKVIEWER_H

#include <gtk/gtk.h>

#include "gv_media.h"
//#include "gv_params.h"

extern GtkWidget *_gv_window;
    extern GtkWidget *_gv_hbox;
        extern GtkWidget *_gv_nbmedia; // notebook
//        extern GtkWidget *_gv_nbparams;    // notebook

void    gv_init (int *pargc, char **pargv[], char *wtitle, char *help);
void    gv_run  ();

#endif

