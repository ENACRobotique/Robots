#include <stdlib.h>
#include <string.h>

#include <glib.h>
#include <gtk/gtk.h>

#include "gv.h"

GtkWidget *_gv_window;
    GtkWidget *_gv_hbox;
        GtkWidget *_gv_nbmedia; // notebook
//        GtkWidget *_gv_nbparams;    // notebook

void key_event(GtkWidget *widget, GdkEvent *event, gpointer data) {
    unsigned int k;

    k = event->key.keyval;
//printf("%08x %s\n", k, event->key.string);
    if (k < 0xff) {
        switch (k) {
        case 'g':		/* grab a raw image */
            _media_RawDump();
            break;
        default:
            break;
        }
    }
}

void gv_init(int *pargc, char **pargv[], char *wtitle, char *help) {
    GdkColor bg = {0, 32768, 32768, 32768};

    // init GUI
    gtk_init(pargc, pargv);

    // the main window
    _gv_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(_gv_window), wtitle);
    g_signal_connect(_gv_window, "delete-event", G_CALLBACK(gtk_main_quit), NULL);
    g_signal_connect(_gv_window, "key-release-event", G_CALLBACK(key_event), NULL);

        // the main horizontal box
        _gv_hbox = gtk_hbox_new(FALSE, 0);
        gtk_container_add(GTK_CONTAINER(_gv_window), _gv_hbox);

            // the notebook for the media
            _gv_nbmedia = gtk_notebook_new();
            gtk_widget_modify_bg(GTK_WIDGET(_gv_nbmedia), GTK_STATE_NORMAL, &bg);
            gtk_container_add(GTK_CONTAINER(_gv_hbox), _gv_nbmedia);

                // the help first page
                GtkTextBuffer *bhwtxt = gtk_text_buffer_new(NULL);
                gtk_text_buffer_insert_at_cursor(bhwtxt, help, strlen(help));
                GtkWidget *whwtxt = gtk_text_view_new_with_buffer(bhwtxt);
                gtk_text_view_set_editable(GTK_TEXT_VIEW(whwtxt), FALSE);
                GtkWidget *lhwtxt = gtk_image_new_from_stock(GTK_STOCK_HOME, GTK_ICON_SIZE_BUTTON);
                gtk_notebook_prepend_page(GTK_NOTEBOOK(_gv_nbmedia), whwtxt, lhwtxt);
                gtk_widget_show(whwtxt);
                gtk_widget_show(lhwtxt);

            gtk_widget_show(_gv_nbmedia);

            // the notebook for the parameters
//            _gv_nbparams = gtk_notebook_new();
//            gtk_container_add(GTK_CONTAINER(_gv_hbox), _gv_nbparams);
//            gtk_widget_show(_gv_nbparams);

        gtk_widget_show(_gv_hbox);

    gtk_widget_show(_gv_window);
}

void gv_run() {
    gtk_main();
}

