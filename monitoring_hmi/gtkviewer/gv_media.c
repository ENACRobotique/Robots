#include <stdlib.h>
#include <assert.h>
#include <time.h>
#include <string.h>

#include <glib.h>
#include <gtk/gtk.h>
#include <gdk-pixbuf/gdk-pixbuf.h>

#include "gv.h"

#include "gv_media.h"

void _media_RawDump() {
    GdkPixbuf *pb;
    GtkWidget *image, *ebox;
    FILE *fd;
    unsigned char *data;
    char fn[64], date[32], cmd[128];
    time_t rawtime;
    struct tm * timeinfo;
    int width, height, rowstride;

    ebox = gtk_notebook_get_nth_page(GTK_NOTEBOOK(_gv_nbmedia), gtk_notebook_get_current_page(GTK_NOTEBOOK(_gv_nbmedia)));

    image = gtk_bin_get_child(GTK_BIN(ebox));
    pb = gtk_image_get_pixbuf(GTK_IMAGE(image));
    data = gdk_pixbuf_get_pixels(pb);

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(date, sizeof(date), "%y%m%d-%X", timeinfo);

    width = gdk_pixbuf_get_width(pb);
    height = gdk_pixbuf_get_height(pb);
    rowstride = gdk_pixbuf_get_rowstride(pb);

    if(rowstride != width*3)
        return;

    snprintf(fn, sizeof(fn)-4, "dump-%s-%s-%dx%d", gtk_label_get_text(GTK_LABEL(gtk_notebook_get_tab_label(GTK_NOTEBOOK(_gv_nbmedia), ebox))), date, width, height);
    sprintf(cmd, "convert -size \"%ix%i\" -depth 8 %s.rgb %s.png ", width, height, fn, fn);
    strcat(fn, ".rgb");

    fd = fopen(fn, "wb+");
    if(!fd)
        return;
    fwrite(data, width*3, height, fd);
    fclose(fd);

    if(!system(cmd)){
        unlink(fn);
        strcpy(fn+strlen(fn)-3, "png");
    }

    printf("data saved to %s\n", fn);

/*printf("channels: %d\n", gdk_pixbuf_get_n_channels(pb));
printf("colorspace: %d(%d)\n", gdk_pixbuf_get_colorspace(pb), GDK_COLORSPACE_RGB);
printf("bitspersample: %d\n", gdk_pixbuf_get_bits_per_sample(pb));
printf("has_alpha: %d\n", gdk_pixbuf_get_has_alpha(pb));
printf("width: %d\n", gdk_pixbuf_get_width(pb));
printf("height: %d\n", gdk_pixbuf_get_height(pb));
printf("rowstride: %d\n", gdk_pixbuf_get_rowstride(pb));*/
}

int gv_media_new(char *name, char *desc, unsigned int width, unsigned int height, GtkSignalFunc handler, void *handler_data) {
    GtkWidget *label = gtk_label_new(name);
    gtk_widget_set_tooltip_text(label, desc);

    GtkWidget *ebox = gtk_event_box_new();
    int mid = gtk_notebook_append_page(GTK_NOTEBOOK(_gv_nbmedia), ebox, label);

    if(handler){
        gtk_widget_add_events(ebox, GDK_BUTTON_RELEASE_MASK);

        gtk_signal_connect(GTK_OBJECT(ebox), "event", handler, handler_data);
    }

    GdkPixbuf *pb_video = gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, width, height);
    GtkWidget *content = gtk_image_new_from_pixbuf(pb_video);
    g_object_unref(pb_video);

    gtk_container_add(GTK_CONTAINER(ebox), content);

    gtk_widget_show(content);
    gtk_widget_show(ebox);
    gtk_widget_show(label);

    return mid;
}

void gv_media_update(int mid, unsigned char *data, unsigned int width, unsigned int height, gv_destroy destroy, void *destroy_data) {
    GtkWidget *content = gtk_bin_get_child(GTK_BIN(gtk_notebook_get_nth_page(GTK_NOTEBOOK(_gv_nbmedia), mid)));
    assert(content);

    GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data(data, GDK_COLORSPACE_RGB, FALSE, 8, width, height, width*3, (GdkPixbufDestroyNotify)destroy, destroy_data);
    gtk_image_set_from_pixbuf(GTK_IMAGE(content), pixbuf);
//    gtk_widget_queue_draw(GTK_WIDGET(content));
    g_object_unref(pixbuf);
}

void gv_media_del(int mid) {
    // TODO
}

