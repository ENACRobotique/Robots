/*
 ============================================================================
 Name        : test_cairo_hmi.c
 Author      : Ludovic Lacoste
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in GTK+
 ============================================================================
 */

// example from http://zetcode.com/gfx/cairo/cairobackends/
#include <cairo.h>
#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include "millis.h"

static gboolean on_draw_event(GtkWidget *widget, cairo_t *cr, gpointer user_data) {
    cairo_set_source_rgb(cr, 0.5+0.5*cos((double)millis()/250.), 0, 0);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, 40.0 + 5.*sin((double)millis()/150.));

    cairo_move_to(cr, 10.0, 50.0 + 5.*cos((double)millis()/200.));
    cairo_show_text(cr, "Disziplin ist Macht.");

    return FALSE;
}

gint invalidate(GtkWidget *darea) {
    gint width, height;
    width = gtk_widget_get_allocated_width(darea);
    height = gtk_widget_get_allocated_height(darea);

    GdkRectangle rect = { .x = 0, .y = 0, .width = width, .height = height };

    gdk_window_invalidate_rect(gtk_widget_get_window(darea), &rect, 1);

    return 1; // continue to be called
}

int main(int argc, char *argv[]) {
    GtkWidget *window;
    GtkWidget *darea;

    gtk_init(&argc, &argv);

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    darea = gtk_drawing_area_new();
    gtk_container_add(GTK_CONTAINER(window), darea);

    g_signal_connect(G_OBJECT(darea), "draw", G_CALLBACK(on_draw_event), NULL);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(window), 400, 90);
    gtk_window_set_title(GTK_WINDOW(window), "GTK window");

    gtk_widget_show_all(window);

    g_timeout_add(1000/30, (GSourceFunc)invalidate, darea); // 30Hz

    gtk_main();

    return 0;
}
