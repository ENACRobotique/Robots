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

typedef struct {
    GtkWidget *window;
    GtkWidget *drawing_area;
    gint da_width, da_height;

    double wld_width, wld_height;
} sContext;

static gboolean on_draw_event(GtkWidget *widget, cairo_t *cr, sContext *ctx) {
    { // prepare transformation for playground to fit in window not stretched
        ctx->da_width = gtk_widget_get_allocated_width(ctx->drawing_area);
        ctx->da_height = gtk_widget_get_allocated_height(ctx->drawing_area);

        double scale = (double) ctx->da_height / ctx->wld_height;
        double w = scale * ctx->wld_width, h;
        if (w < ctx->da_width) {
            cairo_translate(cr, ((double) ctx->da_width - w) / 2., 0.);
        }
        else {
            scale = (double) ctx->da_width / ctx->wld_width;
            h = scale * ctx->wld_height;
            cairo_translate(cr, 0., ((double) ctx->da_height - h) / 2.);
        }
        cairo_scale(cr, scale, scale);
        cairo_translate(cr, 0, ctx->wld_height);
        cairo_scale(cr, 1, -1);
    }

    { // this is where we draw in centimeters!
      // draw x axis
        cairo_set_source_rgb(cr, 1, 0, 0);
        cairo_move_to(cr, 0, 0);
        cairo_line_to(cr, 10, 0);
        cairo_stroke(cr);

        // draw y axis
        cairo_set_source_rgb(cr, 0, 1, 0);
        cairo_move_to(cr, 0, 0);
        cairo_line_to(cr, 0, 10);
        cairo_stroke(cr);

        // draw playground outline
        double x = 1.5, y = 1.5;
        cairo_device_to_user_distance(cr, &x, &y);
        cairo_set_line_width(cr, MAX(x, y));
        cairo_set_source_rgba(cr, 0, 0, 0, 0.5);
        cairo_rectangle(cr, 0., 0., ctx->wld_width, ctx->wld_height);
        cairo_stroke(cr);

        // draw some moving text
        cairo_move_to(cr, 100.0, 50.0 + 5. * cos((double) millis() / 200.));
        cairo_save(cr);
        cairo_scale(cr, 1, -1);
        cairo_set_source_rgb(cr, 0.5 + 0.5 * cos((double) millis() / 250.), 0, 0);
        cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_font_size(cr, 40.0 + 5. * sin((double) millis() / 150.));
        cairo_show_text(cr, "Disziplin ist Macht.");
        cairo_restore(cr);
    }

    return FALSE;
}

gboolean invalidate(sContext *ctx) {
    // for now, invalidate all
    GdkRectangle rect = { .x = 0, .y = 0, .width = ctx->da_width, .height = ctx->da_height };

    gdk_window_invalidate_rect(gtk_widget_get_window(ctx->drawing_area), &rect, 1);

    return TRUE; // continue to be called
}

int main(int argc, char *argv[]) {
    sContext ctx;

    // cairo coordinates will be managed in centimeters
    ctx.wld_width = 300.;
    ctx.wld_height = 200.;

    gtk_init(&argc, &argv);

    ctx.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    ctx.drawing_area = gtk_drawing_area_new();
    gtk_container_add(GTK_CONTAINER(ctx.window), ctx.drawing_area);

    g_signal_connect(G_OBJECT(ctx.drawing_area), "draw", G_CALLBACK(on_draw_event), &ctx);
    g_signal_connect(ctx.window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_window_set_position(GTK_WINDOW(ctx.window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(ctx.window), 400, 90);
    gtk_window_set_title(GTK_WINDOW(ctx.window), "GTK window");

    gtk_widget_show_all(ctx.window);

    g_timeout_add(1000 / 25, (GSourceFunc) invalidate, &ctx); // 25Hz

    gtk_main();

    return 0;
}
