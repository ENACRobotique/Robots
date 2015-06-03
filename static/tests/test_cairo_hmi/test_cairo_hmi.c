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
#include <string.h>
#include "millis.h"
#include "time_tools.h"
#include "pos_uncertainty.h"

#include "context.h"

#include "cairo_tools.h"

gboolean on_invalidate(sContext *ctx) {
    da_invalidate_all(&ctx->da);

    return TRUE; // continue to be called
}

static gboolean on_event(GtkWidget *widget, GdkEvent *event, sContext *ctx) {
    switch (event->type) {
    case GDK_BUTTON_PRESS:
        if (event->button.button == 1) {
            da_event_click__px(&ctx->da, event->button.x, event->button.y);

            da_event_pan_start__px(&ctx->da, event->button.x, event->button.y);
        }
        break;
    case GDK_MOTION_NOTIFY:
        if (da_state_is_panning(&ctx->da)) {
            da_event_pan_update_abs__px(&ctx->da, event->motion.x, event->motion.y);
        }

        da_event_hover__px(&ctx->da, event->motion.x, event->motion.y);
        break;
    case GDK_BUTTON_RELEASE:
        if (event->button.button == 1) {
            da_event_pan_update_abs__px(&ctx->da, event->button.x, event->button.y);
            da_event_pan_stop(&ctx->da);
        }
        break;
    case GDK_SCROLL:
        switch (event->scroll.direction) {
        case GDK_SCROLL_UP:
            da_event_scale_rel(&ctx->da, 1.1);
            break;
        case GDK_SCROLL_DOWN:
            da_event_scale_rel(&ctx->da, 1. / 1.1);
            break;
        default:
            break;
        }

        break;
    case GDK_KEY_PRESS: {
//            GdkModifierType modifiers;
//
//            modifiers = gtk_accelerator_get_default_mod_mask ();
//
//            printf("mod %08x\n", modifiers);
//            printf("kyv %08x\n", event->key.keyval);
//            printf("sta %08x\n", event->key.state);
//
//            if(event->key.state & GDK_BUTTON1_MASK){
//                printf("button1\n");
//            }
//            if(event->key.state & GDK_CONTROL_MASK){
//                printf("control\n");
//            }
//
//            if (event->key.keyval == GDK_KEY_F10
//                    && (event->key.state & modifiers) == GDK_CONTROL_MASK)
//            {
//                g_print ("Control-F10 was pressed\n");
//            }
    }
        break;
    default:
        break;
    }

    if (da_need_early_update(&ctx->da)) {
        // ask redraw
        da_invalidate_all(&ctx->da);
    }

    return TRUE;
}

static gboolean on_draw_event(GtkWidget *widget, cairo_t *cr, sContext *ctx) {
    da_prepare_draw(&ctx->da, cr);
    // this is where we draw in centimeters!

    // draw x axis
    cairo_set_source_rgb(cr, 1, 0, 0);
    cairo_move_to(cr, 0, 0);
    cairo_line_to(cr, 10, 0);
    cairo_stroke(cr);

    // draw press_y axis
    cairo_set_source_rgb(cr, 0, 1, 0);
    cairo_move_to(cr, 0, 0);
    cairo_line_to(cr, 0, 10);
    cairo_stroke(cr);

    // draw playground outline
    double x = 1.5, y = 1.5;
    cairo_device_to_user_distance(cr, &x, &y);
    cairo_set_line_width(cr, MAX(x, y));
    cairo_set_source_rgba(cr, 0, 0, 0, 0.5);
    cairo_rectangle(cr, 0., 0., ctx->da.wld_width, ctx->da.wld_height);
    cairo_stroke(cr);

    // draw grid
    {
        double x = 1.2, y = 1.2;
        int i;
        cairo_device_to_user_distance(cr, &x, &y);
        cairo_set_line_width(cr, MAX(x, y));
        double dashes[] = { 3. / 2., 2. / 2. };
        cairo_set_dash(cr, dashes, sizeof(dashes) / sizeof(*dashes), 0);
        cairo_set_source_rgba(cr, 0, 0, 0, 0.3);

        double incr = 10.;

        for (i = 1; i < ceil(ctx->da.wld_width / incr); i++) {
            cairo_move_to(cr, (double) i * incr, 0);
            cairo_line_to(cr, (double) i * incr, ctx->da.wld_height);
        }

        for (i = 1; i < ceil(ctx->da.wld_height / incr); i++) {
            cairo_move_to(cr, 0, (double) i * incr);
            cairo_line_to(cr, ctx->da.wld_width, (double) i * incr);
        }

        cairo_stroke(cr);
        cairo_set_dash(cr, NULL, 0, 0);
    }

    // draw previous click
    {
        if (da_state_click_moved(&ctx->da)) {
            char text[32];

            double a = (double) millis() / 1000.; // trick to avoid precision pb with double 2 float conversion (millis() may be big!!)
            a -= 2 * M_PI * (int) (a / (2 * M_PI));

            sprintf(text, "x:%.2f, y:%.2f, theta=%.2f\n", ctx->i2.pos.x, ctx->i2.pos.y, a * 180. / M_PI);
            GtkTextBuffer *buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(ctx->console));
            gtk_text_buffer_insert_at_cursor(buffer, text, strlen(text));
            GtkTextIter iter;
            gtk_text_buffer_get_end_iter(buffer, &iter);
            gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(ctx->console), &iter, 0, FALSE, 0, 0);
        }

        cairo_arc(cr, ctx->da.mouse_lastpress_x__px, ctx->da.mouse_lastpress_y__px, 5, 0, 2 * M_PI);
        cairo_stroke(cr);
    }

    // draw test data
    {
        sGenericPosStatus o;

        if (da_state_hover_moved(&ctx->da)) {
            da_state_get_hover__cm__float(&ctx->da, &ctx->i2.pos.x, &ctx->i2.pos.y);
        }

        double a = (double) millis() / 1000.; // trick to avoid precision pb with double 2 float conversion (millis() may be big!!)
        a -= 2 * M_PI * (int) (a / (2 * M_PI));
        ctx->i2.pos_u.a_angle = a;

        // fixed ellipse (RED)
        cairo_set_source_rgb(cr, 1, 0, 0);
        cairo_ellipse(cr, ctx->i1.pos.x, ctx->i1.pos.y, 2 * sqrt(ctx->i1.pos_u.a_var), 2 * sqrt(ctx->i1.pos_u.b_var), -ctx->i1.pos_u.a_angle); // 95% ellipse
        cairo_stroke(cr);

        // ellipse following mouse pointer and rotating with respect to the time (GREEN)
        cairo_set_source_rgb(cr, 0, 1, 0);
        cairo_ellipse(cr, ctx->i2.pos.x, ctx->i2.pos.y, 2 * sqrt(ctx->i2.pos_u.a_var), 2 * sqrt(ctx->i2.pos_u.b_var), -ctx->i2.pos_u.a_angle); // 95% ellipse
        cairo_stroke(cr);

        // actual multiplication
        pos_uncertainty_mix(&ctx->i1, &ctx->i2, &o);

        // result of the multiplication of the 2 previous 2D gaussians (BLUE)
        cairo_set_source_rgb(cr, 0, 0, 1);
        cairo_ellipse(cr, o.pos.x, o.pos.y, 2 * sqrt(o.pos_u.a_var), 2 * sqrt(o.pos_u.b_var), -o.pos_u.a_angle); // 95% ellipse
        cairo_stroke(cr);

        // interactive text following result ellipse
        {
            char text[64];
            double va = o.pos_u.a_var, vb = o.pos_u.b_var, a = o.pos_u.a_angle;

            if (va < vb) {
                double tmp = vb;
                vb = va;
                va = tmp;
                a += M_PI / 2.;
            }
            while (a > M_PI / 2.)
                a -= M_PI;
            while (a < -M_PI / 2.)
                a += M_PI;

            sprintf(text, "x:%.2fcm, y:%.2fcm", o.pos.x, o.pos.y);
            cairo_text(cr, o.pos.x + 2, o.pos.y, 5, text);

            sprintf(text, "Va=%.2fcm², Vb=%.2fcm², a_theta=%.2f°", va, vb, a * 180. / M_PI);
            cairo_text(cr, o.pos.x + 2, o.pos.y - 7, 5, text);
        }
    }

    return FALSE;
}

int main(int argc, char *argv[]) {
    sContext ctx = { 0 };

    float initialSizeFactor = 3;

    // cairo coordinates will be managed in centimeters
    ctx.da.wld_width = 300.;
    ctx.da.wld_height = 200.;
    ctx.da.scale = initialSizeFactor;
    ctx.da.center_x__cm = ctx.da.wld_width / 2.;
    ctx.da.center_y__cm = ctx.da.wld_height / 2.;
    ctx.da.center_px_moved = FALSE;

    // test setup
    ctx.i1.id = ELT_PRIMARY;
    ctx.i1.date = TD_GET_LoUs(tD_newNow_Lo());
    ctx.i1.pos.frame = FRAME_PLAYGROUND;
    ctx.i1.pos.x = 100.;
    ctx.i1.pos.y = 100.;
    ctx.i1.pos_u.a_angle = -10. * M_PI / 180.;
    ctx.i1.pos_u.a_var = 4;
    ctx.i1.pos_u.b_var = 25;
    ctx.i1.pos.theta = 0.;
    ctx.i1.pos_u.theta = 0.;
    ctx.i2.id = ELT_PRIMARY;
    ctx.i2.date = ctx.i1.date;
    ctx.i2.pos.frame = FRAME_PLAYGROUND;
    ctx.i2.pos.x = 102.;
    ctx.i2.pos.y = 103.;
    ctx.i2.pos_u.a_angle = 20. * M_PI / 180.;
    ctx.i2.pos_u.a_var = 0.5;
    ctx.i2.pos_u.b_var = 10;
    ctx.i2.pos.theta = 0.;
    ctx.i2.pos_u.theta = 0.;

    gtk_init(&argc, &argv);

    ctx.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    ctx.da.widget = gtk_drawing_area_new();
    ctx.console = gtk_text_view_new();

    {
        GtkWidget *scrolledwindowConsole = gtk_scrolled_window_new(NULL, NULL);
        GtkWidget* paned = gtk_paned_new(GTK_ORIENTATION_VERTICAL);

        gtk_container_add(GTK_CONTAINER(scrolledwindowConsole), ctx.console);

        gtk_widget_set_hexpand(ctx.da.widget, TRUE);
        gtk_widget_set_vexpand(ctx.da.widget, TRUE);
        gtk_widget_set_hexpand(scrolledwindowConsole, TRUE);

        gtk_paned_pack1(GTK_PANED(paned), ctx.da.widget, TRUE, FALSE);
        gtk_paned_pack2(GTK_PANED(paned), scrolledwindowConsole, FALSE, FALSE);
        gtk_paned_set_position(GTK_PANED(paned), initialSizeFactor * ctx.da.wld_height);

        gtk_container_add(GTK_CONTAINER(ctx.window), paned);
    }

    gtk_widget_add_events(ctx.da.widget, GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_SCROLL_MASK | GDK_KEY_PRESS_MASK | GDK_BUTTON1_MOTION_MASK);
    g_signal_connect(G_OBJECT(ctx.da.widget), "draw", G_CALLBACK(on_draw_event), &ctx);
    g_signal_connect(G_OBJECT(ctx.da.widget), "button-press-event", G_CALLBACK(on_event), &ctx);
    g_signal_connect(G_OBJECT(ctx.da.widget), "button-release-event", G_CALLBACK(on_event), &ctx);
    g_signal_connect(G_OBJECT(ctx.window), "key-press-event", G_CALLBACK(on_event), &ctx);
    g_signal_connect(G_OBJECT(ctx.da.widget), "scroll-event", G_CALLBACK(on_event), &ctx);
    g_signal_connect(G_OBJECT(ctx.da.widget), "motion-notify-event", G_CALLBACK(on_event), &ctx);
    g_signal_connect(ctx.window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_window_set_position(GTK_WINDOW(ctx.window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size(GTK_WINDOW(ctx.window), initialSizeFactor * ctx.da.wld_width, (initialSizeFactor + 0.5) * ctx.da.wld_height);
    gtk_window_set_title(GTK_WINDOW(ctx.window), "GTK window");

    gtk_widget_show_all(ctx.window);

    g_timeout_add(1000 / 25, (GSourceFunc) on_invalidate, &ctx); // 25Hz

    gtk_main();

    return 0;
}
