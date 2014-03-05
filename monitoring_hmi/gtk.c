#include <stdio.h>
#include <stdlib.h>
#include <string.h> // memset
#include <getopt.h> // parameters parsing
#include <math.h> // M_PI

#include <glib.h>
#include <gtk/gtk.h>

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../../global_errors.h"
#include "node_cfg.h"
extern int serial_port;

#include "gv.h"
#include "video_draw.h"

#include "context.h"
#include "draw_list.h"

void usage(char *cl) {
    printf("GTK UI\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--verbose, -v             increases verbosity\n");
    printf("\t--quiet, -q               not verbose\n");
    printf("\t--help, -h, -?            prints this help\n");
}

gint event_cb(GtkWidget *widget, GdkEvent *event, context_t *ctx) {
    GdkEventButton *bevent = (GdkEventButton *)event;

//    printf("event%i\n", (int)event->type);

    switch ((gint)event->type) {
    case GDK_BUTTON_PRESS:
        /* Handle mouse button press */

//        printf("Mouse click %g,%g\n", bevent->x, bevent->y);

        ctx->mouse_event = 1;
        ctx->mouse_x = (int)bevent->x;
        ctx->mouse_y = (int)bevent->y;

        return TRUE;
    }

    /* Event not handled; try parent item */
    return FALSE;
}

int handle(GIOChannel *source, GIOCondition condition, context_t *ctx) {
    sMsg inMsg, outMsg;
    int ret;

    ret = bn_receive(&inMsg);
    if(ret > 0){
        if(ctx->verbose >= 1){
            printf("message received from %03hx, type : %s (%hhu)\n", inMsg.header.srcAddr, eType2str(inMsg.header.type), inMsg.header.type);
        }

        switch(inMsg.header.type){
        case E_DEBUG:
            if(ctx->verbose >= 2){
                printf("  %s\n", inMsg.payload.debug);
            }
            break;
        case E_POS:{
            int x, y, dx, dy;
            float factor_x, factor_y;
            sPosLEl *pel;
            sTrajLEl *tel;

            if(ctx->verbose >= 2){
                printf("  robot%hhu@(%fcm,%fcm,%f°)\n", inMsg.payload.pos.id, inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta*180./M_PI);
            }

            // adds position to list of positions to draw
            pl_addTail(&ctx->poslist, &inMsg.payload.pos);

            factor_x = (float)ctx->pos_szx/300.;
            factor_y = (float)ctx->pos_szy/200.;

            // send trajectory if event
            if(ctx->mouse_event){
                outMsg.header.destAddr = ADDRI_MAIN_PROP;
                outMsg.header.type = E_TRAJ;
                outMsg.header.size = sizeof(outMsg.payload.traj);
                // payload
                outMsg.payload.traj.p1_x = inMsg.payload.pos.x;
                outMsg.payload.traj.p1_y = inMsg.payload.pos.y;
                outMsg.payload.traj.p2_x = (float)ctx->mouse_x/factor_x;
                outMsg.payload.traj.p2_y = 200. - (float)ctx->mouse_y/factor_y;
                outMsg.payload.traj.seg_len = sqrt((outMsg.payload.traj.p1_x - outMsg.payload.traj.p2_x)*(outMsg.payload.traj.p1_x - outMsg.payload.traj.p2_x) + (outMsg.payload.traj.p1_y - outMsg.payload.traj.p2_y)*(outMsg.payload.traj.p1_y - outMsg.payload.traj.p2_y));
                outMsg.payload.traj.c_x = outMsg.payload.traj.p2_x;
                outMsg.payload.traj.c_y = outMsg.payload.traj.p2_y;
                outMsg.payload.traj.c_r = 0.;
                outMsg.payload.traj.arc_len = 0.;
                outMsg.payload.traj.tid = ctx->tid++;
                outMsg.payload.traj.sid = 0;

                tl_addTail(&ctx->trajlist, &outMsg.payload.traj);

                ret = bn_send(&outMsg);
                if(ret <= 0){
                    fprintf(stderr, "bn_send() error #%i\r\n", -ret); // '\r' needed, raw mode
                }

                ctx->mouse_event = 0;
            }

            // update position media
            ctx->pos_cur ^= 1;
            memset(ctx->pos_data[ctx->pos_cur], 255, ctx->pos_szx*ctx->pos_szy*3);

            // draw current position
            x = (int)(inMsg.payload.pos.x*factor_x + 0.5);
            y = ctx->pos_szy - (int)(inMsg.payload.pos.y*factor_y + 0.5);
            dx = 8*factor_x*cos(inMsg.payload.pos.theta);
            dy = -8*factor_y*sin(inMsg.payload.pos.theta);

            video_draw_arrow(ctx->pos_data[ctx->pos_cur], ctx->pos_szx, ctx->pos_szy, ctx->pos_szx*3, x, y, dx, dy, 10, 255, 0, 0);

            // draw trajectories
            for(tel = tl_getFirst(&ctx->trajlist); tel; tel = tl_getNext(&ctx->trajlist)){
                x = (int)(tel->traj.p1_x*factor_x + 0.5);
                y = ctx->pos_szy - (int)(tel->traj.p1_y*factor_y + 0.5);
                dx = (int)(tel->traj.p2_x*factor_x + 0.5) - x;
                dy = ctx->pos_szy - (int)(tel->traj.p2_y*factor_y + 0.5) - y;

                video_draw_line(ctx->pos_data[ctx->pos_cur], ctx->pos_szx, ctx->pos_szy, ctx->pos_szx*3, x, y, x+dx, y+dy, 0, 0, 0);

                x = (int)(tel->traj.c_x*factor_x + 0.5);
                y = ctx->pos_szy - (int)(tel->traj.c_y*factor_y + 0.5);
                dx = (int)(tel->traj.c_r*factor_x + 0.5);
                dy = (int)(tel->traj.c_r*factor_y + 0.5);

                video_draw_pixel(ctx->pos_data[ctx->pos_cur], ctx->pos_szx*3, ctx->pos_szy, x, y, 0, 0, 0);
                video_draw_circle(ctx->pos_data[ctx->pos_cur], ctx->pos_szx, ctx->pos_szy, ctx->pos_szx*3, x, y, (dx+dy)/2, 0, 0, 0);
            }

            // draw previous positions
            for(pel = pl_getFirst(&ctx->poslist); pel && pel->next; pel = pl_getNext(&ctx->poslist)){
                x = (int)(pel->pos.x*factor_x + 0.5);
                y = ctx->pos_szy - (int)(pel->pos.y*factor_y + 0.5);
                dx = 3*factor_x;
                dy = 3*factor_y;

//                video_draw_cross(ctx->pos_data[ctx->pos_cur], ctx->pos_szx, ctx->pos_szy, ctx->pos_szx*3, x, y, (dx+dy)/2, 0, 0, 255);
                video_draw_pixel(ctx->pos_data[ctx->pos_cur], ctx->pos_szx*3, ctx->pos_szy, x, y, 0, 0, 255);
                video_draw_circle(ctx->pos_data[ctx->pos_cur], ctx->pos_szx, ctx->pos_szy, ctx->pos_szx*3, x, y, (dx+dy)/2, 0, 0, 255);
            }

            // switch drawing buffer in gui
            gv_media_update(ctx->pos_mid, ctx->pos_data[ctx->pos_cur], ctx->pos_szx, ctx->pos_szy, (gv_destroy)NULL, NULL);

            break;
        }
        default:
            break;
        }
    }

    return TRUE; // handled
}

int main(int argc, char *argv[]) {
    context_t ctx;
    int ret;

    memset(&ctx, 0, sizeof(ctx));
    pl_init(&ctx.poslist, -1);
    tl_init(&ctx.trajlist, -1);

    // arguments options
    ctx.verbose = 1;
    while(1) {
        static struct option long_options[] = {
            {"verbose",   no_argument,       NULL, 'v'},
            {"quiet",     no_argument,       NULL, 'q'},
            {"help",      no_argument,       NULL, 'h'},
            {NULL,        0,                 NULL, 0}
        };

        int c = getopt_long(argc, argv, "vqh?", long_options, NULL);
        if(c == -1)
            break;

        switch(c) {
        case 'v':
            ctx.verbose++;
            break;
        case 'q':
            ctx.verbose = 0;
            break;

        default:
           printf("?? getopt returned character code 0%o ??\n", c);
           /* no break */
        case '?':
        case 'h':
            usage(argv[0]);
            exit(EXIT_FAILURE);
            break;
        }
    }

    // arguments check

    // gtkviewer init
    gv_init(&argc, &argv, "GTK UI to botNet", "En construction...");
    // gtkviewer media creation
    ctx.pos_szx = 300*4;
    ctx.pos_szy = 200*4;
    ctx.pos_mid = gv_media_new("position", "Position des robots en temps réel", ctx.pos_szx, ctx.pos_szy, GTK_SIGNAL_FUNC(event_cb), &ctx);
    ctx.pos_data[0] = malloc(ctx.pos_szx*ctx.pos_szy*3);
    ctx.pos_data[1] = malloc(ctx.pos_szx*ctx.pos_szy*3);
    ctx.pos_cur = 0;

    ret = bn_init();
    if(ret < 0){
        printf("bn_init failed err#%i\n", -ret);
        exit(1);
    }

    if(ctx.verbose >= 1){
        printf("Identified as ADDRX_DEBUG on bn network.\n");
    }

    { // add channel input watch
        GIOChannel *ch = g_io_channel_unix_new(serial_port);
        g_io_channel_set_encoding(ch, NULL, NULL);  // this is binary data
        g_io_add_watch(ch, G_IO_IN /*| G_IO_PRI*/, (GIOFunc)handle, &ctx);
        g_io_channel_unref(ch);
    }

    printf("(Listening...)\n");

    // send position
    {
        sMsg outMsg;
        outMsg.header.destAddr = ADDRI_MAIN_PROP;
        outMsg.header.type = E_POS;
        outMsg.header.size = sizeof(outMsg.payload.pos);
        outMsg.payload.pos.id = 0;
        outMsg.payload.pos.theta = M_PI;
        outMsg.payload.pos.u_a = 0;
        outMsg.payload.pos.u_a_theta = 0;
        outMsg.payload.pos.u_b = 0;
        outMsg.payload.pos.x = 294;
        outMsg.payload.pos.y = 57;
        if(ctx.verbose >= 1){
            printf("Sending initial position to robot%i (%fcm,%fcm,%f°).\n", outMsg.payload.pos.id, outMsg.payload.pos.x, outMsg.payload.pos.y, outMsg.payload.pos.theta*180./M_PI);
        }
        ret = bn_sendAck(&outMsg);
        if(ret <= 0){
            fprintf(stderr, "bn_sendAck() error #%i\n", -ret);
        }
    }

    // run main loop
    gv_run();

    // never reached
    return 0;
}

