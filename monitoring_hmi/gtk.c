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

void usage(char *cl) {
    printf("GTK UI\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--verbose, -v             increases verbosity\n");
    printf("\t--quiet, -q               not verbose\n");
    printf("\t--help, -h, -?            prints this help\n");
}

int handle(GIOChannel *source, GIOCondition condition, context_t *ctx) {
    sMsg inMsg;
    int ret;

    ret = bn_receive(&inMsg);
    if(ret > 0){
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

                if(ctx->verbose >= 2){
                    printf("  robot%hhu@(%fcm,%fcm,%f°)\n", inMsg.payload.pos.id, inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta*180./M_PI);
                }

                factor_x = (float)ctx->pos_szx/300.;
                factor_y = (float)ctx->pos_szy/200.;

                x = (int)(inMsg.payload.pos.x*factor_x + 0.5);
                y = ctx->pos_szy - (int)(inMsg.payload.pos.y*factor_y + 0.5);
                dx = 8*factor_x*cos(inMsg.payload.pos.theta);
                dy = -8*factor_y*sin(inMsg.payload.pos.theta);

                // update position media
                ctx->pos_cur ^= 1;
                memset(ctx->pos_data[ctx->pos_cur], 255, ctx->pos_szx*ctx->pos_szy*3);
                video_draw_arrow(ctx->pos_data[ctx->pos_cur], ctx->pos_szx, ctx->pos_szy, ctx->pos_szx*3, x, y, dx, dy, 10, 255, 0, 0);
                gv_media_update(ctx->pos_mid, ctx->pos_data[ctx->pos_cur], ctx->pos_szx, ctx->pos_szy, (gv_destroy)NULL, NULL);

                break;
            }
            default:
                break;
            }
        }
    }

    return TRUE; // handled
}

int main(int argc, char *argv[]) {
    context_t ctx;

    memset(&ctx, 0, sizeof(ctx));

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
    ctx.pos_mid = gv_media_new("position", "Position des robots en temps réel", ctx.pos_szx, ctx.pos_szy);
    ctx.pos_data[0] = malloc(ctx.pos_szx*ctx.pos_szy*3);
    ctx.pos_data[1] = malloc(ctx.pos_szx*ctx.pos_szy*3);
    ctx.pos_cur = 0;

    bn_init();
    if(ctx.verbose >= 1){
        printf("Identified as ADDRX_DEBUG on bn network.");
    }

    { // add channel input watch
        GIOChannel *ch = g_io_channel_unix_new(serial_port);
        g_io_channel_set_encoding(ch, NULL, NULL);  // this is binary data
        g_io_add_watch(ch, G_IO_IN /*| G_IO_PRI*/, (GIOFunc)handle, &ctx);
        g_io_channel_unref(ch);
    }

    printf("(Listening...)\n");

    // run main loop
    gv_run();

    // never reached
    return 0;
}

