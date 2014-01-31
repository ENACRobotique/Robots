#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <math.h>

#include <glib.h>
#include <gtk/gtk.h>

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../../global_errors.h"
#include "node_cfg.h"
extern int serial_port;

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
        if(ret > 0 && ctx->verbose >= 1){
            printf("message received from %03hx, type : %s (%hhu)\n", inMsg.header.srcAddr, eType2str(inMsg.header.type), inMsg.header.type);

            if(ctx->verbose >= 2){
                switch(inMsg.header.type){
                case E_DEBUG:
                    printf("  %s\n", inMsg.payload.debug);
                    break;
                case E_POS:
                    printf("  robot%hhu@(%fcm,%fcm,%f°)\n", inMsg.payload.pos.id, inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta*180./M_PI);
                    break;
                default:
                    break;
                }
            }
        }
    }

    return TRUE; // handled
}

int main(int argc, char *argv[]) {
    context_t ctx;

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

    gtk_main();

    // never reached
    return 0;
}

