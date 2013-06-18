#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <getopt.h>
#include <math.h>

#include <glib.h>
#include <gtk/gtk.h>

#include "messages.h"
#include "lib_checksum.h"
#include "network_cfg.h"
#include "lib_Xbee_x86.h"

#include "context.h"

void usage(char *cl) {
    printf("Command Line Interface\n");
    printf("Usage:\n\t%s (--debug | --remote-ia) [--device=<devname>] [--verbose] [--help]\n", cl);
    printf("Help:\n");
    printf("\t--debug, -d               catch messages adressed to ADDRX_DEBUG\n");
    printf("\t--remote-ia, -i           catch messages adressed to ADDRX_REMOTE_IA\n");
    printf("\t--device=<dev>, -D <dev>  specify the device you want to listen from\n\t                          (may be -, in this case, reading from 0 and writing to 2)\n");
    printf("\t--verbose, -v             increases verbosity\n");
    printf("\t--help, -h, -?            prints this help\n");
}

int handle(GIOChannel *source, GIOCondition condition, context_t *ctx) {
    sMsg inMsg;
    int bytesCount;

    if((bytesCount = Xbee_receive(&inMsg)) > 0 && ((inMsg.header.destAddr&SUBNET_MASK)==SUBNETX) && (inMsg.header.destAddr&ctx->addrmask)) {
        switch(inMsg.header.destAddr) {
        case ADDRX_REMOTE_IA:
            printf("[RIA] ");
            break;
        case ADDRX_DEBUG:
            printf("[DBG] ");
            break;
        case ADDRX_BROADCAST:
            printf("[BDC] ");
            break;
        default:
            break;
        }

        printf("received %i bytes from %04x: type %s\n", bytesCount, inMsg.header.srcAddr, eType2str(inMsg.header.type));

        if(ctx->verbose > 0) {
            switch(inMsg.header.type) {
            case E_PERIOD:
                printf("  %u\n", inMsg.payload.period);
                break;
            case E_DEBUG:
                printf("  %s,%i,%u\n", inMsg.payload.debug.msg, inMsg.payload.debug.i, inMsg.payload.debug.u);
                break;
            case E_DATA:
                printf("  %s\n", inMsg.payload.raw);
                break;
            case E_POS:
                printf("  robot id%i @ (%.2fcm, %.2fcm, %.2f°)\n", inMsg.payload.pos.id, inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta*180./M_PI);
                break;
            default:
                break;
            }
        }
    }

    return TRUE; // handled
}

int main(int argc, char *argv[]) {
    context_t ctx;

    // arguments options
    strncpy(ctx.devname, "/dev/ttyUSB0", sizeof(ctx.devname));
    ctx.verbose = 0;
    ctx.addrmask = 0;
    while(1) {
        static struct option long_options[] = {
            {"device",    required_argument, NULL, 'D'},
            {"debug",     no_argument,       NULL, 'd'},
            {"remote-ia", no_argument,       NULL, 'i'},
            {"verbose",   no_argument,       NULL, 'v'},
            {"help",      no_argument,       NULL, 'h'},
            {NULL,        0,                 NULL, 0}
        };

        int c = getopt_long(argc, argv, "D:divh?", long_options, NULL);
        if(c == -1)
            break;

        switch(c) {
        case 'D':
            strncpy(ctx.devname, optarg, sizeof(ctx.devname));
            break;

        case 'd':
            ctx.addrmask |= ADDRX_DEBUG;
            break;

        case 'i':
            ctx.addrmask |= ADDRX_REMOTE_IA;
            break;

        case 'v':
            ctx.verbose++;
            break;

        default:
           printf("?? getopt returned character code 0%o ??\n", c);
        case '?':
        case 'h':
            usage(argv[0]);
            exit(EXIT_FAILURE);
            break;
        }
    }

    // arguments check
    ctx.addrmask &= ~SUBNET_MASK;
    if(!ctx.addrmask) {
        usage(argv[0]);
        exit(EXIT_FAILURE);
    }

    printf("Listening to %s messages on device %s.\n", ctx.addrmask&ADDRX_REMOTE_IA?(ctx.addrmask&ADDRX_DEBUG?"REMOTE_IA and DEBUG":"REMOTE_IA"):"DEBUG", ctx.devname);

    Xbee_initSerial(ctx.devname);

    { // add channel input watch
        GIOChannel *ch = g_io_channel_unix_new(Xbee_serial_port);
        g_io_channel_set_encoding(ch, NULL, NULL);  // this is binary data
        g_io_add_watch(ch, G_IO_IN /*| G_IO_PRI*/, (GIOFunc)handle, &ctx);
        g_io_channel_unref(ch);
    }

    gtk_main();

    // never reached
    return 0;
}

