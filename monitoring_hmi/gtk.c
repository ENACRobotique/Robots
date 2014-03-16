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
#include "roles.h"
#include "node_cfg.h"

#include "gv.h"
#include "video_draw.h"

#include "context.h"
#include "draw_list.h"

#define R_ROBOT (15.)
// real number
typedef float sNum_t;
// 2D point
typedef struct {
    sNum_t x;
    sNum_t y;
} sPt_t;
// an obstacle
typedef struct {
    sPt_t c;    // center of obstacle
    sNum_t r;   // radius

    uint8_t moved:4;
    uint8_t active:4;
} sObs_t;   // sizeof(sObs_t)=16

sObs_t obs[] = {
	// départ
		{{1., 199.}, 0., 1, 1},
	// arbres
		{{0.  , 70. }, R_ROBOT+15, 1, 1},
		{{70. , 0.  }, R_ROBOT+15, 1, 1},
		{{230., 0.  }, R_ROBOT+15, 1, 1},
		{{300., 70. }, R_ROBOT+15, 1, 1},
	// bac fruit
		{{75. , 185.}, R_ROBOT+15, 1, 1},
		{{42. , 185.}, R_ROBOT+4, 1, 1},
		{{108. ,185.}, R_ROBOT+4, 1, 1},
		{{42. , 172.}, R_ROBOT+4, 1, 1},
		{{108., 172.}, R_ROBOT+4, 1, 1},

		{{225., 185.}, R_ROBOT+15, 1, 1},
		{{192., 185.}, R_ROBOT+4, 1, 1},
		{{258., 185.}, R_ROBOT+4, 1, 1},
		{{192., 172.}, R_ROBOT+4, 1, 1},
		{{258., 172.}, R_ROBOT+4, 1, 1},
	// foyers
		{{0.  , 0.  }, R_ROBOT+25, 1, 1},
		{{300., 0.  }, R_ROBOT+25, 1, 1},
		{{150., 95. }, R_ROBOT+15, 1, 1},
	// torches mobile
		{{90. , 90. }, R_ROBOT+8, 1, 1},
		{{210., 90. }, R_ROBOT+8, 1, 1},
	// torches fixe
		{{1.  , 126}, R_ROBOT+2, 1, 1},
		{{1.  , 114}, R_ROBOT+2, 1, 1},

		{{136, 1.  }, R_ROBOT+2, 1, 1},
		{{124, 1.  }, R_ROBOT+2, 1, 1},

		{{176., 1. }, R_ROBOT+2, 1, 1},
		{{164., 1. }, R_ROBOT+2, 1, 1},

		{{299.,126.}, R_ROBOT+2, 1, 1},
		{{299.,114.}, R_ROBOT+2, 1, 1},
	// feux
		{{40. ,  90. }, R_ROBOT+2, 1, 1},
		{{43.4 , 90. }, R_ROBOT+2, 1, 1},
		{{36.6 , 90. }, R_ROBOT+2, 1, 1},

		{{90. , 40.  }, R_ROBOT+2, 1, 1},
		{{90. , 43.4 }, R_ROBOT+2, 1, 1},
		{{90. , 36.6 }, R_ROBOT+2, 1, 1},

		{{90. , 140. }, R_ROBOT+2, 1, 1},
		{{90. , 143.4}, R_ROBOT+2, 1, 1},
		{{90. , 136.6}, R_ROBOT+2, 1, 1},

		{{210., 40.  }, R_ROBOT+2, 1, 1},
		{{210., 43.4 }, R_ROBOT+2, 1, 1},
		{{210., 36.6 }, R_ROBOT+2, 1, 1},

		{{210., 140. }, R_ROBOT+2, 1, 1},
		{{210., 143.4}, R_ROBOT+2, 1, 1},
		{{210., 136.6}, R_ROBOT+2, 1, 1},

		{{260.,  90.}, R_ROBOT+2, 1, 1},
		{{263.4, 90.}, R_ROBOT+2, 1, 1},
		{{256.6, 90.}, R_ROBOT+2, 1, 1},

		{{1.  , 120. }, R_ROBOT+2, 1, 1},
		{{1.  , 123.4}, R_ROBOT+2, 1, 1},
		{{1.  , 116.6}, R_ROBOT+2, 1, 1},

		{{130. , 1.  }, R_ROBOT+2, 1, 1},
		{{133.4, 1.  }, R_ROBOT+2, 1, 1},
		{{126.6, 1.  }, R_ROBOT+2, 1, 1},

		{{170. , 1.  }, R_ROBOT+2, 1, 1},
		{{173.4, 1.  }, R_ROBOT+2, 1, 1},
		{{166.6, 1.  }, R_ROBOT+2, 1, 1},

		{{299., 120. }, R_ROBOT+2, 1, 1},
		{{299., 123.4}, R_ROBOT+2, 1, 1},
		{{299., 116.6}, R_ROBOT+2, 1, 1},

		{{90. , 94.  }, R_ROBOT+4, 1, 1},
		{{93.4, 88.  }, R_ROBOT+4, 1, 1},
		{{86.6, 88.  }, R_ROBOT+4, 1, 1},

		{{90. , 94.  }, R_ROBOT+4, 1, 1},
		{{93.4, 88.  }, R_ROBOT+4, 1, 1},
		{{86.6, 88.  }, R_ROBOT+4, 1, 1},

		{{90. , 94.  }, R_ROBOT+4, 1, 1},
		{{93.4, 88.  }, R_ROBOT+4, 1, 1},
		{{86.6, 88.  }, R_ROBOT+4, 1, 1},

		{{210. ,94. }, R_ROBOT+4, 1, 1},
		{{213.4,88. }, R_ROBOT+4, 1, 1},
		{{206.6,88. }, R_ROBOT+4, 1, 1},

		{{210. ,94. }, R_ROBOT+4, 1, 1},
		{{213.4,88. }, R_ROBOT+4, 1, 1},
		{{206.6,88. }, R_ROBOT+4, 1, 1},

		{{210. ,94. }, R_ROBOT+4, 1, 1},
		{{213.4,88. }, R_ROBOT+4, 1, 1},
		{{206.6,88. }, R_ROBOT+4, 1, 1},
	// arrivée
		{{225.,170.},0,1,1}
	};


void usage(char *cl) {
    printf("GTK UI\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--prop-simulator, -s      uses prop simulator\n");
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

uint8_t nb_obss = 0;
struct{
    float x;
    float y;
    float r;

    uint8_t moved:4;
    uint8_t active:4;
} *obss = NULL;
float r_robot = -1.;
float curr_x = 0., curr_y = 0., curr_theta = 0.;

int handle(GIOChannel *source, GIOCondition condition, context_t *ctx) {
#define X_CM2PX(x) CTX_X_CM2PX(ctx, x)
#define Y_CM2PX(y) CTX_Y_CM2PX(ctx, y)
#define DX_CM2PX(dx) CTX_DX_CM2PX(ctx, dx)
#define DY_CM2PX(dy) CTX_DY_CM2PX(ctx, dy)
#define CM2PX(r) CTX_CM2PX(ctx, r)
#define WIDTH() CTX_WIDTH(ctx)
#define ROWSTRIDE() CTX_ROWSTRIDE(ctx)
#define HEIGHT() CTX_HEIGHT(ctx)
#define BUFSZ() CTX_BUFSZ(ctx)
#define X_PX2CM(x) CTX_X_PX2CM(ctx, x)
#define Y_PX2CM(y) CTX_Y_PX2CM(ctx, y)


    sMsg inMsg, outMsg = {{0}};
    int ret, i;
    uint8_t update_media = 0;

    ret = bn_receive(&inMsg);
    if(ret > 0){
        ret = role_relay(&inMsg);
        if(ret < 0){
            fprintf(stderr, "role_relay() error #%i\n", ret);
        }

        if(ctx->verbose >= 1){
            if(inMsg.header.type != E_POS)
            printf("message received from %s (%03hx), type : %s (%hhu)\n", role_string(role_get_role(inMsg.header.srcAddr)), inMsg.header.srcAddr, eType2str(inMsg.header.type), inMsg.header.type);
        }

        switch(inMsg.header.type){
        case E_DEBUG:
            if(ctx->verbose >= 2){
                printf("  %s\n", inMsg.payload.debug);
            }
            break;
        case E_OBS_CFG:
            nb_obss = inMsg.payload.obsCfg.nb_obs;
            obss = (typeof(obss))calloc(inMsg.payload.obsCfg.nb_obs, sizeof(*obss));
            r_robot = inMsg.payload.obsCfg.r_robot;

            bn_printfDbg("received obs cfg, %hhuobss\n", nb_obss);
            break;
        case E_OBSS:
            for(i = 0; i < inMsg.payload.obss.nb_obs; i++){
                if(inMsg.payload.obss.obs[i].id < nb_obss){
                    obss[inMsg.payload.obss.obs[i].id].active = inMsg.payload.obss.obs[i].active;
                    obss[inMsg.payload.obss.obs[i].id].moved = inMsg.payload.obss.obs[i].moved;
                    obss[inMsg.payload.obss.obs[i].id].x = (float)inMsg.payload.obss.obs[i].x/100.;
                    obss[inMsg.payload.obss.obs[i].id].y = (float)inMsg.payload.obss.obs[i].y/100.;
                    obss[inMsg.payload.obss.obs[i].id].r = (float)inMsg.payload.obss.obs[i].r/100.;
                }
            }

            if(inMsg.payload.obss.nb_obs > 0){
                bn_printfDbg("received obss, %hhuobss (first %hhu)\n", inMsg.payload.obss.nb_obs, inMsg.payload.obss.obs[0].id);
            }

            update_media = 1;
            break;
        case E_TRAJ:
            tl_addTail(&ctx->trajlist, &inMsg.payload.traj);

            update_media = 1;
            break;
        case E_POS:{
            if(ctx->verbose >= 2){
                printf("  robot%hhu@(%fcm,%fcm,%f°)\n", inMsg.payload.pos.id, inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta*180./M_PI);
            }

            curr_x = inMsg.payload.pos.x;
            curr_y = inMsg.payload.pos.y;
            curr_theta = inMsg.payload.pos.theta;

            // updates list of obstacles
            if(obss){
                obss[0].x = curr_x;
                obss[0].y = curr_y;
                obss[0].r = 0.;
                obss[0].moved = 1;
            }

            // adds position to list of positions to draw
            pl_addTail(&ctx->poslist, &inMsg.payload.pos);

            // send trajectory if event
            if(ctx->mouse_event){
                outMsg.header.destAddr = role_get_addr(ROLE_IA);
                outMsg.header.type = E_GOAL;
                outMsg.header.size = sizeof(outMsg.payload.pos);

                outMsg.payload.pos.id = 0; // main
                outMsg.payload.pos.x = X_PX2CM(ctx->mouse_x);
                outMsg.payload.pos.y = Y_PX2CM(ctx->mouse_y);

                // updates list of obstacles
                if(obss){
                    obss[nb_obss - 1].x = outMsg.payload.pos.x;
                    obss[nb_obss - 1].y = outMsg.payload.pos.y;
                    obss[nb_obss - 1].r = 0.;
                    obss[nb_obss - 1].moved = 1;
                }

                ret = bn_sendAck(&outMsg);
                if(ret <= 0){
                    fprintf(stderr, "bn_sendAck(E_GOAL) error #%i\n", -ret);
                }

                ctx->mouse_event = 0;
            }

            update_media = 1;
            break;
        }
        default:
            break;
        }
    }

    // update position media
    if(update_media){
        int x, y, dx, dy, r, x1, y1, x2, y2;
        int i;
        sPosLEl *pel;
        sTrajLEl *tel;

        // draw background
        ctx->pos_cur ^= 1;
        {
            unsigned char *p = &ctx->pos_data[ctx->pos_cur][0];
            for(i = 0; i < BUFSZ(); i+=3){
                *p++ = 0;
                *p++ = 127;
                *p++ = 0;
            }
        }

        // draw inactive obstacles
        for(i = 0; obss && i < nb_obss; i++){
            x = X_CM2PX(obss[i].x);
            y = Y_CM2PX(obss[i].y);
            if(obss[i].r > r_robot){
                r = CM2PX(obss[i].r - r_robot);
            }
            else{
                r = CM2PX(obss[i].r);
            }

            if(!obss[i].active){
                if(r > 0){
                    video_draw_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, BLACK(), 255);
                }
                else{
                    video_draw_cross(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, CM2PX(8.), BLACK(), 255);
                }
            }
        }
        // draw active obstacles
        for(i = 0; obss && i < nb_obss; i++){
            x = X_CM2PX(obss[i].x);
            y = Y_CM2PX(obss[i].y);
            r = CM2PX(obss[i].r);

            if(obss[i].active && r > 0){
                video_draw_filled_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, ORANGE(255), 255);
            }
        }
        for(i = 0; obss && i < nb_obss; i++){
            x = X_CM2PX(obss[i].x);
            y = Y_CM2PX(obss[i].y);
            if(obss[i].r > r_robot){
                r = CM2PX(obss[i].r - r_robot);
            }
            else{
                r = CM2PX(obss[i].r);
            }

            if(obss[i].active){
                if(r > 0){
                    video_draw_filled_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, RED(255), 255);
                }
                else{
                    video_draw_cross(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, CM2PX(8.), RED(255), 255);
                }
            }
        }

        // draw trajectories
        for(tel = tl_getFirst(&ctx->trajlist); tel; tel = tl_getNext(&ctx->trajlist)){
            // segment
            x1 = X_CM2PX(tel->traj.p1_x);
            y1 = Y_CM2PX(tel->traj.p1_y);
            x2 = X_CM2PX(tel->traj.p2_x);
            y2 = Y_CM2PX(tel->traj.p2_y);

            video_draw_line(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x1, y1, x2, y2, BLACK(), 255);

            // arc
            x = X_CM2PX(tel->traj.c_x);
            y = Y_CM2PX(tel->traj.c_y);
            r = CM2PX(fabs(tel->traj.c_r));

            if(tel->next && tel->traj.tid == tel->next->traj.tid && tel->traj.sid + 1 == tel->next->traj.sid){
                if(r){
                    x1 = X_CM2PX(tel->next->traj.p1_x);
                    y1 = Y_CM2PX(tel->next->traj.p1_y);

                    if(tel->traj.c_r > 0.){
                        video_draw_arc(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, x2, y2, x1, y1, BLACK(), 255);
                    }
                    else{
                        video_draw_arc(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, x1, y1, x2, y2, BLACK(), 255);
                    }
                }
                else{
                    video_draw_pixel(ctx->pos_data[ctx->pos_cur], ROWSTRIDE(), HEIGHT(), x, y, BLACK(), 255);
                }
            }
            else{
                video_draw_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, BLACK(), 255);
            }
        }

        // draw previous positions
        r = CM2PX(3.);
        for(i = 0, pel = pl_getFirst(&ctx->poslist); pel && pel->next; i++, pel = pl_getNext(&ctx->poslist)){
            x = X_CM2PX(pel->pos.x);
            y = Y_CM2PX(pel->pos.y);

            video_draw_pixel(ctx->pos_data[ctx->pos_cur], ROWSTRIDE(), HEIGHT(), x, y, BLUE(255), 255*(i + ctx->poslist.maxlen - ctx->poslist.len)/ctx->poslist.maxlen);
            video_draw_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, BLUE(255), 255*(i + ctx->poslist.maxlen - ctx->poslist.len)/ctx->poslist.maxlen);
        }

        // draw current position
        x = X_CM2PX(curr_x);
        y = Y_CM2PX(curr_y);
        r = CM2PX(r_robot);
        dx = DX_CM2PX(r_robot*cos(curr_theta));
        dy = DY_CM2PX(r_robot*sin(curr_theta));

        video_draw_filled_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, GREY(150), 150);
        video_draw_arrow(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, dx, dy, 10, BLACK(), 255);

        // switch drawing buffer in gui
        gv_media_update(ctx->pos_mid, ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), (gv_destroy)NULL, NULL);
    }

    return TRUE; // handled

#undef X_CM2PX
#undef Y_CM2PX
#undef DX_CM2PX
#undef DY_CM2PX
#undef CM2PX
#undef WIDTH
#undef ROWSTRIDE
#undef HEIGHT
#undef BUFSZ
#undef X_PX2CM
#undef Y_PX2CM
}

int main(int argc, char *argv[]) {
    context_t ctx;
    int ret;

    memset(&ctx, 0, sizeof(ctx));
    pl_init(&ctx.poslist, 100);
    tl_init(&ctx.trajlist, 2);

    // arguments options
    ctx.verbose = 1;
    ctx.prop_address = ADDRI_MAIN_PROP;
    while(1) {
        static struct option long_options[] = {
            {"prop-simulator", no_argument,  NULL, 's'},
            {"verbose",   no_argument,       NULL, 'v'},
            {"quiet",     no_argument,       NULL, 'q'},
            {"help",      no_argument,       NULL, 'h'},
            {NULL,        0,                 NULL, 0}
        };

        int c = getopt_long(argc, argv, "svqh?", long_options, NULL);
        if(c == -1)
            break;

        switch(c) {
        case 's':
            ctx.prop_address = ADDRD_MAIN_PROP_SIMU;
            break;
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
    ctx.pos_maxx = 300.;
    ctx.pos_maxy = 200.;
    ctx.pos_wfactor = 4;
    ctx.pos_mid = gv_media_new("position", "Position des robots en temps réel", CTX_WIDTH(&ctx), CTX_HEIGHT(&ctx), GTK_SIGNAL_FUNC(event_cb), &ctx);
    ctx.pos_data[0] = malloc(CTX_BUFSZ(&ctx));
    ctx.pos_data[1] = malloc(CTX_BUFSZ(&ctx));
    ctx.pos_cur = 0;

    bn_attach(E_ROLE_SETUP, role_setup);

    ret = bn_init();
    if(ret < 0){
        printf("bn_init failed err#%i\n", -ret);
        exit(1);
    }

    if(ctx.verbose >= 1){
        printf("Identified as ADDRX_DEBUG on bn network.\n");
    }

#if MYADDRX
    { // add channel input watch
        extern int serial_port;

        GIOChannel *ch = g_io_channel_unix_new(serial_port);
        g_io_channel_set_encoding(ch, NULL, NULL);  // this is binary data
        g_io_add_watch(ch, G_IO_IN /*| G_IO_PRI*/, (GIOFunc)handle, &ctx);
        g_io_channel_unref(ch);
    }
#endif
#if MYADDRD
    { // add channel input watch
        extern int udpsockfd; // file descriptor

        GIOChannel *ch = g_io_channel_unix_new(udpsockfd);
        g_io_channel_set_encoding(ch, NULL, NULL);  // this is binary data
        g_io_add_watch(ch, G_IO_IN /*| G_IO_PRI*/, (GIOFunc)handle, &ctx);
        g_io_channel_unref(ch);
    }
#endif

    printf("Listening...\n");

    {
        sMsg outMsg = {{0}};
// send initial position
        outMsg.header.type = E_POS;
        outMsg.header.size = sizeof(outMsg.payload.pos);

        outMsg.payload.pos.id = 0;
        outMsg.payload.pos.theta = M_PI;
        outMsg.payload.pos.u_a = 0;
        outMsg.payload.pos.u_a_theta = 0;
        outMsg.payload.pos.u_b = 0;
        outMsg.payload.pos.x = 294.;
        outMsg.payload.pos.y = 57.;
        if(ctx.verbose >= 1){
            printf("Sending initial position to robot%i (%fcm,%fcm,%f°).\n", outMsg.payload.pos.id, outMsg.payload.pos.x, outMsg.payload.pos.y, outMsg.payload.pos.theta*180./M_PI);
        }
        ret = role_send(&outMsg);
        if(ret <= 0){
            fprintf(stderr, "role_send() error #%i\n", -ret);
        }

// ask obstacles
        outMsg.header.destAddr = role_get_addr(ROLE_IA);
        outMsg.header.type = E_OBS_CFG;
        outMsg.header.size = sizeof(outMsg.payload.obsCfg);

        outMsg.payload.obsCfg.nb_obs = 0;
        outMsg.payload.obsCfg.r_robot = 0.;

        ret = bn_send(&outMsg);
        if(ret < 0){
            fprintf(stderr, "bn_send(E_OBS_CFG) error #%i\n", -ret);
        }
    }

    // run main loop
    gv_run();

    // never reached
    return 0;
}
