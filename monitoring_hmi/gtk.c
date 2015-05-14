#include <stdio.h>
#include <stdlib.h>
#include <string.h> // memset
#include <getopt.h> // parameters parsing
#include <math.h> // M_PI

#include <glib.h>
#include <gtk/gtk.h>

#include "../botNet/shared/botNet_core.h"
#include "../network_tools/bn_debug.h"
#include "global_errors.h"
#include "roles.h"
#include "node_cfg.h"

#include "gv.h"
#include "video_draw.h"
#include "video_draw_strings.h"

#include "context.h"
#include "draw_list.h"

#define SQR(v) ((long long)(v)*(v))
#define SIGN(v) (((v)>0) - ((v)<0))
#define CONV2TRAJ(a, b) ((double) a/(pow(2,b)))

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
float x_min = 0., x_max = 0., y_min = 0., y_max = 0.;
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
    sTrajElRaw_t te;
    sTrajOrientElRaw_t toe;

    ret = bn_receive(&inMsg);
    if(ret > 0){
        ret = role_relay(&inMsg);
        if(ret < 0){
            fprintf(stderr, "role_relay() error #%i\n", ret);
        }

        if(ctx->verbose >= 1){
            if(inMsg.header.type != E_GENERIC_POS_STATUS)
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
            x_min = inMsg.payload.obsCfg.x_min;
            x_max = inMsg.payload.obsCfg.x_max;
            y_min = inMsg.payload.obsCfg.y_min;
            y_max = inMsg.payload.obsCfg.y_max;

            bn_printfDbg("received obs cfg, %hhuobss\n", nb_obss);
            break;
        case E_OBSS:
            if(!nb_obss){
                outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
                outMsg.header.type = E_OBS_CFG;
                outMsg.header.size = sizeof(outMsg.payload.obsCfg);

                outMsg.payload.obsCfg.nb_obs = 0;
                outMsg.payload.obsCfg.r_robot = 0.;
                outMsg.payload.obsCfg.x_min = 0.;
                outMsg.payload.obsCfg.x_max = 0.;
                outMsg.payload.obsCfg.y_min = 0.;
                outMsg.payload.obsCfg.y_max = 0.;

                ret = bn_send(&outMsg);
                if(ret < 0){
                    fprintf(stderr, "bn_send(E_OBS_CFG) error #%i\n", -ret);
                }

                break;
            }

            for(i = 0; i < inMsg.payload.obss.nb_obs; i++){
                if(inMsg.payload.obss.obs[i].id < nb_obss){
                    obss[inMsg.payload.obss.obs[i].id].active = inMsg.payload.obss.obs[i].active;
                    obss[inMsg.payload.obss.obs[i].id].moved = inMsg.payload.obss.obs[i].moved;
                    obss[inMsg.payload.obss.obs[i].id].x = ((float)inMsg.payload.obss.obs[i].x)/100.;
                    obss[inMsg.payload.obss.obs[i].id].y = ((float)inMsg.payload.obss.obs[i].y)/100.;
                    obss[inMsg.payload.obss.obs[i].id].r = ((float)inMsg.payload.obss.obs[i].r)/100.;
                }
            }

            if(inMsg.payload.obss.nb_obs > 0){
                bn_printfDbg("received obss, %hhuobss (first %hhu)\n", inMsg.payload.obss.nb_obs, inMsg.payload.obss.obs[0].id);
            }

            update_media = 1;
            break;
        case E_TRAJ_ORIENT_EL:
            toe = inMsg.payload.trajOrientEl;

            for(i = 0 ; i < 2 ; i++){
                te.p1_x = CONV2TRAJ(toe.elts[i].p1_x, 6);
                te.p1_y = CONV2TRAJ(toe.elts[i].p1_y, 6);
                te.p2_x = CONV2TRAJ(toe.elts[i].p2_x, 6);
                te.p2_y = CONV2TRAJ(toe.elts[i].p2_y, 6);

                te.c_x = CONV2TRAJ(toe.elts[i].c_x, 6);
                te.c_y = CONV2TRAJ(toe.elts[i].c_y, 6);
                te.c_r = CONV2TRAJ(toe.elts[i].c_r, 5);

                te.seg_len = CONV2TRAJ(toe.elts[i].seg_len, 5);
                te.arc_len = CONV2TRAJ(toe.elts[i].arc_len, 5);

                te.sid = toe.sid*2 + i;
                te.tid = toe.tid;

                tl_addTail(&ctx->trajlist, &te);

                if(toe.elts[i].is_last_element) //end of trajectory
                    break;
            }
            update_media = 1;

            break;
        case E_TRAJ:
            tl_addTail(&ctx->trajlist, &inMsg.payload.traj);

            update_media = 1;
            break;
        case E_GENERIC_POS_STATUS:{
            if(ctx->verbose >= 2){
                printf("  robot%hhu@(%fcm,%fcm,%f°)\n", inMsg.payload.genericPosStatus.id, inMsg.payload.genericPosStatus.pos.x, inMsg.payload.genericPosStatus.pos.y, inMsg.payload.genericPosStatus.pos.theta*180./M_PI);
            }

            curr_x = inMsg.payload.genericPosStatus.pos.x;
            curr_y = inMsg.payload.genericPosStatus.pos.y;
            curr_theta = inMsg.payload.genericPosStatus.pos.theta;

            // updates list of obstacles
            if(obss){
                obss[0].x = curr_x;
                obss[0].y = curr_y;
                obss[0].r = 0.;
                obss[0].moved = 1;
            }

            // adds position to list of positions to draw
            pl_addTail(&ctx->poslist, &inMsg.payload.genericPosStatus);

            // send trajectory if event
            if(ctx->mouse_event){
                memset(&outMsg, 0, sizeof(outMsg));

                outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
                outMsg.header.type = E_GOAL;
                outMsg.header.size = sizeof(outMsg.payload.genericPosStatus);

                outMsg.payload.genericPosStatus.id = ELT_PRIMARY; // main
                outMsg.payload.genericPosStatus.pos.x = X_PX2CM(ctx->mouse_x);
                outMsg.payload.genericPosStatus.pos.y = Y_PX2CM(ctx->mouse_y);
                outMsg.payload.genericPosStatus.pos.theta = 0;

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
        char s_nb[16];

        // draw background
        ctx->pos_cur ^= 1;
        {
            unsigned char *p = &ctx->pos_data[ctx->pos_cur][0];
            for(i = 0; i < BUFSZ(); i+=3){
                *p++ = 0;
                *p++ = 127;
                *p++ = 255;
            }
        }

        //yellow starting
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(0.), Y_CM2PX(80.), X_CM2PX(45.), Y_CM2PX(120.), YELLOW(255), 255);
        video_draw_filled_circle(ctx->pos_data[ctx->pos_cur]   , WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(45.), Y_CM2PX(100.), CM2PX(20.), YELLOW(255), 255);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX( 0.), Y_CM2PX( 40.), X_CM2PX(40.), Y_CM2PX(80.), GREEN(200), 255);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX( 0.), Y_CM2PX(120.), X_CM2PX(40.), Y_CM2PX(160.), GREEN(200), 255);

        //green starting
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX( 300-0.), Y_CM2PX(80.), X_CM2PX(300-45.), Y_CM2PX(120.), GREEN(200), 255);
        video_draw_filled_circle(ctx->pos_data[ctx->pos_cur]   , WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(300-45.), Y_CM2PX(100.), CM2PX(20.), GREEN(200), 255);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX( 300-0.), Y_CM2PX( 40.), X_CM2PX(300-40.), Y_CM2PX(80.), YELLOW(255), 255);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX( 300-0.), Y_CM2PX(120.), X_CM2PX(300-40.), Y_CM2PX(160.), YELLOW(255), 255);


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
        // draw limits
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(0.), Y_CM2PX(0.+x_min), X_CM2PX(x_min), Y_CM2PX(ctx->pos_maxy-x_min), RED(255), 150);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(0.), Y_CM2PX(0.), X_CM2PX(ctx->pos_maxx), Y_CM2PX(y_min), RED(255), 150);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(0.), Y_CM2PX(y_max), X_CM2PX(ctx->pos_maxx), Y_CM2PX(ctx->pos_maxy), RED(255), 150);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(x_max), Y_CM2PX(0.+x_min), X_CM2PX(ctx->pos_maxx), Y_CM2PX(ctx->pos_maxy-x_min), RED(255), 150);

        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(96.7-x_min), Y_CM2PX(142.-x_min), X_CM2PX(203.3+x_min), Y_CM2PX(200.-x_min), RED(255), 150);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(120.-x_min), Y_CM2PX(0.+x_min), X_CM2PX(180.+x_min), Y_CM2PX(10.+x_min), RED(255), 150);

        // draw active obstacles
        for(i = 0; obss && i < nb_obss; i++){
            x = X_CM2PX(obss[i].x);
            y = Y_CM2PX(obss[i].y);
            r = CM2PX(obss[i].r);

            if(obss[i].active && r > 0){
                video_draw_filled_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, GREY(150), 150);
            }
        }
        // draw limits
        if(x_min - r_robot >= 0.){
            video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(0.), Y_CM2PX(0.), X_CM2PX(x_min - r_robot), Y_CM2PX(ctx->pos_maxy), RED(255), 255);
        }
        if(y_min - r_robot >= 0.){
            video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(0.), Y_CM2PX(0.), X_CM2PX(ctx->pos_maxx), Y_CM2PX(y_min - r_robot), RED(255), 255);
        }
        if(y_max + r_robot <= ctx->pos_maxy){
            video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(0.), Y_CM2PX(y_max + r_robot), X_CM2PX(ctx->pos_maxx), Y_CM2PX(ctx->pos_maxy), RED(255), 255);
        }
        if(x_max + r_robot <= ctx->pos_maxx){
            video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(x_max + r_robot), Y_CM2PX(0.), X_CM2PX(ctx->pos_maxx), Y_CM2PX(ctx->pos_maxy), RED(255), 255);
        }

        // draw current position
        x = X_CM2PX(curr_x);
        y = Y_CM2PX(curr_y);
        r = CM2PX(r_robot);
        dx = DX_CM2PX(r_robot*cos(curr_theta));
        dy = DY_CM2PX(r_robot*sin(curr_theta));

        video_draw_filled_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, 174, 136, 86, 255);
        video_draw_arrow(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, dx, dy, 10, BLACK(), 255);

        // draw active obstacles
        for(i = 0; obss && i < nb_obss; i++){
            if(obss[i].active){
                x = X_CM2PX(obss[i].x);
                y = Y_CM2PX(obss[i].y);
                if(obss[i].r > r_robot){
                    r = CM2PX(obss[i].r - r_robot);
                }
                else{
                    r = CM2PX(obss[i].r);
                }

                if(r > 0){
                    video_draw_filled_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, ORANGE(255), 255);
                }
                else{
                    video_draw_cross(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, CM2PX(8.), ORANGE(255), 255);
                }
            }
        }

        // draw fixed elements
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(96.7), Y_CM2PX(142.), X_CM2PX(150.), Y_CM2PX(200.), YELLOW(255), 255);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(150.), Y_CM2PX(142.), X_CM2PX(203.3), Y_CM2PX(200.), GREEN(200), 255);
        video_draw_filled_rectangle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), X_CM2PX(120.), Y_CM2PX(0.), X_CM2PX(180.), Y_CM2PX(10.), RED(255), 255);

        // draw obstacles' label
        for(i = 0; obss && i < nb_obss; i++){
            float f_dx, f_dy, f_tmp;

#if 1
            f_dy = 5.;
            f_dx = f_dy*4./3.;

            f_tmp = obss[i].x + 0.5;
            if(f_tmp < 1.) f_tmp = 1.;
            if(f_tmp > ctx->pos_maxx - f_dx - 1.) f_tmp = ctx->pos_maxx - f_dx - 1.;
            x = X_CM2PX(f_tmp);

            f_tmp = obss[i].y + 0.5;
            if(f_tmp < 1.) f_tmp = 1.;
            if(f_tmp > ctx->pos_maxy - f_dy - 1.) f_tmp = ctx->pos_maxy - f_dy - 1.;
            y = Y_CM2PX(f_tmp);

            dy = CM2PX(f_dy);
#else
            // deterministic pseudo-random positioning of the labels: yeah, this is paradoxical
            f_dy = 4.;
            f_dx = f_dy*4/3;

            f_tmp = obss[i].x + -30. + ((i*79)%59);
            if(f_tmp < 1.) f_tmp = 1.;
            if(f_tmp > ctx->pos_maxx - f_dx - 1.) f_tmp = ctx->pos_maxx - f_dx - 1.;
            x = X_CM2PX(f_tmp);

            f_tmp = obss[i].y + 15 - ((i*79)%29);
            if(f_tmp < 1.) f_tmp = 1.;
            if(f_tmp > ctx->pos_maxy - f_dy - 1.) f_tmp = ctx->pos_maxy - f_dy - 1.;
            y = Y_CM2PX(f_tmp);

            dy = CM2PX(f_dy);

            x1 = X_CM2PX(obss[i].x);
            y1 = Y_CM2PX(obss[i].y);

            video_draw_line(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, x1, y1, BLACK(), 60);
#endif

            sprintf(s_nb, "%02i", i);
            video_draw_string(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), &ctx->vdsctx, x, y, dy, s_nb, BLUE(255), 255);
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
            x = X_CM2PX(pel->pos.pos.x);
            y = Y_CM2PX(pel->pos.pos.y);

            video_draw_pixel(ctx->pos_data[ctx->pos_cur], ROWSTRIDE(), HEIGHT(), x, y, BLUE(255), 255*(i + ctx->poslist.maxlen - ctx->poslist.len)/ctx->poslist.maxlen);
            video_draw_circle(ctx->pos_data[ctx->pos_cur], WIDTH(), HEIGHT(), ROWSTRIDE(), x, y, r, BLUE(255), 255*(i + ctx->poslist.maxlen - ctx->poslist.len)/ctx->poslist.maxlen);
        }

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

void usage(char *cl) {
    printf("GTK UI\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--win-factor,     -w          size of the window\n");
    printf("\t--verbose,        -v          increases verbosity\n");
    printf("\t--quiet,          -q          not verbose\n");
    printf("\t--help,           -h, -?      prints this help\n");
}

int main(int argc, char *argv[]) {
    context_t ctx;
    int ret;

    memset(&ctx, 0, sizeof(ctx));
    pl_init(&ctx.poslist, 100);
    tl_init(&ctx.trajlist, 2);

    ret = video_draw_strings_init(&ctx.vdsctx, "/usr/share/fonts/truetype/ubuntu-font-family/Ubuntu-R.ttf");
    if(ret < 0){
        printf("video_draw_strings_init failed err#%i\n", -ret);
        exit(1);
    }

    // arguments options
    ctx.verbose = 1;
    ctx.pos_wfactor = 4.;
    while(1) {
        static struct option long_options[] = {
            {"win-factor",      required_argument,  NULL, 'w'},
            {"verbose",         no_argument,        NULL, 'v'},
            {"quiet",           no_argument,        NULL, 'q'},
            {"help",            no_argument,        NULL, 'h'},
            {NULL,              0,                  NULL, 0}
        };

        int c = getopt_long(argc, argv, "w:vqh?", long_options, NULL);
        if(c == -1)
            break;

        switch(c) {
        case 'w':
            ctx.pos_wfactor = strtof(optarg, NULL);
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
    ctx.pos_mid = gv_media_new("position", "Position des robots en temps réel", CTX_WIDTH(&ctx), CTX_HEIGHT(&ctx), GTK_SIGNAL_FUNC(event_cb), &ctx);
    ctx.pos_data[0] = malloc(CTX_BUFSZ(&ctx));
    ctx.pos_data[1] = malloc(CTX_BUFSZ(&ctx));
    ctx.pos_cur = 0;

    x_min = 0.;
    y_min = 0.;
    x_max = ctx.pos_maxx;
    y_max = ctx.pos_maxy;

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
//// send initial position
//        outMsg.header.type = E_POS;
//        outMsg.header.size = sizeof(outMsg.payload.pos);
//
//        outMsg.payload.pos.id = 0;
//        outMsg.payload.pos.theta = M_PI;
//        outMsg.payload.pos.u_a = 0;
//        outMsg.payload.pos.u_a_theta = 0;
//        outMsg.payload.pos.u_b = 0;
//        outMsg.payload.pos.x = 294.;
//        outMsg.payload.pos.y = 57.;
//        if(ctx.verbose >= 1){
//            printf("Sending initial position to robot%i (%fcm,%fcm,%f°).\n", outMsg.payload.pos.id, outMsg.payload.pos.x, outMsg.payload.pos.y, outMsg.payload.pos.theta*180./M_PI);
//        }
//        ret = role_send(&outMsg);
//        if(ret <= 0){
//            fprintf(stderr, "role_send() error #%i\n", -ret);
//        }

// ask obstacles
        outMsg.header.destAddr = role_get_addr(ROLE_PRIM_AI);
        outMsg.header.type = E_OBS_CFG;
        outMsg.header.size = sizeof(outMsg.payload.obsCfg);

        outMsg.payload.obsCfg.nb_obs = 0;
        outMsg.payload.obsCfg.r_robot = 0.;
        outMsg.payload.obsCfg.x_min = 0.;
        outMsg.payload.obsCfg.x_max = 0.;
        outMsg.payload.obsCfg.y_min = 0.;
        outMsg.payload.obsCfg.y_max = 0.;

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
