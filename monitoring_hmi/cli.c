#include <stdlib.h>
#include <stdio.h>
#include <signal.h> // ctrl+c for menu
#include <getopt.h> // parameters parsing
#include <termios.h> // terminal raw mode
#include <unistd.h> // select, read
#include <sys/time.h> // select
#include <sys/types.h> // select
#include <ctype.h> // isspace, toupper
#include <math.h> // M_PI, cos, sin
#include <errno.h> // errno

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../../global_errors.h"
#include "node_cfg.h"

// terminal-related definitions
static struct termios orig_termios;
int tty_reset(){
    // flush and reset
    if(tcsetattr(0, TCSAFLUSH, &orig_termios) < 0) return -1;
    return 0;
}
void tty_atexit(){
    tty_reset();
}
// put terminal in raw mode - see termio(7I) for modes
void tty_raw(){
    struct termios raw;

    raw = orig_termios;

    // input modes - clear indicated ones giving: no break, no CR to NL, no parity check, no strip char, no start/stop output (sic) control
    raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    // output modes - clear giving: no post processing such as NL to CR+NL
    raw.c_oflag &= ~(OPOST);

    // control modes - set 8 bit chars
    raw.c_cflag |= (CS8);

    // local modes - clear giving: echoing off, canonical off (no erase with backspace, ^U,...),  no extended functions, no signal chars (^Z,^C)
    raw.c_lflag &= ~(ECHO | ICANON | IEXTEN/* | ISIG*/);

    // control chars - set return condition: min number of bytes and timer
    raw.c_cc[VMIN] = 5; raw.c_cc[VTIME] = 8; // after 5 bytes or .8 seconds after first byte seen
    raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 0; // immediate - anything
    raw.c_cc[VMIN] = 2; raw.c_cc[VTIME] = 0; // after two bytes, no timer
    raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 8; // after a byte or .8 seconds

    // put terminal in raw mode after flushing
    if(tcsetattr(0, TCSAFLUSH, &raw) < 0){
        perror("tcsetattr");
        exit(EXIT_FAILURE);
    }
}

// signals-related definitions
volatile int sigint = 0;
void sigHandler(int sig){
    sigint = 1;
}

void usage(char *cl){
    printf("Command Line Interface\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--log-file, -f        output log file (overwritten)\n");
    printf("\t--distance, -d        distance travelled for each step (cm)\n");
    printf("\t--angle, -a           angle travelled fo each rotation step (°)\n");
    printf("\t--x-initial, -x       initial x position (cm)\n");
    printf("\t--y-initial, -y       initial y position (cm)\n");
    printf("\t--theta-initial, -t   initial orientation theta (°)\n");
    printf("\t--verbose, -v         increases verbosity\n");
    printf("\t--quiet, -q           not verbose\n");
    printf("\t--help, -h, -?        prints this help\n");
}

int main(int argc, char *argv[]){
    FILE *fd = NULL;
    sig_t prevH = NULL;
    sMsg inMsg, outMsg;
    bn_Address destAd;
    int ret, cmd, quit = 0, quitMenu = 0;
    enum{
        E_IDLE,
        E_REMOTE
    } state = E_IDLE;
    fd_set rfds;
    struct timeval tv;
    char c;
    float x = 0., y = 0., theta = 0., dist = 10., angle = M_PI/3., l_x, l_y, l_r;
    uint16_t tid = 0;
    enum{
        E_DIR_IDLE,
        E_DIR_STOP,
        E_DIR_FWD,
        E_DIR_LFT,
        E_DIR_RGT
    } dir_state = E_DIR_STOP;

    // setup terminal
    if(!isatty(0)){
        perror("isatty");
        exit(EXIT_FAILURE);
    }
    // store current tty settings in orig_termios
    if(tcgetattr(0, &orig_termios) < 0){
        perror("tcgetattr");
        exit(EXIT_FAILURE);
    }
    // reset terminal on exit (not a signal)
    if(atexit(tty_atexit)){
        fprintf(stderr, "can't set exit function\n");
        exit(EXIT_FAILURE);
    }

    // arguments parsing
    int verbose = 1;
    while(1){
        static struct option long_options[] = {
                {"log-file",        required_argument,  NULL, 'f'},
                {"distance",        required_argument,  NULL, 'd'},
                {"angle",           required_argument,  NULL, 'a'},
                {"x-initial",       required_argument,  NULL, 'x'},
                {"y-initial",       required_argument,  NULL, 'y'},
                {"theta-initial",   required_argument,  NULL, 't'},
                {"verbose",         no_argument,        NULL, 'v'},
                {"quiet",           no_argument,        NULL, 'q'},
                {"help",            no_argument,        NULL, 'h'},
                {NULL,              0,                  NULL, 0}
        };

        int c = getopt_long(argc, argv, "f:d:a:x:y:t:vqh?", long_options, NULL);
        if(c == -1)
            break;

        switch(c){
        case 'f':
            if(fd){
                fclose(fd);
            }
            fd = fopen(optarg, "wb+");
            break;
        case 'd':
            dist = fabs(strtof(optarg, NULL));
            break;
        case 'a':
            angle = fabs(strtof(optarg, NULL)*M_PI/180.);
            break;
        case 'x':
            x = strtof(optarg, NULL);
            break;
        case 'y':
            y = strtof(optarg, NULL);
            break;
        case 't':
            theta = strtof(optarg, NULL)*M_PI/180.;
            break;
        case 'v':
            verbose++;
            break;
        case 'q':
            verbose = 0;
            break;

        default:
           printf("?? getopt returned character code 0%o ??\n", c);
           /* no break */
        case 'h':
        case '?':
            usage(argv[0]);
            exit(EXIT_FAILURE);
            break;
        }
    }

    // arguments check

    // botNet initialization
    bn_init();
    if(verbose >= 1){
        printf("Identified as ADDRX_DEBUG on bn network.");
    }

    // send position
    outMsg.header.destAddr = ADDRI_MAIN_PROP;
    outMsg.header.type = E_POS;
    outMsg.header.size = sizeof(outMsg.payload.pos);
    outMsg.payload.pos.id = 0;
    outMsg.payload.pos.theta = theta;
    outMsg.payload.pos.u_a = 0;
    outMsg.payload.pos.u_a_theta = 0;
    outMsg.payload.pos.u_b = 0;
    outMsg.payload.pos.x = x;
    outMsg.payload.pos.y = y;
    if(verbose >= 1){
        printf("Sending initial position to robot%i (%fcm,%fcm,%f°).\n", outMsg.payload.pos.id, outMsg.payload.pos.x, outMsg.payload.pos.y, outMsg.payload.pos.theta*180./M_PI);
    }
    if(fd) fprintf(fd, "<POS %f, %f, %f\n", x, y, theta);
    ret = bn_sendAck(&outMsg);
    if(ret <= 0){
        fprintf(stderr, "bn_sendAck() error #%i\n", -ret);
    }

    // initial config
    if(verbose >= 1){
        printf("dist  = %.2fcm\n", dist);
        printf("angle = %.2f°\n", angle*180./M_PI);
    }
    if(fd) fprintf(fd, "=TRN %f, %f\n", dist, angle);

    // setup the SIGINT handler
    prevH = signal(SIGINT, sigHandler);
    printf("(Listening... Ctrl+C to show menu)\n");

    // main loop
    while(!quit){
        ret = bn_receive(&inMsg);
        if(ret < 0){
            if(ret == -ERR_INTERRUPTED){
                sigint=1;
            }
            else if(ret != -ERR_UART_READ_BYTE_TIMEOUT){
                fprintf(stderr, "bn_receive() error #%i\n", -ret);
                exit(1);
            }
        }

        switch(state){
        case E_IDLE:
            if(ret > 0 && verbose >= 1){
                printf("message received from %03hx, type : %s (%hhu)\n", inMsg.header.srcAddr, eType2str(inMsg.header.type), inMsg.header.type);

                if(verbose >= 2){
                    switch(inMsg.header.type){
                    case E_DEBUG:
                        printf("  %s\n", inMsg.payload.debug);
                        break;
                    case E_POS:
                        if(inMsg.payload.pos.id == 0){
                            x = inMsg.payload.pos.x;
                            y = inMsg.payload.pos.y;
                            theta = inMsg.payload.pos.theta;
                            if(fd) fprintf(fd, ">POS %f, %f, %f\n", x, y, theta);
                        }
                        printf("  robot%hhu@(%fcm,%fcm,%f°)\n", inMsg.payload.pos.id, inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta*180./M_PI);
                        break;
                    default:
                        break;
                    }
                }
            }

            //menu
            if(sigint){
                quitMenu = 0;
                while(!quitMenu){
                    printf("\n#Menu:\n");
                    printf("#  s - send debugger address to distant node\n");
                    printf("#  m - start remote mode (Ctrl+C to exit)\n");
                    printf("#  r - return\n");
                    printf("#  q - quit\n");

                    printf("#> ");
                    while(isspace(cmd=getchar()));

                    switch (toupper(cmd)){
                    case 'S':  //sends debug address to distant node
                        do{
                            printf("#enter destination address\n");
                            ret = scanf("%hx", &destAd);
                            if(ret != 1){
                                printf("#error getting destination address\n");
                            }
                        }while(ret != 1);
                        if((ret = bn_debugSendAddr(destAd)) > 0){
                            printf("#signalling send\n");
                            quitMenu=1;
                        }
                        else{
                            printf("#error while sending : %d\n", ret);
                        }
                        break;
                    case 'M':
                        state = E_REMOTE;
                        printf("#switching to remote mode, Ctrl+C to stop remote mode and open menu\n");
                        tty_raw();
                        printf("\n\x1b[s");
                        quitMenu = 1;
                        break;
                    case 'R':
                        quitMenu = 1;
                        printf("#back to listening\n");
                        break;
                    case 'Q':
                        quitMenu = 1;
                        quit = 1;
                        break;
                    default:
                        break;
                    }
                }

                sigint=0;
            }
            break;
        case E_REMOTE:
            if(sigint){
                state = E_IDLE;
                tty_reset();
//                sigint = 0; keep sigint==1 go back to E_IDLE menu
            }

            if(ret > 0){
                switch(inMsg.header.type){
                case E_POS:
                    if(inMsg.payload.pos.id == 0){
                        x = inMsg.payload.pos.x;
                        y = inMsg.payload.pos.y;
                        theta = inMsg.payload.pos.theta;

                        if(fd) fprintf(fd, ">POS %f, %f, %f\n", x, y, theta);
                        printf("\x1b[u\x1b[Krobot%hhu @ %fcm, %fcm, %f°", inMsg.payload.pos.id, x, y, theta*180./M_PI);
                        fflush(stdout);

// #### start message handling
                        switch(dir_state){
                        case E_DIR_FWD:
                            // header
                            outMsg.header.destAddr = ADDRI_MAIN_PROP;
                            outMsg.header.type = E_TRAJ;
                            outMsg.header.size = sizeof(outMsg.payload.traj);
                            // payload
                            outMsg.payload.traj.p1_x = x;
                            outMsg.payload.traj.p1_y = y;
                            outMsg.payload.traj.p2_x = x + dist*cos(theta);
                            outMsg.payload.traj.p2_y = y + dist*sin(theta);
                            outMsg.payload.traj.seg_len = dist;
                            outMsg.payload.traj.c_x = outMsg.payload.traj.p2_x;
                            outMsg.payload.traj.c_y = outMsg.payload.traj.p2_y;
                            outMsg.payload.traj.c_r = 0.;
                            outMsg.payload.traj.arc_len = 0.;
                            outMsg.payload.traj.sid = 0;
                            outMsg.payload.traj.tid = tid++;

                            if(fd) fprintf(fd, "<FWD %f, %f, %f, %f, %f, %f, %f, %f, %f, %hu, %hu\n", outMsg.payload.traj.p1_x, outMsg.payload.traj.p1_y, outMsg.payload.traj.p2_x, outMsg.payload.traj.p2_y, outMsg.payload.traj.seg_len, outMsg.payload.traj.c_x, outMsg.payload.traj.c_y, outMsg.payload.traj.c_r, outMsg.payload.traj.arc_len, outMsg.payload.traj.tid, outMsg.payload.traj.sid);
                            ret = bn_send(&outMsg);
                            if(ret <= 0){
                                fprintf(stderr, "bn_send() error #%i\r\n", -ret); // '\r' needed, raw mode
                            }
                            break;
                        case E_DIR_STOP:
                            // header
                            outMsg.header.destAddr = ADDRI_MAIN_PROP;
                            outMsg.header.type = E_TRAJ;
                            outMsg.header.size = sizeof(outMsg.payload.traj);
                            // payload
                            outMsg.payload.traj.p1_x = x;
                            outMsg.payload.traj.p1_y = y;
                            outMsg.payload.traj.p2_x = x;
                            outMsg.payload.traj.p2_y = y;
                            outMsg.payload.traj.seg_len = 0.;
                            outMsg.payload.traj.c_x = x;
                            outMsg.payload.traj.c_y = y;
                            outMsg.payload.traj.c_r = 0.;
                            outMsg.payload.traj.arc_len = 0.;
                            outMsg.payload.traj.sid = 0;
                            outMsg.payload.traj.tid = tid++;

                            if(fd) fprintf(fd, "<STP %f, %f, %f, %f, %f, %f, %f, %f, %f, %hu, %hu\n", outMsg.payload.traj.p1_x, outMsg.payload.traj.p1_y, outMsg.payload.traj.p2_x, outMsg.payload.traj.p2_y, outMsg.payload.traj.seg_len, outMsg.payload.traj.c_x, outMsg.payload.traj.c_y, outMsg.payload.traj.c_r, outMsg.payload.traj.arc_len, outMsg.payload.traj.tid, outMsg.payload.traj.sid);
                            ret = bn_send(&outMsg);
                            if(ret <= 0){
                                fprintf(stderr, "bn_send() error #%i\r\n", -ret); // '\r' needed, raw mode
                            }
                            break;
                        case E_DIR_RGT:
                            // header
                            outMsg.header.destAddr = ADDRI_MAIN_PROP;
                            outMsg.header.type = E_TRAJ;
                            outMsg.header.size = sizeof(outMsg.payload.traj);

                            // payload #1
                            outMsg.payload.traj.p1_x = x;
                            outMsg.payload.traj.p1_y = y;
                            outMsg.payload.traj.p2_x = x;
                            outMsg.payload.traj.p2_y = y;
                            outMsg.payload.traj.seg_len = 0.;
                            l_r = dist/angle;
                            outMsg.payload.traj.c_r = l_r;
                            l_x = l_r*sin(angle);
                            l_y = l_r*(cos(angle)-1);
                            outMsg.payload.traj.c_x = x + l_r*sin(theta);
                            outMsg.payload.traj.c_y = y - l_r*cos(theta);
                            outMsg.payload.traj.arc_len = dist;
                            outMsg.payload.traj.sid = 0;
                            outMsg.payload.traj.tid = tid;

                            if(fd) fprintf(fd, "<RG1 %f, %f, %f, %f, %f, %f, %f, %f, %f, %hu, %hu\n", outMsg.payload.traj.p1_x, outMsg.payload.traj.p1_y, outMsg.payload.traj.p2_x, outMsg.payload.traj.p2_y, outMsg.payload.traj.seg_len, outMsg.payload.traj.c_x, outMsg.payload.traj.c_y, outMsg.payload.traj.c_r, outMsg.payload.traj.arc_len, outMsg.payload.traj.tid, outMsg.payload.traj.sid);
                            ret = bn_send(&outMsg);
                            if(ret <= 0){
                                fprintf(stderr, "bn_send() error #%i\r\n", -ret); // '\r' needed, raw mode
                            }

                            // payload #2
                            outMsg.payload.traj.p1_x = x + cos(theta)*l_x - sin(theta)*l_y;
                            outMsg.payload.traj.p1_y = y + sin(theta)*l_x + cos(theta)*l_y;
                            outMsg.payload.traj.p2_x = outMsg.payload.traj.p1_x;
                            outMsg.payload.traj.p2_y = outMsg.payload.traj.p1_y;
                            outMsg.payload.traj.seg_len = 0.;
                            outMsg.payload.traj.c_r = 0.;
                            outMsg.payload.traj.c_x = outMsg.payload.traj.p1_x;
                            outMsg.payload.traj.c_y = outMsg.payload.traj.p1_y;
                            outMsg.payload.traj.arc_len = 0.;
                            outMsg.payload.traj.sid = 1;
                            outMsg.payload.traj.tid = tid++;

                            if(fd) fprintf(fd, "<RG2 %f, %f, %f, %f, %f, %f, %f, %f, %f, %hu, %hu\n", outMsg.payload.traj.p1_x, outMsg.payload.traj.p1_y, outMsg.payload.traj.p2_x, outMsg.payload.traj.p2_y, outMsg.payload.traj.seg_len, outMsg.payload.traj.c_x, outMsg.payload.traj.c_y, outMsg.payload.traj.c_r, outMsg.payload.traj.arc_len, outMsg.payload.traj.tid, outMsg.payload.traj.sid);
                            ret = bn_send(&outMsg);
                            if(ret <= 0){
                                fprintf(stderr, "bn_send() error #%i\r\n", -ret); // '\r' needed, raw mode
                            }

                            break;
                        case E_DIR_LFT:
                            // header
                            outMsg.header.destAddr = ADDRI_MAIN_PROP;
                            outMsg.header.type = E_TRAJ;
                            outMsg.header.size = sizeof(outMsg.payload.traj);

                            // payload #1
                            outMsg.payload.traj.p1_x = x;
                            outMsg.payload.traj.p1_y = y;
                            outMsg.payload.traj.p2_x = x;
                            outMsg.payload.traj.p2_y = y;
                            outMsg.payload.traj.seg_len = 0.;
                            l_r = -dist/angle;
                            outMsg.payload.traj.c_r = l_r;
                            l_x = -l_r*sin(angle);
                            l_y = l_r*(cos(angle)-1);
                            outMsg.payload.traj.c_x = x + l_r*sin(theta);
                            outMsg.payload.traj.c_y = y - l_r*cos(theta);
                            outMsg.payload.traj.arc_len = dist;
                            outMsg.payload.traj.sid = 0;
                            outMsg.payload.traj.tid = tid;

                            if(fd) fprintf(fd, "<LF1 %f, %f, %f, %f, %f, %f, %f, %f, %f, %hu, %hu\n", outMsg.payload.traj.p1_x, outMsg.payload.traj.p1_y, outMsg.payload.traj.p2_x, outMsg.payload.traj.p2_y, outMsg.payload.traj.seg_len, outMsg.payload.traj.c_x, outMsg.payload.traj.c_y, outMsg.payload.traj.c_r, outMsg.payload.traj.arc_len, outMsg.payload.traj.tid, outMsg.payload.traj.sid);
                            ret = bn_send(&outMsg);
                            if(ret <= 0){
                                fprintf(stderr, "bn_send() error #%i\r\n", -ret); // '\r' needed, raw mode
                            }

                            // payload #2
                            outMsg.payload.traj.p1_x = x + cos(theta)*l_x - sin(theta)*l_y;
                            outMsg.payload.traj.p1_y = y + sin(theta)*l_x + cos(theta)*l_y;
                            outMsg.payload.traj.p2_x = outMsg.payload.traj.p1_x;
                            outMsg.payload.traj.p2_y = outMsg.payload.traj.p1_y;
                            outMsg.payload.traj.seg_len = 0.;
                            outMsg.payload.traj.c_r = 0.;
                            outMsg.payload.traj.c_x = outMsg.payload.traj.p1_x;
                            outMsg.payload.traj.c_y = outMsg.payload.traj.p1_y;
                            outMsg.payload.traj.arc_len = 0.;
                            outMsg.payload.traj.sid = 1;
                            outMsg.payload.traj.tid = tid++;

                            if(fd) fprintf(fd, "<LF2 %f, %f, %f, %f, %f, %f, %f, %f, %f, %hu, %hu\n", outMsg.payload.traj.p1_x, outMsg.payload.traj.p1_y, outMsg.payload.traj.p2_x, outMsg.payload.traj.p2_y, outMsg.payload.traj.seg_len, outMsg.payload.traj.c_x, outMsg.payload.traj.c_y, outMsg.payload.traj.c_r, outMsg.payload.traj.arc_len, outMsg.payload.traj.tid, outMsg.payload.traj.sid);
                            ret = bn_send(&outMsg);
                            if(ret <= 0){
                                fprintf(stderr, "bn_send() error #%i\r\n", -ret); // '\r' needed, raw mode
                            }

                            break;
                        case E_DIR_IDLE:
                        default:
                            break;
                        }

                        dir_state = E_DIR_IDLE;
// #### end message handling
                    }
                    break;
                default:
                    break;
                }
            }

            // check if any data is available
            FD_ZERO(&rfds);
            FD_SET(0, &rfds);
            tv.tv_sec = 0;
            tv.tv_usec = 1000;

            ret = select(0+1, &rfds, NULL, NULL, &tv);
            if(ret < 0){
                if(errno == EINTR){
                    sigint = 1;
                    break;
                }
                else{
                    perror("select");
                    exit(EXIT_FAILURE);
                }
            }
            else if(!ret){
                break;
            }

            // data available
            ret = read(0, &c, 1);
            if(ret < 0){
                if(errno == EINTR){
                    sigint = 1;
                    break;
                }
                else{
                    perror("read(0)");
                    exit(1);
                }
            }
            else if(!ret){
                break;
            }

            switch(toupper((int)c)){
            case 'Z':
                dir_state = E_DIR_FWD;
                break;
            case 'Q':
                dir_state = E_DIR_LFT;
                break;
            case 'D':
                dir_state = E_DIR_RGT;
                break;
            case 'S':
                dir_state = E_DIR_STOP;
                break;
            default:
                dir_state = E_DIR_IDLE;
                break;
            }
            if(fd) fprintf(fd, "=DIR %i\n", dir_state);
            break;
        }
    }

    printf("Ciao!\n");
    if(fd){
        fclose(fd);
    }
    if(prevH){
        prevH(SIGINT);
    }
    return 0;
}
