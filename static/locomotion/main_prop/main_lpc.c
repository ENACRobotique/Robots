#include <lpc214x.h>
#include <asserv.h>
#include <gpio.h>
#include <messages.h>
#include <pos_history.h>
#include <roles.h>
#include <shared/botNet_core.h>
#include <shared/message_header.h>
#include <sys_time.h>

#include "../../communication/network_tools/bn_debug.h"
#include "../../communication/network_tools/bn_intp.h"

// pins usage with PCB_IA_C2014R2:
//    UART0
//        TXD0    P0.0    EXT1.1      BOTNET          !PWM1
//        RXD0    P0.1    EXT1.2      BOTNET          !PWM3 ~EINT0
//    I²C0
//        SCL0    P0.2    EXT1.3      BOTNET
//        SDA0    P0.3    EXT1.4      BOTNET          ~EINT1
//    EINT*
//        EINT0   P0.16   EXT1.17     ENC_A RIGHT
//        EINT3   P0.20   EXT1.21     ENC_A LEFT
//    PWM*
//        PWM5    P0.21   EXT1.22     SPEED LEFT
//        PWM2    P0.7    EXT1.8      SPEED RIGHT     ~EINT2
//    GPIO*
//        OUT     P1.24   EXT2.13     Green LED
//        OUT     P0.31   EXT2.4      Orange LED
//        OUT     P0.19   EXT1.20     DIR LEFT
//        OUT     P0.5    EXT1.6      DIR RIGHT
//        IN      P0.22   EXT1.23     ENC_B LEFT
//        IN      P0.18   EXT1.19     ENC_B RIGHT
// for more details, see Features2Pins.txt file

void intp_sync_handler(sMsg *msg){
    bn_intp_msgHandle(msg);

    if(bn_intp_isSync()){
        tD_setLo2GlMsOffset(bn_intp_MicrosOffset);
    }
}

int main(int argc, char *argv[]){
    sMsg inMsg = {{0}}, outMsg = {{0}};
    int ret, quit = 0, ledStatus = 0;
    unsigned int prevAsserv = 0, prevPos = 0, prevDbg = 0, prevLed = 0;

    // botNet initialization
    bn_attach(E_ROLE_SETUP, role_setup);
    bn_attach(E_INTP, intp_sync_handler); // replaces bn_intp_install() to catch synchronization

    bn_init();

    // hardware initialization
    asserv_init();

    // main loop
    while(!quit){
        ret = bn_receive(&inMsg);
        if(ret > 0){
            role_relay(&inMsg); // relay any received message if asked to

            switch(inMsg.header.type){
            case E_SPEED_SETPOINT:
                new_speed_setpoint(inMsg.payload.speedSetPoint.speed);
                break;
            case E_TRAJ:
                new_traj_el(&inMsg.payload.traj);
                break;
            case E_POS:
                bn_printDbg("got pos");

                new_pos(&inMsg.payload.pos);
                break;
            case E_POS_QUERY:
                if(inMsg.payload.posQuery.id == ELT_PRIMARY){
                    outMsg.header.destAddr = inMsg.header.srcAddr;
                    outMsg.header.size = sizeof(outMsg.payload.genericStatus);
                    outMsg.header.type = E_GENERIC_STATUS;

                    if(!ph_get_pos(&outMsg.payload.genericStatus, tD_newFrom_GlUs(inMsg.payload.posQuery.date))){
                        bn_send(&outMsg);
                    }
                }
                break;
            case E_DATA:
            case E_PING:
                break;
            default:
                bn_printfDbg("got unhandled msg: type%hhu sz%hhu", inMsg.header.type, inMsg.header.size);
                break;
            }
        }

        if(millis() - prevAsserv >= 20){
            if(millis() - prevAsserv > 30) { // we are very late, do not take care of these data
                prevAsserv = millis();
                continue;
            }

            prevAsserv = millis();

            new_asserv_step();

            // enqueue latest position
            if(bn_intp_isSync()){
                unsigned int p_t; // local time (µs)
                sPHPos *slot = ph_get_new_slot_pointer();

                get_pos(&slot->s.prop_status.pos, &slot->s.prop_status.pos_u, &p_t);

                slot->s.date = bn_intp_micros2s(p_t); // global time
                slot->s.id = ELT_PRIMARY;

                slot->t = tD_newFrom_LoUs(p_t);

                ph_incr_new_slot_pointer();
            }
        }

        if(millis() - prevPos >= 100) {
            prevPos = millis();

            send_pos();
        }

        if(millis() - prevDbg >= 1000){
            prevDbg = millis();

            show_stats();
        }

        if(millis() - prevLed >= 500){
            prevLed = millis();

            gpio_write(0, 31, ledStatus^=1);
        }
    }

    return 0;
}
