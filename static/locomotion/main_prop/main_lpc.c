#include <stdlib.h>
#include <stdio.h>

#include <lpc214x.h>

#include <string.h>
#include <math.h>

#include <gpio.h>
#include <eint.h>
#include <ime.h>
#include <pwm.h>
#include <sys_time.h>
#include <trigo.h>

#include "../botNet/shared/botNet_core.h"
#include "../network_tools/bn_debug.h"

#include "params.h"

#include "controller.h"
#include "asserv.h"

// pins usage with PCB_IA_C2013R1:
//    I²C0
//        SCL0    P0.2    EXT1.3      BOTNET
//        SDA0    P0.3    EXT1.4      BOTNET          ~EINT1
//    EINT*
//        EINT0   P0.1    EXT1.2      ENC_A RIGHT     !UART0 !PWM3
//        EINT3   P0.20   EXT1.21     ENC_A LEFT
//    PWM*
//        PWM4    P0.8    EXT1.9      SPEED LEFT      !UART1
//        PWM6    P0.9    EXT1.10     SPEED RIGHT     !UART1 ~EINT3
//    GPIO*
//        OUT     P1.24   EXT2.13     Green LED
//        OUT     P0.31   EXT2.4      Orange LED
//        OUT     P1.22   EXT2.11     DIR LEFT
//        OUT     P1.23   EXT2.12     DIR RIGHT
//        IN      P1.16   EXT2.5      ENC_B LEFT
//        IN      P1.17   EXT2.6      ENC_B RIGHT
// for more details, see Features2Pins.txt file

int main(int argc, char *argv[]){
    sMsg inMsg;
    int ret, quit = 0, ledStatus = 0;
    unsigned int prevAsserv = 0, prevPos = 0, prevDbg = 0, prevLed = 0;

    // botNet initialization
    bn_attach(E_ROLE_SETUP, role_setup);

    bn_init();

    // hardware initialization
    asserv_init();

    prevAsserv = millis();

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
            case E_DATA:
            case E_PING:
                break;
            default:
                bn_printfDbg("got unhandled msg: type%hhu sz%hhu", inMsg.header.type, inMsg.header.size);
                break;
            }
        }

        if(millis() - prevAsserv >= 20){
            prevAsserv += 20;

            if(millis() - prevAsserv > 10) { // we are very late, do not take care of these data
                prevAsserv = millis();
                continue;
            }

            new_asserv_step();
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
