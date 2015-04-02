#include <stdlib.h>
#include <stdio.h>

#include <millis.h>
#include <shared/botNet_core.h>
#include <messages.h>

#include "debug.h"
#include "tools.h"
#include "params.h"
#include "position_controller.h"
#include "trajectory_manager.h"
#include "global_errors.h"
#include "bn_intp.h"

/*
 *      y axis
 *         ^
 *    P2   #   P1
 *     \   #   /
 *      \  #  /
 *       \ # /
 *        \#/
 *         o######### > x axis
 *         |
 *         |
 *         |
 *         |
 *         P3
 */

/* from theoretical data
 M_rob2pods =
 -5.00000000000000e-01   8.66025403784439e-01   1.34580000000000e+01
 -5.00000000000000e-01  -8.66025403784439e-01   1.34580000000000e+01
 1.00000000000000e+00  -1.83697019872103e-16   1.34580000000000e+01
 */
const int32_t mat_rob2pods[NB_PODS][NB_SPDS] = {
        { -0.5 * dMSHIFT, 0.866025403784439 * dMSHIFT, D2I(13.458) * dMoRSHIFT },
        { -0.5 * dMSHIFT, -0.866025403784439 * dMSHIFT, D2I(13.458) * dMoRSHIFT },
        { 1. * dMSHIFT, 0 * dMSHIFT, D2I(13.458) * dMoRSHIFT }
};

int main() {
    int ret;

    // Debug
    debug_leds_init();
    debug_switches_init();

    // Trajectory manager
    trajectory_manager_t traj_mngr;
    trajmngr_init(&traj_mngr, mat_rob2pods);

    // BotNet initialization (i²c + uart)
    bn_attach(E_ROLE_SETUP, role_setup);
    bn_intp_install();
    ret = bn_init();
    if(ret < 0){
        printf("bn_init() failed, err: %s (%i)\n", getErrorStr(-ret), -ret);
        exit(1);
    }

    //// Global variables
    unsigned int prevControl_us = micros();
    unsigned int prevPos_ms = millis();
    sMsg inMsg = { { 0 } }, outMsg = { { 0 } };

    // FIXME need to have a first loop here to wait for time synchronization

    while (1) {
        // Reception and processing of the message
        ret = bn_receive(&inMsg);
        if(ret < 0) {
            printf("bn_receive() failed, err: %s (%i)\n", getErrorStr(-ret), -ret);
            exit(1);
        }
        else if (ret > 0) {
            ret = role_relay(&inMsg); // relay any received message if asked to
            if(ret < 0) {
                printf("role_relay() failed, err: %s (%i)\n", getErrorStr(-ret), -ret);
            }

            switch (inMsg.header.type) {
            case E_TRAJ_ORIENT_EL: // Get the new step of a trajectory
                printf("got traj elt: tid%hu, sid%hhu\n", (uint16_t)inMsg.payload.trajOrientEl.tid, (uint8_t)inMsg.payload.trajOrientEl.sid);
                trajmngr_new_traj_el(&traj_mngr, &inMsg.payload.trajOrientEl);
                break;
            case E_POS:
                printf("got position: %.2fcm %.2fcm %.2f°\n", inMsg.payload.pos.x, inMsg.payload.pos.y, inMsg.payload.pos.theta * 180. / M_PI);
                trajmngr_set_pos(&traj_mngr, &inMsg.payload.pos);
                break;
            default:
                printf("got unhandled message with type: %s (%i)\n", eType2str(inMsg.header.type), inMsg.header.type);
                break;
            }
        }

        // Automatic control
        if (micros() - prevControl_us >= PER_CTRL_LOOP_US) {
            // If there is too much delay we skip to the next increment of the loop
            if (micros() - prevControl_us > PER_CTRL_LOOP_CRITIC_US) {
                prevControl_us = micros();
                trajmngr_reset(&traj_mngr);
                continue;
            }
            prevControl_us = micros();

            // Control trajectory
            trajmngr_update(&traj_mngr);
        }

        if (millis() - prevPos_ms >= 100) {
            prevPos_ms = millis();

            // FIXME todo
//            outMsg.header.type = E_GENERIC_STATUS;
//            outMsg.header.size = sizeof(outMsg.payload.genericStatus);
//            trajmngr_fill_pos(&traj_mngr, &outMsg.payload.genericStatus);

            {
                //    msg.header.destAddr = ADDRD_MONITORING; this is a role_send => the destination address is ignored
                outMsg.header.type = E_POS;
                outMsg.header.size = sizeof(outMsg.payload.pos);
                outMsg.payload.pos.id = ELT_PRIMARY; // main robot
                outMsg.payload.pos.x = I2Ds(traj_mngr.ctlr.x);
                outMsg.payload.pos.y = I2Ds(traj_mngr.ctlr.y);
                outMsg.payload.pos.theta = (double)traj_mngr.ctlr.theta / dASHIFT;
            }

            role_send(&outMsg);
        }
    }
}
