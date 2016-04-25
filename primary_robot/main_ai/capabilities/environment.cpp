/*
 * environment.cpp
 *
 *  Created on: 19 avr. 2015
 *      Author: Sébastien Malissard
 */

#include "environment.h"
#include "communications.h"


std::vector<astar::sObs_t> obs= {
   // robots
   {{0., 0.},          0., 1, 1, 1},    //primary
   {{0., 0.}, R_ROBOT+8. , 1, 0, 1},    //secondary
   {{0., 0.}, R_ROBOT+25., 1, 0, 1},    //primary adv
   {{0., 0.}, R_ROBOT+20., 1, 0, 1},    //secondary adv

   //Walls in building zone
   {{150., 125.}, 2.+R_ROBOT, 0, 1, 1}, //4
   {{120., 125.}, 2.+R_ROBOT, 0, 1, 1},
   {{90., 125.}, 2.+R_ROBOT, 0, 1, 1},
   {{210., 125.}, 2.+R_ROBOT, 0, 1, 1},
   {{180., 125.}, 2.+R_ROBOT, 0, 1, 1},
   {{150., 65.}, 2.+R_ROBOT, 0, 1, 1},
   {{150., 95.}, 2.+R_ROBOT, 0, 1, 1},

   //Walls in Dune
   {{80., 180.}, 2.+R_ROBOT, 0, 1, 1}, //11
   {{220., 180.}, 2.+R_ROBOT, 0, 1, 1},

   //Rocks in corner
   {{0., 0.}, 25.+R_ROBOT, 0, 1, 1}, //13
   {{300., 0.}, 25.+R_ROBOT, 0, 1, 1},

   //Sand heaps (in front of the starting zone)
   {{65., 110.}, 8.+R_ROBOT, 1, 1, 1},//15
   {{235., 110.}, 8.+R_ROBOT, 1, 1, 1},

   //Dune heaps (heaps on the sides of the dune)
   {{210., 194.}, 8.+R_ROBOT, 1, 1, 1},//17
   {{90., 194.}, 8.+R_ROBOT, 1, 1, 1},

   //Yellow spots //if moved change START_STAND
//   {{85. ,180.}, 3. + R_ROBOT, 1, 1, 1}, //4
//   {{85. ,190.}, 3. + R_ROBOT, 1, 1, 1},
//   {{9.  , 25.}, 3. + R_ROBOT, 1, 1, 1},
//   {{9.  , 15.}, 3. + R_ROBOT, 1, 1, 1},
//   {{9.  ,180.}, 3. + R_ROBOT, 1, 1, 1},
//   {{87. ,64.5}, 3. + R_ROBOT, 1, 1, 1},
//   {{110., 23.}, 3. + R_ROBOT, 1, 1, 1},
//   {{130., 60.}, 3. + R_ROBOT, 1, 1, 1},
//
//   //Green spots
//   {{300 - 85. ,180.}, 3. + R_ROBOT, 1, 1, 1}, //12
//   {{300 - 85. ,190.}, 3. + R_ROBOT, 1, 1, 1},
//   {{300 - 9.  , 25.}, 3. + R_ROBOT, 1, 1, 1},
//   {{300 - 9.  , 15.}, 3. + R_ROBOT, 1, 1, 1},
//   {{300 - 9.  ,180.}, 3. + R_ROBOT, 1, 1, 1},
//   {{300 - 87. ,64.5}, 3. + R_ROBOT, 1, 1, 1},
//   {{300 - 110., 23.}, 3. + R_ROBOT, 1, 1, 1},
//   {{300 - 130., 60.}, 3. + R_ROBOT, 1, 1, 1},
//
//   //Cup
//   {{25. , 25.}, 5. + R_ROBOT, 1, 1, 1}, //20
//   {{91. ,120.}, 5. + R_ROBOT, 1, 1, 1},
//   {{150., 35.}, 5. + R_ROBOT, 1, 1, 1},
//   {{300 - 25. , 25.}, 5. + R_ROBOT, 1, 1, 1},
//   {{300 - 91. ,120.}, 5. + R_ROBOT, 1, 1, 1},
//
//   //Popcorn machine
//   {{30. ,196.5}, 5. + R_ROBOT, 0, 1, 1}, //25
//   {{60. ,196.5}, 5. + R_ROBOT, 0, 1, 1},
//   {{300 - 30. ,196.5}, 5. + R_ROBOT, 0, 1, 1},
//   {{300 - 60. ,196.5}, 5. + R_ROBOT, 0, 1, 1},
//
//   //Stairs
//   {{102. ,147.}, 7. + R_ROBOT, 0, 1, 1}, //29
//   {{150.,200.}, 53. + R_ROBOT, 0, 1, 1},
//   {{300 - 102. ,147.}, 7. + R_ROBOT, 0, 1, 1},
//
//   //Platform
//   {{125.,  5.}, 7. + R_ROBOT, 0, 1, 1}, //32
//   {{300 - 125.,  5.}, 7. + R_ROBOT, 0, 1, 1},
//
//   //Starting zone
//   {{39., 79.}, 2. + R_ROBOT + .5, 0, 1, 1}, //34
//   {{15., 79.}, 2. + R_ROBOT+ .5, 0, 1, 1},
//   {{39.,121.}, 2. + R_ROBOT+ .5, 0, 1, 1},
//   {{15.,121.}, 2. + R_ROBOT+ .5, 0, 1, 1},
//   {{300 - 39., 79.}, 2. + R_ROBOT+ .5, 0, 1, 1},
//   {{300 - 15., 79.}, 2. + R_ROBOT+ .5, 0, 1, 1},
//   {{300 - 39.,121.}, 2. + R_ROBOT+ .5, 0, 1, 1},
//   {{300 - 15.,121.}, 2. + R_ROBOT+ .5, 0, 1, 1},
//
   //Block adversary zone
   {{0., 0.}, 0., 1, 1, 1}, //19
//
   //Cercles du robot anti-demi-tour
   {{0., 0. }, 0, 0, 0, 1}, //20
   {{0., 0. }, 0, 0, 0, 1},
   {{0., 0. }, 0, 0, 0, 1},

   //Cercles d'approches
   {{0., 0. }, 0, 0, 0, 1},//23
   {{0., 0. }, 0, 0, 0, 1},
   {{0., 0. }, 0, 0, 0, 1},

   // arrivée
   {{0. , 0.}, 0, 0, 1, 1} //25
};

std::vector<uint8_t> obsUpdated(obs.size());

Environment Env2016::env(obs);

void Env2016::setup(){
    for(unsigned int i = 0 ; i < env.obs_updated.size() ; i++)
        if(env.obs[i].active)
            env.obs_updated[i] = 1;
}

int Env2016::loop(){

    if(askObsCfg(true))
        sendObsCfg(obs.size(), R_ROBOT, X_MIN, X_MAX, Y_MIN, Y_MAX);

    env.obs_updated[0] = 0;             // Already updated by the propulsion
    sendObss(env.obs, env.obs_updated);

    return 0;
}
