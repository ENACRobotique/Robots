#include "pos_estimate.h"

const int pos_next[pNUM /* prev position */][2 /* left sensor */][2 /* right sensor */] = {
    // -----------left 0--------------------    ---------left 1-----------------
    // ---right 0----|----right 1-----------    ---right 0--------|--right 1----

// pLOST
    { { pLOST, pLEFT_CENTER }, { pRIGHT_CENTER, pCENTER|pWTF } },

// pLEFT
    { { pLEFT, pLEFT_CENTER }, { pRIGHT_CENTER|pWTF, pCENTER|pWTF } },
// pLEFT_CENTER
    { { pLEFT, pLEFT_CENTER },  { pRIGHT_CENTER|pWTF, pCENTER } },
// pCENTER
    { { pLOST|pWTF, pLEFT_CENTER }, { pRIGHT_CENTER, pCENTER } },
// pRIGHT_CENTER
    { { pRIGHT, pLEFT_CENTER|pWTF }, { pRIGHT_CENTER, pCENTER } },
// pRIGHT
    { { pRIGHT, pLEFT_CENTER|pWTF }, { pRIGHT_CENTER, pCENTER|pWTF } }
};

