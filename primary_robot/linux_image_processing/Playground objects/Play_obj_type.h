#ifndef PLAY_OBJ_TYPE_H
#define PLAY_OBJ_TYPE_H

#include <stdlib.h>

typedef enum eObjShape {
    cone,
    cylinder,
    parallelepiped,
    objShapeMax
}eObjShape;

typedef enum eObjCol {
    yellowDaffodil,  // Sand pieces
    whiteTraffic,   // Neutral shellfishes
    greenEmerald,   // For team green
    violetSignal,    // For team violet
    objColMax
}eColObj;

typedef enum eObjType {
    sandCube,
    sandCyl,
    sandCone,
    shellWhite,
    shellGreen,
    shellViolet,
    objTypeMax
}eObjType;
#endif
