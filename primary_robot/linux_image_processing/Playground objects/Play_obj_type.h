#ifndef PLAY_OBJ_TYPE_H
#define PLAY_OBJ_TYPE_H

#include <stdlib.h>
#include <string>

#define USE_UI

typedef enum eObjShape {
    cone,
    cylinder,
    shell_cylinder,
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

std::string getStrObjType(eObjType type);
std::string getStrObjShape(eObjShape shape);
std::string getStrObjCol(eObjCol col);
#endif
