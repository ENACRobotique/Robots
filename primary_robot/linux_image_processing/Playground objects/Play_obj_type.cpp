#include "Play_obj_type.h"

#ifdef USE_UI
std::string getStrObjType(eObjType type){
    switch(type){
    case sandCube:
        return std::string("sandCube");
    case sandCyl:
        return std::string("sandCyl");
    case sandCone:
        return std::string("sandCone");
    case shellGreen:
        return std::string("shellGreen");
    case shellViolet:
        return std::string("shellViolet");
    case shellWhite:
        return std::string("shellWhite");
    case objTypeMax:
        return std::string("objTypeMax");
    default:
        return std::string("objTypeUnknown");
    }
}

std::string getStrObjShape(eObjShape shape){
    switch(shape){
    case parallelepiped:
        return std::string("parallelepiped");
    case cylinder:
        return std::string("cylinder");
    case cone:
        return std::string("cone");
    case objShapeMax:
        return std::string("objShapeMax");
    default:
        return std::string("objShapeUnknown");
    }
}

std::string getStrObjCol(eObjCol col){
    switch(col){
    case yellowDaffodil:
        return std::string("yellowDaffodil");
    case greenEmerald:
        return std::string("greenEmerald");
    case violetSignal:
        return std::string("violetSignal");
    case whiteTraffic:
        return std::string("whiteTraffic");
    case objColMax:
        return std::string("objColMax");
    default:
        return std::string("objColUnknown");
    }
}
#endif
