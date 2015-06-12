/*
 * MainAITypes.h
 *
 *  Created on: 22 mai 2015
 *      Author: Sebastien Malissard
 */

#ifndef MAIN_AI_TOOLS_MAINAITYPES_H_
#define MAIN_AI_TOOLS_MAINAITYPES_H_

#include <ostream>

#include "message_header.h"

enum class envNode{
    REAL,
    SIMU,
    NONE,
    SIZE
};

enum class nameNode{
    MAIN_AI,
    PROP,
    MONITORING,
    ARDUINO_IO,
    DEBUG_BRIDGE,
    TURRET,
    HMI,
    SIZE
};

enum class addrType{
    BN_ADDR,
    ROLE,
    NONE
};


typedef struct{
    envNode     env;
    bn_Address  addr;
    char        role;
}cfgNode;

extern std::ostream& operator<<(std::ostream& out, const envNode& a);
extern std::ostream& operator<<(std::ostream& out, const nameNode& a);
extern std::ostream& operator<<(std::ostream& out, const addrType& a);

#endif /* MAIN_AI_TOOLS_MAINAITYPES_H_ */
