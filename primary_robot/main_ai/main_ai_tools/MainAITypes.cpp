/*
 * MainAITypes.cpp
 *
 *  Created on: 22 mai 2015
 *      Author: Sebastien Malissard
 */

#include "MainAITypes.h"

std::ostream& operator<<(std::ostream& out, const envNode& a){
    switch(a){
        case envNode::REAL:
            out << "REAL";
            break;
        case envNode::SIMU:
            out << "SIMU";
            break;
        case envNode::NONE:
            out << "NONE";
            break;
        default:
            out << "Unknown envNode";
    }
    return out;
}

std::ostream& operator<<(std::ostream& out, const nameNode& a){
    switch(a){
        case nameNode::MAIN_AI:
            out << "MAIN_AI     ";
            break;
        case nameNode::PROP:
            out << "PROP        ";
            break;
        case nameNode::MONITORING:
            out << "MONITORING  ";
            break;
        case nameNode::ARDUINO_IO:
            out << "ARDUINO_IO  ";
            break;
        case nameNode::DEBUG_BRIDGE:
            out << "DEBUG_BRIDGE";
            break;
        case nameNode::TURRET:
            out << "TURRET      ";
            break;
        case nameNode::HMI:
            out << "HMI         ";
            break;
        default:
            out << "Unknown nameNode";
    }
    return out;
}

std::ostream& operator<<(std::ostream& out, const addrType& a){
    switch(a){
        case addrType::BN_ADDR:
            out << "BN_ADDR";
            break;
        case addrType::ROLE:
            out << "ROLE   ";
            break;
        case addrType::NONE:
            out << "NONE   ";
            break;
        default:
            out << "Unknown addrType";
    }
    return out;
}
