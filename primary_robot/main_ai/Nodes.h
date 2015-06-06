/*
 * Nodes.cpp
 *
 *  Created on: 24 mai 2015
 *      Author: Sebastien Malissard
 */

#ifndef NODES_H_
#define NODES_H_

#include <type_traits>
#include <array>

#include "MainAITypes.h"
#include "network_cfg.h"
#include "messages-network.h"

template <typename E>
constexpr typename std::underlying_type<E>::type castEnum(E e) {
    return static_cast<typename std::underlying_type<E>::type>(e);
}

/*
 * Address and role for each configuration nodes.
 */

class Nodes{
    public:
        Nodes(){
            _nodes[castEnum(nameNode::MAIN_AI)][castEnum(envNode::SIMU)] =
                                    {envNode::SIMU, ADDRD1_MAIN_AI_SIMU, ROLE_PRIM_AI};
            _nodes[castEnum(nameNode::MAIN_AI)][castEnum(envNode::REAL)] =
                                    {envNode::REAL, ADDR_PRIM_AI_DFLT, ROLE_PRIM_AI};

            _nodes[castEnum(nameNode::ARDUINO_IO)][castEnum(envNode::SIMU)] =
                                    {envNode::SIMU, 0, ROLE_UNDEFINED};
            _nodes[castEnum(nameNode::ARDUINO_IO)][castEnum(envNode::REAL)] =
                                    {envNode::REAL, ADDRI_MAIN_IO, ROLE_UNDEFINED};

            _nodes[castEnum(nameNode::TURRET)][castEnum(envNode::SIMU)] =
                                    {envNode::SIMU, 0, ROLE_UNDEFINED};
            _nodes[castEnum(nameNode::TURRET)][castEnum(envNode::REAL)] =
                                    {envNode::SIMU, ADDRI_MAIN_TURRET, ROLE_UNDEFINED};

            _nodes[castEnum(nameNode::PROP)][castEnum(envNode::SIMU)] =
                                    {envNode::SIMU, ADDRD1_MAIN_PROP_SIMU, ROLE_PRIM_PROPULSION};
            _nodes[castEnum(nameNode::PROP)][castEnum(envNode::REAL)] =
                                    {envNode::REAL, ADDR_PRIM_PROP_DFLT, ROLE_PRIM_PROPULSION};

            _nodes[castEnum(nameNode::DEBUG_BRIDGE)][castEnum(envNode::SIMU)] =
                                    {envNode::SIMU, ADDRD1_DBGBRIDGE, ROLE_UNDEFINED};
            _nodes[castEnum(nameNode::DEBUG_BRIDGE)][castEnum(envNode::REAL)] =
                                    {envNode::REAL, ADDRD1_DBGBRIDGE, ROLE_UNDEFINED};

        }
        ~Nodes(){};

        cfgNode getCfg(const nameNode name, const envNode env){
            return _nodes[castEnum(name)][castEnum(env)];
        }

    private:
        std::array<std::array<cfgNode,castEnum(envNode::SIZE)>, castEnum(nameNode::SIZE)> _nodes;
};

#endif



