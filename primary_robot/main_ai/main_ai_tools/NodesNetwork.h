/*
 * Nodes.h
 *
 *  Created on: 15 mai 2015
 *      Author: Sébastien Malissard
 */

#ifndef MAIN_AI_TOOLS_NODES_H_l
#define MAIN_AI_TOOLS_NODES_H_l

//rename in environment
#include <map>
#include <iterator>

#include "MainAITypes.h"

class NodesNetwork {
    public:
        NodesNetwork(){};
        ~NodesNetwork(){};

        void addNodes(nameNode name, cfgNode cfg){
            nodes[name] = cfg;
        }
    //return all simu
        //return all REAL

        void print();

        std::map<nameNode, cfgNode> nodes;
};

//extern std::ostream& operator<<(std::ostream& out, const nameNode& a);
#endif /* MAIN_AI_TOOLS_NODES_H_ */
