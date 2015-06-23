/*
 * Nodes.cpp
 *
 *  Created on: 20 mai 2015
 *      Author: seb
 */

#include <NodesNetwork.h>
#include "tools.h"



void NodesNetwork::print(){
    logs << INFO << "Primary robot environment :\n";
    for(std::map<nameNode, cfgNode>::iterator it=nodes.begin() ; it != nodes.end() ; ++it){
        logs << "              " <<  it->first << " : " << it->second.env << "\n";

    }
}

