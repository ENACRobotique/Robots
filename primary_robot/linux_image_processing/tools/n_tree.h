/*
 * n_tree.h
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_N_TREE_H_
#define TOOLS_N_TREE_H_

#include <array>
#include <functional>
#include <map>

//#define NT_DEBUG

template<typename energy, typename status, int sz>
status n_tree(status const& x, energy z_end, unsigned nb_max, unsigned nb_store,
        std::function<energy(status const&)> const& ef,
        std::function<std::array<status, 1 << sz>(status const&, int iter)> const& nf) {

    std::multimap<energy, status> best;
    best.insert({ef(x), x});

    for (unsigned iter = 0; iter < nb_max; iter++) {
        while(best.size() > nb_store){
            best.erase(std::prev(best.end()));
        }

        for(std::pair<const energy, status> const& el : best) {
            std::array<status, 1<<sz> children(nf(el.second, iter));

            for(status const& st : children){
                energy e(ef(st));

                if(best.upper_bound(e) != best.end()){
                    best.insert({e, st});
                }
            }
        }
    }

    return best.begin()->second;
}

#endif /* TOOLS_N_TREE_H_ */

