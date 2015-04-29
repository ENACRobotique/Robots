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

#define NT_DEBUG 0

#if NT_DEBUG > 0
#   include <iostream>
#endif

/**
 * Number of calls of the energy function: (1<<sz)*(nb_store*(nb_max-1) + 1) + 1
 */
template<typename energy, typename status, int sz>
status n_tree(status const& x, energy z_end, unsigned nb_max, unsigned nb_store,
        std::function<energy(status const&)> const& ef,
        std::function<std::array<status, 1 << sz>(status const&, int iter)> const& nf) {

    std::multimap<energy, status> best;
    best.insert( { ef(x), x });

    for (unsigned iter = 0; iter < nb_max; iter++) {
        while (best.size() > nb_store) {
            best.erase(std::prev(best.end()));
        }
        std::multimap<energy, status> nbest;

        for (std::pair<const energy, status> const& el : best) {
            std::array<status, 1 << sz> children(nf(el.second, iter));

            for (status const& st : children) {
                energy e(ef(st));

                if (e < z_end) {
                    return st;
                }

                if (nbest.upper_bound(e) != nbest.end() || nbest.size() < nb_store) {
#if NT_DEBUG > 1
    std::cout << "inserting, e: " << e << ", pt: " << st << std::endl;
#endif

                    nbest.insert( { e, st });
                }
            }
        }

        best = nbest;
    }

#if NT_DEBUG > 0
    std::cout << "last values:" << std::endl;
    for (std::pair<const energy, status> const& el : best) {
        std::cout << "e: " << el.first << ", pt: " << el.second << std::endl;
    }
#endif

    return best.begin()->second;
}

#endif /* TOOLS_N_TREE_H_ */

