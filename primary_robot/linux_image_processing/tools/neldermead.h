/*
 * neldermead.h
 *
 *  Created on: 26 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_NELDERMEAD_H_
#define TOOLS_NELDERMEAD_H_

#include <functional>
#include <array>

//#define NM_DEBUG

template<typename T, typename V, int sz>
V neldermead(std::array<V, sz + 1>& x, std::function<T(V const&, int)> f, T z_end, int nb_max) {
    V xb, xr, xe, xc;
    const V x0;
    int i_max, i_MAX, i_MIN;
    int indexes[sz + 1];
    T z[sz + 1], z_xr, z_xe, z_xc, alpha = 1., gamma = 2., rho = -0.5,
            sigma = 0.5;

    for (int i = 0; i < sz + 1; i++) {
        z[i] = f(x[i], 0);
        indexes[i] = i;

#ifdef NM_DEBUG
        std::cout << "f(x[" << i << "] = " << x[i] << ") = " << z[i] << std::endl;
#endif

        if (z[i] < z_end) {
#ifdef NM_DEBUG
            std::cout << "=> ret x[" << i << "]" << std::endl;
#endif
            return x[i];
        }
    }

    for (int iter = 1; iter <= nb_max; iter++) {
#ifdef NM_DEBUG
        std::cout << "## loop " << iter << std::endl;
#endif

// tri (insertion sort)
        for (int i = 1; i < sz + 1; i++) {
            int k = indexes[i];
            int j = i;
            while (j > 0 && z[indexes[j - 1]] > z[k]) {
                indexes[j] = indexes[j - 1];
                j--;
            }
            indexes[j] = k;
        }
        i_MIN = indexes[0];
        i_max = indexes[sz - 1];
        i_MAX = indexes[sz];
#ifdef NM_DEBUG
        std::cout << "   i_MIN = " << i_MIN << ", i_max = " << i_max << ", i_MAX = " << i_MAX << std::endl;
#endif

// barycentre
        xb = V();
        for (int j = 0; j < sz + 1; j++) {
            if (j != i_MAX) {
                xb += x[j] * (T(1) / sz);
            }
        }
#ifdef NM_DEBUG
        std::cout << "   xb = (" << xb << ")" << std::endl;
#endif

// calcul point réflechi
        xr = xb + alpha * (xb - x[i_MAX]);
        z_xr = f(xr, iter);
#ifdef NM_DEBUG
        std::cout << "   f(xr = " << xr << ") = " << z_xr << std::endl;
#endif
        if (z_xr < z_end) {
#ifdef NM_DEBUG
            std::cout << "=> ret xr" << std::endl;
#endif
            return xr;
        }

// reflexion
        if (z[i_MIN] <= z_xr && z_xr < z[i_max]) {
#ifdef NM_DEBUG
            std::cout << "=> x[" << i_MAX << "] = xr" << std::endl;
#endif
            x[i_MAX] = xr;
            z[i_MAX] = z_xr;
            continue;
        }

// expansion
        if (z_xr < z[i_MIN]) {
            xe = xb + gamma * (xb - x[i_MAX]);
            z_xe = f(xe, iter);
#ifdef NM_DEBUG
            std::cout << "   f(xe = " << xe << ") = " << z_xe << std::endl;
#endif
            if (z_xe < z_end) {
#ifdef NM_DEBUG
                std::cout << "=> ret xe" << std::endl;
#endif
                return xe;
            }

            if (z_xe < z_xr) {
#ifdef NM_DEBUG
                std::cout << "=> x[" << i_MAX << "] = xe" << std::endl;
#endif
                x[i_MAX] = xe;
                z[i_MAX] = z_xe;
            }
            else {
#ifdef NM_DEBUG
                std::cout << "=> x[" << i_MAX << "] = xr" << std::endl;
#endif
                x[i_MAX] = xr;
                z[i_MAX] = z_xr;
            }
            continue;
        }

// contraction
        xc = xb + rho * (xb - x[i_MAX]);
        z_xc = f(xc, iter);
#ifdef NM_DEBUG
        std::cout << "   f(xc = " << xc << ") = " << z_xc << std::endl;
#endif
        if (z_xc < z_end) {
#ifdef NM_DEBUG
            std::cout << "=> ret xc" << std::endl;
#endif
            return xc;
        }

        if (z_xc < z_xr) {
#ifdef NM_DEBUG
            std::cout << "=> x[" << i_MAX << "] = xc" << std::endl;
#endif
            x[i_MAX] = xc;
            z[i_MAX] = z_xc;
            continue;
        }

// reduction
        for (int j = 0; j < sz + 1; j++) {
            if (j == i_MIN)
                continue;

            x[j] = x[i_MIN] + sigma * (x[j] - x[i_MIN]);
            z[j] = f(x[j], iter);
#ifdef NM_DEBUG
            std::cout << "=> f(x[" << j << "] = " << x[j] << ") = " << z[j] << std::endl;
#endif
            if (z[j] < z_end) {
#ifdef NM_DEBUG
                std::cout << "=> ret x[" << j << "]" << std::endl;
#endif
                return x[j];
            }
        }
    }

// too much iterations
#ifdef NM_DEBUG
    std::cout << "=> no convergence:" << std::endl;
#endif

    i_MIN = 0;
    z[0] = f(x[0], -1);
#ifdef NM_DEBUG
    std::cout << " z[" << i_MIN << "] = " << z[i_MIN] << std::endl;
#endif
    for (int i = 1; i < sz + 1; i++) {
        z[i] = f(x[i], -1);

#ifdef NM_DEBUG
        std::cout << " z[" << i << "] = " << z[i] << std::endl;
#endif

        if (z[i] < z[i_MIN])
            i_MIN = i;
    }
#ifdef NM_DEBUG
    std::cout << "=> ret x[" << i_MIN << "]" << std::endl;
#endif
    return x[i_MIN];
}

#endif /* TOOLS_NELDERMEAD_H_ */

