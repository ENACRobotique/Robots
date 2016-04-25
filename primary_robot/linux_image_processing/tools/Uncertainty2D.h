/*
 * Uncertainty2D.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_UNCERTAINTY2D_H_
#define TOOLS_UNCERTAINTY2D_H_

#ifdef USE_BOTNET
#include <messages-position.h>
#endif

template<typename T>
class Uncertainty2D {
public:
    T a_var, b_var, a_angle, theta; // (cm x cm x rad x rad)

    Uncertainty2D(T a_var, T b_var, T a_angle, T theta) :
        a_var(a_var), b_var(b_var), a_angle(a_angle), theta(theta) {
    }

#ifdef USE_BOTNET
    Uncertainty2D(const s2DPAUncert& u) :
        a_var(u.a_var), b_var(u.b_var), a_angle(u.a_angle), theta(u.theta_var) {
    }
#endif
};

#endif /* TOOLS_UNCERTAINTY2D_H_ */
