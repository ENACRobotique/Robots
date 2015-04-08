/*
 ============================================================================
 Name        : PWL_approx.c
 Author      :
 Version     :
 Description :
 ============================================================================
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <float.h>

typedef double (*pfunc1D)(double);

#define SIGN(v) (((v)>=0) - ((v)<0))

double u(double t){
    return (0.05 - 0.2) / (0.3 - 0.15*t);
}

double up(double t){
    return (0.05 - 0.2) * 0.15 / pow(0.3 - 0.15*t, 2);
}

double f(double t){
    return atan(u(t));
}

double fp(double t){
    return up(t)/(1 + pow(u(t), 2));
}


double f_1ox(double t){
    return 1/t;
}

double fp_1ox(double t){
    return -1/pow(t, 2);
}


int lin_approx(pfunc1D f, pfunc1D fp, double t1, double t2, int nb_seg, double *epsilon_final, double alphai[], double betai[]){
    int j=0,i;
    double delta=1, t[nb_seg][2], alpha[nb_seg+1], epsilon[nb_seg][2], epsilon_max=0,epsilon_min=0,epsilon_max_old=DBL_MAX,param_delta=0.1;
    int epsilon_max_sign;
    alpha[0] = t1;
    alpha[nb_seg] = t2;

    for (i=0;i<=nb_seg;i++){
        t[i][j] = t[i][!j] = alpha[0] + ((i + 0.5)/nb_seg)*(alpha[nb_seg] - alpha[0]);
    }

    int nb_iter = 0;
    while (1){
        nb_iter++;

        for (i=0;i<=nb_seg;i++){
            printf("t[%i][%i]=%.3g\n", i, j, t[i][j]);
        }

//        getchar();

        for (i=1;i<nb_seg;i++){
            alpha[i]=( f(t[i-1][j]) - f(t[i][j]) + fp(t[i][j])*t[i][j] - fp(t[i-1][j])*t[i-1][j] ) / ( fp(t[i][j]) - fp(t[i-1][j]) );
        }

        for (i=0;i<=nb_seg;i++){
            printf("alpha[%i]=%.3g\n", i, alpha[i]);
        }

//        getchar();

        for (i=0;i<=nb_seg;i++){
            epsilon[i][j] = fp(t[i][j])*(alpha[i] - t[i][j]) + f(t[i][j]) - f(alpha[i]);

            printf("epsilon[%i][%i]=%.3g\n", i, j, epsilon[i][j]);
        }

//        getchar();

        for (i=0;i<nb_seg;i++){
            double feij = fabs(epsilon[i][j]);

            if(i == 0){
                epsilon_min = epsilon_max = feij;
                epsilon_max_sign = SIGN(epsilon[i][j]);
            }
            else if(feij > epsilon_max){
                epsilon_max = feij;
                epsilon_max_sign = SIGN(epsilon[i][j]);
            }
            else if(feij < epsilon_min){
                epsilon_min = feij;
            }
        }

        printf("epsilon_max=%.3g (%i)\n", epsilon_max, epsilon_max_sign);
        printf("epsilon_min=%.3g\n", epsilon_min);

        if (epsilon_max/epsilon_min - 1. < param_delta){
            *epsilon_final = 0.5*epsilon_max*epsilon_max_sign;

            for(i=0;i<=nb_seg;i++){
                betai[i] = fp(t[i][j])*(alpha[i] - t[i][j]) + f(t[i][j]) - *epsilon_final;
                alphai[i] = t[i][j];
            }

            break;
        }

        if (epsilon_max > epsilon_max_old){
            j=!j;
            delta /= 2;
        }

        printf("delta=%.3g\n", delta);

        epsilon_max_old = epsilon_max;

        for (i=0;i<nb_seg;i++){
            t[i][!j] = t[i][j] + delta * (epsilon[i+1][j] - epsilon[i][j]) / ( (epsilon[i+1][j] / (alpha[i+1] - t[i][j])) + (epsilon[i][j] / (t[i][j] - alpha[i])));
        }

        j=!j;
    }

    printf("nb_iter=%i\n", nb_iter);

    return 0;
}

double g(double t, double alphai[], double betai[]){
    int i = 0;
    while(t > alphai[i+1]){
        i++;
    }

    double a = (betai[i+1] - betai[i])/(alphai[i+1] - alphai[i]);
    double b = betai[i] - a*alphai[i];

    return a*t+b;
}

int main(){
    int ret, i;

    double epsilon;
    int nb_seg = 8;
    double alphai[nb_seg+1];
    double betai[nb_seg+1];

//    ret = lin_approx(f, fp, 0., 1.,  nb_seg, &epsilon, alphai, betai);
    ret = lin_approx(f_1ox, fp_1ox, 1., 2.,  nb_seg, &epsilon, alphai, betai);

    double t = 1;
    for(t = 1; t < 2; t+=0.001){
        printf("%.10g;%.10g\n", t, g(t, alphai, betai) - f_1ox(t));
    }

    for (i=0;i<=nb_seg;i++){
        printf("alpha[%i]=%.10g\n", i, alphai[i]);
    }
    for (i=0;i<=nb_seg;i++){
        printf("beta[%i]=%.10g\n", i, betai[i]);
    }
    printf("epsilon=%.10g\n", epsilon);

    return ret<0;
}

