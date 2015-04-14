/*
 ============================================================================
 Name        : PWL_approx.c
 Author      :
 Version     :
 Description : Implémentation de l'algorithme d'approximation linéaire par morceaux décrit ici:
     http://www.iaeng.org/publication/WCECS2008/WCECS2008_pp1191-1194.pdf
 ============================================================================
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <float.h>

//#define DEBUG_LIN_APPROX 1

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
    return 1./t;
}

double fp_1ox(double t){
    return -1./pow(t, 2);
}

int lin_approx(pfunc1D f, pfunc1D fp, double t1, double t2, int nb_seg, double *epsilon_final, double alphai[], double betai[]){
    int j=0,i;
    double delta=1, t[nb_seg][2], alpha[nb_seg+1][2], epsilon[nb_seg+1][2], epsilon_max=0,epsilon_min=0,epsilon_max_old=DBL_MAX,param_delta=0.01;
    int epsilon_max_sign;
    alpha[0][j] = alpha[0][!j] = t1;
    alpha[nb_seg][j] = alpha[nb_seg][!j] = t2;

    for (i=0;i<nb_seg;i++){
        t[i][j] = alpha[0][j] + ((i + 0.5)/nb_seg)*(alpha[nb_seg][j] - alpha[0][j]);
    }

#if DEBUG_LIN_APPROX > 0
    int nb_iter = 0;
#endif
    while (1){
#if DEBUG_LIN_APPROX > 0
        nb_iter++;
#endif

#if DEBUG_LIN_APPROX > 1
        for (i=0;i<nb_seg;i++){
            printf("t[%i][%i]=%.6g\n", i, j, t[i][j]);
        }
#if DEBUG_LIN_APPROX > 2
        getchar();
#endif
#endif

        for (i=1;i<nb_seg;i++){
            alpha[i][j]=( f(t[i-1][j]) - f(t[i][j]) + fp(t[i][j])*t[i][j] - fp(t[i-1][j])*t[i-1][j] ) / ( fp(t[i][j]) - fp(t[i-1][j]) );
        }

#if DEBUG_LIN_APPROX > 1
        for (i=0;i<=nb_seg;i++){
            printf("alpha[%i][%i]=%.6g\n", i, j, alpha[i][j]);
        }
#if DEBUG_LIN_APPROX > 2
        getchar();
#endif
#endif

        for (i=0;i<nb_seg;i++){
            epsilon[i][j] = fp(t[i][j])*(alpha[i][j] - t[i][j]) + f(t[i][j]) - f(alpha[i][j]);
        }
        epsilon[nb_seg][j] = fp(t[nb_seg-1][j])*(alpha[nb_seg][j] - t[nb_seg-1][j]) + f(t[nb_seg-1][j]) - f(alpha[nb_seg][j]);

#if DEBUG_LIN_APPROX > 1
        for (i=0;i<=nb_seg;i++){
            printf("epsilon[%i][%i]=%.6g\n", i, j, epsilon[i][j]);
        }
#if DEBUG_LIN_APPROX > 2
        getchar();
#endif
#endif

        for (i=0;i<=nb_seg;i++){
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

#if DEBUG_LIN_APPROX > 1
        printf("epsilon_max=%.6g (%c)\n", epsilon_max, epsilon_max_sign>0?'+':'-');
        printf("epsilon_min=%.6g\n", epsilon_min);
#endif

        if (epsilon_max/epsilon_min - 1. < param_delta){
            *epsilon_final = 0.5*epsilon_max*epsilon_max_sign;

            for(i=0;i<nb_seg;i++){
                betai[i] = fp(t[i][j])*(alpha[i][j] - t[i][j]) + f(t[i][j]) - *epsilon_final;
                alphai[i] = alpha[i][j];
            }
            betai[nb_seg] = fp(t[nb_seg-1][j])*(alpha[nb_seg][j] - t[nb_seg-1][j]) + f(t[nb_seg-1][j]) - *epsilon_final;
            alphai[nb_seg] = alpha[nb_seg][j];

            break;
        }

        if (epsilon_max > epsilon_max_old){
            j=!j;
            delta /= 2;
        }

        epsilon_max_old = epsilon_max;

#if DEBUG_LIN_APPROX > 1
        printf("delta=%.6g\n", delta);
#endif

        for (i=0;i<nb_seg;i++){
            t[i][!j] = t[i][j] + delta * (epsilon[i+1][j] - epsilon[i][j]) / ( (epsilon[i+1][j] / (alpha[i+1][j] - t[i][j])) + (epsilon[i][j] / (t[i][j] - alpha[i][j])));
        }
        j=!j;
    }

#if DEBUG_LIN_APPROX > 0
    printf("nb_iter=%i\n", nb_iter);
    return nb_iter;
#else
    return 0;
#endif
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

#if DEBUG_LIN_APPROX > 0
    {
        int ret;
        double epsilon;
        int nb_seg;
        FILE* fout = fopen("out_stats.csv", "wb+");
        for(nb_seg = 1; nb_seg < 20; nb_seg++){
            printf("nb_seg=%i, ", nb_seg);

            double alphai[nb_seg+1];
            double betai[nb_seg+1];

            ret = lin_approx(f, fp, 0., 1.,  nb_seg, &epsilon, alphai, betai);

            printf("epsilon=%.6g\n", epsilon);

            fprintf(fout, "%i;%i;%.10g\n", nb_seg, ret, epsilon);
        }
        fclose(fout);
    }
#endif

    { // cas du papier
        int i;
        double epsilon;
        int nb_seg = 8;
        double alphai[nb_seg+1];
        double betai[nb_seg+1];

        lin_approx(f_1ox, fp_1ox, 1., 2.,  nb_seg, &epsilon, alphai, betai);

        FILE* fout = fopen("out_1ox.csv", "wb+");
        double t;
        for(t = 1; t < 2; t+=0.001){
            fprintf(fout, "%.10g;%.10g\n", t, g(t, alphai, betai) - f_1ox(t));
        }
        fclose(fout);

        for (i=0;i<=nb_seg;i++){
            printf("alpha[%i]=%.10g\n", i, alphai[i]);
        }
        for (i=0;i<=nb_seg;i++){
            printf("beta[%i]=%.10g\n", i, betai[i]);
        }
        printf("epsilon=%.10g\n", epsilon);
    }

    return EXIT_SUCCESS;
}

