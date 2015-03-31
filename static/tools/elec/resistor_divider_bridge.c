// resistor divider bridge values finder
// by Ludovic Lacoste <ludolacost@gmail.com>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#define BIT(b) (1<<(b))

typedef enum{
    E24 = BIT(0),
    E12 = BIT(1)
} eSerie;

struct{
    int val;
    eSerie serie;
} list[] = {
    {10, E24|E12},
    {11, E24},
    {12, E24|E12},
    {13, E24},
    {15, E24|E12},
    {16, E24},
    {18, E24|E12},
    {20, E24},
    {22, E24|E12},
    {24, E24},
    {27, E24|E12},
    {30, E24},
    {33, E24|E12},
    {36, E24},
    {39, E24|E12},
    {43, E24},
    {47, E24|E12},
    {51, E24},
    {56, E24|E12},
    {62, E24},
    {68, E24|E12},
    {75, E24},
    {82, E24|E12},
    {91, E24}
};
#define LIST_LEN (sizeof(list)/sizeof(*list))

typedef struct {
    int mul_R1;
    int val_R1;
    int R1;

    int mul_R2;
    int val_R2;
    int R2;

    int sum;
    float val;
    float err;
} params;

static inline int getNext(int i, eSerie serie){
    do{
        i++;
    }while(i < LIST_LEN && !(list[i].serie & serie));

    return i;
}

static inline void eval(params *p, float desired_val){
    p->R1 = p->mul_R1*list[p->val_R1].val;
    p->R2 = p->mul_R2*list[p->val_R2].val;

    p->sum = p->R1 + p->R2;

    p->val = (float)p->R1 / ((float)p->R1 + (float)p->R2);

    p->err = 100.*(p->val - desired_val)/desired_val;
}

int main(int argc, char *argv[]){
    eSerie serie = E12;
    float desired_val, desired_sum, voltage = 5.;
    int mul_min = 1, mul_max = 1000;
    params p, p_min_p, p_min_n;

    if(argc != 3){
        printf("bad arguments, see code!\n");
        exit(1);
    }

    desired_val = strtof(argv[1], NULL);
    if(desired_val <= 0.){
        printf("bad arguments, see code!\n");
        exit(1);
    }

    desired_sum = strtof(argv[2], NULL);
    if(desired_sum <= 0.){
        printf("bad arguments, see code!\n");
        exit(1);
    }

    p_min_p.val = 0.;
    p_min_n.val = 0.;

    // R1
    for(p.mul_R1 = mul_min; p.mul_R1 <= mul_max; p.mul_R1*=10){
        p.val_R1 = -1;
        for(p.val_R1 = getNext(p.val_R1, serie); p.val_R1 < LIST_LEN; p.val_R1 = getNext(p.val_R1, serie)){
            // R2
            for(p.mul_R2 = mul_min; p.mul_R2 <= mul_max; p.mul_R2*=10){
                p.val_R2 = -1;
                for(p.val_R2 = getNext(p.val_R2, serie); p.val_R2 < LIST_LEN; p.val_R2 = getNext(p.val_R2, serie)){
                    eval(&p, desired_val);

                    if(p.val > desired_val && (!p_min_p.val || fabs(p.err) < fabs(p_min_p.err) || (fabs(p.err) == fabs(p_min_p.err) && fabs(p.sum - desired_sum) < fabs(p_min_p.sum - desired_sum)))){
                        memcpy(&p_min_p, &p, sizeof(p_min_p));
                    }

                    if(p.val < desired_val && (!p_min_n.val || fabs(p.err) < fabs(p_min_n.err) || (fabs(p.err) == fabs(p_min_n.err) && fabs(p.sum - desired_sum) < fabs(p_min_n.sum - desired_sum)))){
                        memcpy(&p_min_n, &p, sizeof(p_min_n));
                    }
                }
            }
        }
    }

    printf("Best result for %.4f/%.2f:\n", desired_val, desired_sum);
    printf("voltage=%.2fV\n", voltage);
    if(p_min_p.val != 0){
        printf("  value superior:\n");
        printf("    R1    =%i\n", p_min_p.R1);
        printf("    R2    =%i\n", p_min_p.R2);
        printf("    R1+R2 =%i\n", p_min_p.sum);
        printf("    i     =%.2fmA\n", 1000.*voltage/(float)p_min_p.sum);
        printf("    v_out =%.2fV\n", voltage*p_min_p.val);
        printf("    val   =%.4f\n", p_min_p.val);
        printf("    error =%.4f%%\n", p_min_p.err);
    }
    else{
        printf("  no solution with superior value\n");
    }
    if(p_min_p.val != 0){
        printf("  value inferior:\n");
        printf("    R1    =%i\n", p_min_n.R1);
        printf("    R2    =%i\n", p_min_n.R2);
        printf("    R1+R2 =%i\n", p_min_n.sum);
        printf("    i     =%.2fmA\n", 1000.*voltage/(float)p_min_n.sum);
        printf("    v_out =%.2fV\n", voltage*p_min_n.val);
        printf("    val   =%.4f\n", p_min_n.val);
        printf("    error =%.4f%%\n", p_min_n.err);
    }
    else{
        printf("  no solution with superior value\n");
    }

    return 0;
}

