// LPC uart baudrate conf finder
// by Ludovic Lacoste <ludolacost@gmail.com>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

// build with: gcc -o baudrate_conf_finder baudrate_conf_finder.c -lm -Os

/* some results:

Best result for 115200.00bauds:
  PCLK       =30000000Hz
  baudrate superior:
    U0DLM    =0
    U0DLL    =5
    MULVAL   =4
    DIVADDVAL=9
    baudrate =115384.62
    error    =0.1603% (0.1597% truncated)
  baudrate inferior:
    U0DLM    =0
    U0DLL    =6
    MULVAL   =7
    DIVADDVAL=12
    baudrate =115131.58
    error    =-0.0594% (-0.0599% truncated)

Best result for 230400.00bauds:
  PCLK       =30000000Hz
  baudrate superior:
    U0DLM    =0
    U0DLL    =5
    MULVAL   =8
    DIVADDVAL=5
    baudrate =230769.23
    error    =0.1603% (0.1602% truncated)
  baudrate inferior:
    U0DLM    =0
    U0DLL    =3
    MULVAL   =7
    DIVADDVAL=12
    baudrate =230263.16
    error    =-0.0594% (-0.0595% truncated)

Best result for 460800.00bauds:
  PCLK       =30000000Hz
  baudrate superior:
    U0DLM    =0
    U0DLL    =3
    MULVAL   =3
    DIVADDVAL=1
    baudrate =468750.00
    error    =1.7253% (1.7253% truncated)
  baudrate inferior:
    U0DLM    =0
    U0DLL    =3
    MULVAL   =14
    DIVADDVAL=5
    baudrate =460526.31
    error    =-0.0594% (-0.0595% truncated)

Best result for 500000.00bauds:
  PCLK       =30000000Hz
  baudrate superior:
    U0DLM    =0
    U0DLL    =3
    MULVAL   =13
    DIVADDVAL=3
    baudrate =507812.50
    error    =1.5625% (1.5624% truncated)
  baudrate inferior:
    U0DLM    =0
    U0DLL    =3
    MULVAL   =15
    DIVADDVAL=4
    baudrate =493421.06
    error    =-1.3158% (-1.3158% truncated)

Best result for 576000.00bauds:
  PCLK       =30000000Hz
  baudrate superior:
    U0DLM    =0
    U0DLL    =3
    MULVAL   =12
    DIVADDVAL=1
    baudrate =576923.06
    error    =0.1603% (0.1602% truncated)
  baudrate inferior:
    U0DLM    =0
    U0DLL    =3
    MULVAL   =11
    DIVADDVAL=1
    baudrate =572916.69
    error    =-0.5353% (-0.5354% truncated)
*/

// manual-config
#define USEFRACDIV

// auto-config
#ifdef USEFRACDIV
#define DL_MIN (3)
#else
#define DL_MIN (1)
#endif

typedef struct {
    uint16_t U0DLM; // 0<=U0DLM<=255
    uint16_t U0DLL; // 0<=U0DLL<=255 (if U0DLM is 0, U0DLL>=3)
    uint8_t MULVAL; // 1<=MULVAL<=15
    uint8_t DIVADDVAL; // 0<=DIVADDVAL<=15

    float baudrate;
    float err;
    float err_trunc;
} params;

int main(int argc, char *argv[]){
    float desired_baudrate;
    const uint32_t PCLK = 30000000;
    params p, p_min_p, p_min_n;

    if(argc != 2){
        printf("Usage:\n\t%s <desired baudrate>\n", argc>0?argv[0]:"baudrate_conf_finder");
        exit(1);
    }

    desired_baudrate = strtof(argv[1], NULL);
    if(desired_baudrate <= 0){
        printf("bad baudrate\n");
        exit(1);
    }

    p_min_p.baudrate = 0.;
    p_min_n.baudrate = 0.;

#ifndef USEFRACDIV
    p.MULVAL = 1;
    p.DIVADDVAL = 0;
#endif

    for(p.U0DLM = 0; p.U0DLM <= 255; p.U0DLM++){
        for(p.U0DLL = p.U0DLM>0?0:DL_MIN; p.U0DLL <= 255; p.U0DLL++){
#ifdef USEFRACDIV
            for(p.MULVAL = 1; p.MULVAL <= 15; p.MULVAL++){
                for(p.DIVADDVAL = 0; p.DIVADDVAL <= 15; p.DIVADDVAL++){
#endif
                    p.baudrate = ((float)PCLK*(float)p.MULVAL)/(16.*((float)p.U0DLM*256. + (float)p.U0DLL)*((float)p.MULVAL + (float)p.DIVADDVAL));
                    p.err = 100.*(p.baudrate - desired_baudrate)/desired_baudrate;
                    p.err_trunc = 100.*(floorf(p.baudrate) - desired_baudrate)/desired_baudrate;

                    if(p.baudrate > desired_baudrate && (!p_min_p.baudrate || fabs(p.err) < fabs(p_min_p.err))){
                        memcpy(&p_min_p, &p, sizeof(p_min_p));
                    }

                    if(p.baudrate < desired_baudrate && (!p_min_n.baudrate || fabs(p.err) < fabs(p_min_n.err))){
                        memcpy(&p_min_n, &p, sizeof(p_min_n));
                    }
#ifdef USEFRACDIV
                }
            }
#endif
        }
    }

    printf("Best result for %.2fbauds:\n", desired_baudrate);
    printf("  PCLK       =%uHz\n", PCLK);
    if(p_min_p.baudrate != 0){
        printf("  baudrate superior:\n");
        printf("    U0DLM    =%hu\n", p_min_p.U0DLM);
        printf("    U0DLL    =%hu\n", p_min_p.U0DLL);
#ifdef USEFRACDIV
        printf("    MULVAL   =%hu\n", p_min_p.MULVAL);
        printf("    DIVADDVAL=%hu\n", p_min_p.DIVADDVAL);
#endif
        printf("    baudrate =%.2f\n", p_min_p.baudrate);
        printf("    error    =%.4f%% (%.4f%% truncated)\n", p_min_p.err, p_min_p.err_trunc);
    }
    else{
        printf("  no solution with superior baudrate\n");
    }
    if(p_min_n.baudrate != 0){
        printf("  baudrate inferior:\n");
        printf("    U0DLM    =%hu\n", p_min_n.U0DLM);
        printf("    U0DLL    =%hu\n", p_min_n.U0DLL);
#ifdef USEFRACDIV
        printf("    MULVAL   =%hu\n", p_min_n.MULVAL);
        printf("    DIVADDVAL=%hu\n", p_min_n.DIVADDVAL);
#endif
        printf("    baudrate =%.2f\n", p_min_n.baudrate);
        printf("    error    =%.4f%% (%.4f%% truncated)\n", p_min_n.err, p_min_n.err_trunc);
    }
    else{
        printf("  no solution with inferior baudrate\n");
    }

    return 0;
}

