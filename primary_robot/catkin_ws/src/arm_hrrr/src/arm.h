#ifndef ARM_H
#define ARM_H


typedef struct sArmPose{
    double x;
    double y;
    double z;
    double beta;
}sArmPose;

typedef enum eOrienTool{
    vertical,
    horizontal,
    eOrienToolMax
}eOrienTool;

typedef enum eStaticConf{
    rest,
    sStaticConfMax
}eStaticConf;

#endif // ARM_H
