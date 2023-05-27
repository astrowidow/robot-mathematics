#ifndef __EULERANGLES_H__
#define __EULERANGLES_H__

#include <math.h>

typedef enum {
    ZYX,
    XYZ
} RotationOrder;

typedef struct {
    double x;
    double y;
    double z;
    RotationOrder type;
} EulerAngles;

#endif
