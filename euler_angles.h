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

/**
 * @brief Function to create euler angles
 * 
 * @param x Value of the x component
 * @param y Value of the y component
 * @param z Value of the z component
 * @param type rotational sequence
 * @return euler angles
 */
EulerAngles EulerAngles_create(const double x, const double y, const double z, const RotationOrder type)
{
    EulerAngles euler;

    euler.x = x;
    euler.y = y;
    euler.z = z;
    euler.type = type;

    return euler;
}

#endif
