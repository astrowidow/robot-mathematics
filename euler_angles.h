#ifndef __EULERANGLES_H__
#define __EULERANGLES_H__

#ifndef M_PI
#define M_PI (3.141592653589793)
#endif

#include <math.h>

typedef enum {
    ZYX,
    XYZ
} RotationOrder;

typedef enum {
    DEGREES,
    RADIANS
} AngleUnit;

typedef struct {
    double x;
    double y;
    double z;
    RotationOrder type;
    AngleUnit unit;
} EulerAngles;

/**
 * @brief Function to create euler angles
 * 
 * @param x Value of the x component
 * @param y Value of the y component
 * @param z Value of the z component
 * @param type rotational sequence
 * @param unit degree or radian
 * @return euler angles
 */
EulerAngles EulerAngles_create(const double x, const double y, const double z, const RotationOrder type, const AngleUnit unit)
{
    EulerAngles euler;

    euler.x = x;
    euler.y = y;
    euler.z = z;
    euler.type = type;
    euler.unit = unit;

    return euler;
}

/**
 * @brief Convert EulerAngles from degrees to radians
 * 
 * @param angles The EulerAngles in degrees
 * @return EulerAngles The EulerAngles in radians
 */
EulerAngles EulerAngles_radians(const EulerAngles angles)
{
    const double DEG_TO_RAD = M_PI / 180.0;
    EulerAngles anglesInRadians;
    anglesInRadians = angles;

    if(angles.unit == DEGREES) {
        anglesInRadians.x *= DEG_TO_RAD;
        anglesInRadians.y *= DEG_TO_RAD;
        anglesInRadians.z *= DEG_TO_RAD;
        anglesInRadians.unit = RADIANS;
    } else {
        anglesInRadians.unit = DEGREES;
    }

    return anglesInRadians;
}

/**
 * @brief Convert EulerAngles from radians to degrees
 * 
 * @param angles The EulerAngles in radians
 * @return EulerAngles The EulerAngles in degrees
 */
EulerAngles EulerAngles_degrees(const EulerAngles angles)
{
    const double RAD_TO_DEG = 180.0 / M_PI;
    EulerAngles anglesInDegrees;
    anglesInDegrees = angles;

    if(angles.unit == RADIANS) {
        anglesInDegrees.x *= RAD_TO_DEG;
        anglesInDegrees.y *= RAD_TO_DEG;
        anglesInDegrees.z *= RAD_TO_DEG;
        anglesInDegrees.unit = DEGREES;
    } else {
        anglesInDegrees.unit = RADIANS;
    }

    return anglesInDegrees;
}

#endif
