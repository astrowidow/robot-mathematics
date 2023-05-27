#ifndef __VECTOR3_H__
#define __VECTOR3_H__

#include <math.h>
#include "quaternion.h"

typedef struct {
    double x;
    double y;
    double z;
} Vector3;

/**
 * @brief Scales a 3D vector by a scalar value
 * 
 * @param v The vector to be scaled
 * @param scalar The scalar value to scale the vector by
 * @return Vector3 The scaled vector
 */
Vector3 Vector3_multiply(double scalar, const Vector3 v)
{
    Vector3 result;

    result.x = v.x * scalar;
    result.y = v.y * scalar;
    result.z = v.z * scalar;

    return result;
}

/**
 * @brief Adds two 3D vectors
 * 
 * @param v1 The first vector to be added
 * @param v2 The second vector to be added
 * @return Vector3 The sum of the two vectors
 */
Vector3 Vector3_add(const Vector3 v1, const Vector3 v2)
{
    Vector3 result;

    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    result.z = v1.z + v2.z;

    return result;
}

#endif
