
#ifndef __HMAT_H__
#define __HMAT_H__

#include <math.h>
#include "quaternion.h"
#include "vector3.h"
#include "euler_angles.h"
#include "dcm.h"

typedef struct {
    Vector3 pos;
    Quaternion quat;
} Hmat;

Hmat Hmat_create(Quaternion quat, Vector3 pos)
{
    Hmat hmat;

    hmat.pos = pos;
    hmat.quat = quat;

    return hmat;
}

/**
 * @brief Function to compute the inverse of a homogeneous matrix
 *
 * @param hmat Homogeneous matrix to compute the inverse for
 * @return Inverse homogeneous matrix
 */
Hmat Hmat_inverse(const Hmat hmat)
{
    Hmat inverse;

    // Compute the inverse hmat
    inverse.quat = Quaternion_inverse(hmat.quat);
    inverse.pos = Vector3_multiply(-1.0, Quaternion_transformVector(inverse.quat, hmat.pos));
    return inverse;
}

/**
 * @brief Function to compute the Hmat H of two homogeneous matrices
 * 
 * @param h_a2b First hmat representing the initial conversion
 * @param h_b2c Second hmat representing the additional conversion
 * @return Composite hmat representing the combined conversion
 */
Hmat Hmat_composite(const Hmat h_a2b, const Hmat h_b2c)
{
    Hmat h_a2c;
    
    h_a2c.quat = Quaternion_composite(h_a2b.quat, h_b2c.quat);
    h_a2c.pos = Vector3_add(Quaternion_transformVector(h_b2c.quat, h_a2b.pos), h_b2c.pos);
    return h_a2c;
}

/**
 * @brief Compute the relative hmat from h_i2a to h_i2c
 * 
 * @param h_i2c The end orientation (c)
 * @param h_i2a The start orientation (a)
 * @return h_a2c
 */
Hmat Hmat_relative(const Hmat h_i2c, const Hmat h_i2a)
{
    Hmat h_a2i;
    h_a2i = Hmat_inverse(h_i2a);
    return Hmat_composite(h_a2i, h_i2c);
}

/**
 * @brief Wraps an angle in radians to the range -π to π (-180 to 180 degrees)
 * 
 * @param angle The angle in radians
 * @return double The angle wrapped to the range -π to π
 */
double wrapAngleToPiRad(double angle)
{
    const double TWO_PI = 2.0 * M_PI;

    angle = fmod(angle, TWO_PI);
    if (angle <= -M_PI)
    {
        angle += TWO_PI;
    }
    else if (angle > M_PI)
    {
        angle -= TWO_PI;
    }

    return angle;
}

/**
 * @brief Compute the average hmat from h_i2a and h_i2b
 * 
 * @param h_i2b 
 * @param h_i2a 
 * @return h_i2c c is a midpoint of a and b
 */
Hmat Hmat_average(const Hmat h_i2b, const Hmat h_i2a)
{
    Hmat h_a2b, h_a2c;
    RotAxisAngle axis_angle;

    // calc pos average
    h_a2b = Hmat_relative(h_i2b, h_i2a);
    h_a2c.pos = Vector3_multiply(0.5, h_a2b.pos);
    
    // calc pose average
    axis_angle = Quaternion2RotAxisAngle(h_a2b.quat);
    axis_angle.angle = wrapAngleToPiRad(axis_angle.angle);
    axis_angle.angle = 0.5*axis_angle.angle;
    h_a2c.quat = RotAxisAngle2Quaternion(axis_angle);

    return Hmat_composite(h_i2a, h_a2c);
}


#endif
