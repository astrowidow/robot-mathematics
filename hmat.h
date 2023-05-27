
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
    Hmat h_a2i = Hmat_inverse(h_i2a);
    return Hmat_composite(h_a2i, h_i2c);
}

#endif
