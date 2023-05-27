#ifndef __DCM_H__
#define __DCM_H__

#include <math.h>
#include "quaternion.h"
#include "vector3.h"

typedef struct {
    double e[3][3];
} Dcm;

/**
 * @brief Convert a Quaternion to a transposed Direction Cosine Matrix (DCM)
 * 
 * @param q Input Quaternion
 * @return Dcm Transposed DCM representation of the input Quaternion
 */
Dcm Quaternion2Dcm(const Quaternion q)
{
    Dcm dcm;
    
    double xx = q.x * q.x;
    double yy = q.y * q.y;
    double zz = q.z * q.z;
    double xy = q.x * q.y;
    double xz = q.x * q.z;
    double yz = q.y * q.z;
    double wx = q.w * q.x;
    double wy = q.w * q.y;
    double wz = q.w * q.z;

    dcm.e[0][0] = 1.0 - 2.0 * (yy + zz);
    dcm.e[0][1] = 2.0 * (xy + wz);
    dcm.e[0][2] = 2.0 * (xz - wy);

    dcm.e[1][0] = 2.0 * (xy - wz);
    dcm.e[1][1] = 1.0 - 2.0 * (xx + zz);
    dcm.e[1][2] = 2.0 * (yz + wx);

    dcm.e[2][0] = 2.0 * (xz + wy);
    dcm.e[2][1] = 2.0 * (yz - wx);
    dcm.e[2][2] = 1.0 - 2.0 * (xx + yy);

    return dcm;
}

/**
 * @brief Convert a Direction Cosine Matrix (DCM) to a Quaternion
 * 
 * @param dcm Input DCM
 * @return Quaternion Quaternion representation of the input DCM
 */
Quaternion Dcm2Quaternion(const Dcm dcm)
{
    Quaternion q;

    double q0sq = 0.25 * (1.0 + dcm.e[0][0] + dcm.e[1][1] + dcm.e[2][2]); // square of quaternion scalar part
    double q1sq = 0.25 * (1.0 + dcm.e[0][0] - dcm.e[1][1] - dcm.e[2][2]); // square of quaternion vectorial part i
    double q2sq = 0.25 * (1.0 - dcm.e[0][0] + dcm.e[1][1] - dcm.e[2][2]); // square of quaternion vectorial part j
    double q3sq = 0.25 * (1.0 - dcm.e[0][0] - dcm.e[1][1] + dcm.e[2][2]); // square of quaternion vectorial part k

    if ((q0sq >= q1sq) && (q0sq >= q2sq) && (q0sq >= q3sq)) {
        q.w = sqrt(q0sq);
        q.x = (dcm.e[1][2] - dcm.e[2][1]) / (4.0 * q.w);
        q.y = (dcm.e[2][0] - dcm.e[0][2]) / (4.0 * q.w);
        q.z = (dcm.e[0][1] - dcm.e[1][0]) / (4.0 * q.w);
    } else if ((q1sq >= q0sq) && (q1sq >= q2sq) && (q1sq >= q3sq)) {
        q.x = sqrt(q1sq);
        q.w = (dcm.e[1][2] - dcm.e[2][1]) / (4.0 * q.x);
        q.y = (dcm.e[0][1] + dcm.e[1][0]) / (4.0 * q.x);
        q.z = (dcm.e[2][0] + dcm.e[0][2]) / (4.0 * q.x);
    } else if ((q2sq >= q0sq) && (q2sq >= q1sq) && (q2sq >= q3sq)) {
        q.y = sqrt(q2sq);
        q.w = (dcm.e[2][0] - dcm.e[0][2]) / (4.0 * q.y);
        q.x = (dcm.e[0][1] + dcm.e[1][0]) / (4.0 * q.y);
        q.z = (dcm.e[1][2] + dcm.e[2][1]) / (4.0 * q.y);
    } else if ((q3sq >= q0sq) && (q3sq >= q1sq) && (q3sq >= q2sq)) {
        q.z = sqrt(q3sq);
        q.w = (dcm.e[0][1] - dcm.e[1][0]) / (4.0 * q.z);
        q.x = (dcm.e[2][0] + dcm.e[0][2]) / (4.0 * q.z);
        q.y = (dcm.e[1][2] + dcm.e[2][1]) / (4.0 * q.z);
    }

    return Quaternion_normalize(q);
}

/**
 * @brief Transform a 3D vector using a Direction Cosine Matrix (DCM)
 * 
 * @param dcm The transformation matrix
 * @param v The vector to transform
 * @return Vector3 The transformed vector
 */
Vector3 Dcm_transformVector(const Dcm dcm, const Vector3 v)
{
    Vector3 result;

    result.x = dcm.e[0][0] * v.x + dcm.e[0][1] * v.y + dcm.e[0][2] * v.z;
    result.y = dcm.e[1][0] * v.x + dcm.e[1][1] * v.y + dcm.e[1][2] * v.z;
    result.z = dcm.e[2][0] * v.x + dcm.e[2][1] * v.y + dcm.e[2][2] * v.z;

    return result;
}

/**
 * @brief Transform a 3D vector using a Quaternion
 * 
 * @param q The quaternion representing the orientation
 * @param v The vector to transform
 * @return Vector3 The transformed vector
 */
Vector3 Quaternion_transformVector(const Quaternion q, const Vector3 v)
{
    Dcm dcm = Quaternion2Dcm(q);
    return Dcm_transformVector(dcm, v);
}

#endif
