#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <math.h>
#include "euler_angles.h"

typedef struct {
    double x;
    double y;
    double z;
    double w;
} Quaternion;

/**
 * @brief Function to return a normalized Quaternion
 * 
 * @param q Quaternion to be normalized
 * @return Normalized Quaternion
 */
Quaternion Quaternion_normalize(const Quaternion q)
{
    double magnitude = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

    Quaternion normalized;
    normalized.x = q.x / magnitude;
    normalized.y = q.y / magnitude;
    normalized.z = q.z / magnitude;
    normalized.w = q.w / magnitude;

    return normalized;
}

/**
 * @brief Function to create a normalized Quaternion
 * 
 * @param x Value of the x component
 * @param y Value of the y component
 * @param z Value of the z component
 * @param w Value of the w component
 * @return Normalized Quaternion
 */
Quaternion Quaternion_create(const double x, const double y, const double z, const double w)
{
    Quaternion quaternion;

    quaternion.x = x;
    quaternion.y = y;
    quaternion.z = z;
    quaternion.w = w;

    return Quaternion_normalize(quaternion);
}

/**
 * @brief Function to compute the composite Quaternion of two Quaternions
 * 
 * @param q1 First Quaternion representing the initial rotation
 * @param q2 Second Quaternion representing the additional rotation
 * @return Composite Quaternion representing the combined rotation
 */
Quaternion Quaternion_composite(const Quaternion q1, const Quaternion q2)
{
    Quaternion composite;
    
    composite.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    composite.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    composite.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    composite.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    
    return Quaternion_normalize(composite);
}

/**
 * @brief Function to compute the inverse of a Quaternion
 *
 * @param q Quaternion to compute the inverse for
 * @return Inverse Quaternion
 */
Quaternion Quaternion_inverse(const Quaternion q)
{
    Quaternion inverse;

    // Compute the inverse Quaternion
    inverse.x = -q.x;
    inverse.y = -q.y;
    inverse.z = -q.z;
    inverse.w = q.w;

    return Quaternion_normalize(inverse);
}

/**
 * @brief Converts ZYX order Euler Angles to a Quaternion.
 * 
 * This function takes Euler angles with ZYX rotation order (Yaw-Pitch-Roll) as an input 
 * and converts them to a Quaternion. The resulting Quaternion is normalized before returning.
 * The function Quaternion_normalize is used for this purpose.
 *
 * @param angles The input Euler angles in ZYX order
 * @return Quaternion The output normalized Quaternion
 */
Quaternion EulerZYX2Quaternion(EulerAngles angles) {
    Quaternion q;

    // Abbreviations for the various angular functions
    double cy = cos(angles.z * 0.5);
    double sy = sin(angles.z * 0.5);
    double cp = cos(angles.y * 0.5);
    double sp = sin(angles.y * 0.5);
    double cr = cos(angles.x * 0.5);
    double sr = sin(angles.x * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    // Normalize the Quaternion
    q = Quaternion_normalize(q);

    return q;
}

/**
 * @brief Converts a Quaternion to ZYX order Euler Angles.
 * 
 * This function takes a Quaternion as an input and converts it to Euler angles 
 * with ZYX rotation order (Yaw-Pitch-Roll). It returns an EulerAngles structure.
 *
 * @param q The input Quaternion
 * @return EulerAngles The output Euler angles in ZYX order
 */
EulerAngles Quaternion2EulerZYX(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.y = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    angles.z = atan2(siny_cosp, cosy_cosp);

    angles.type = ZYX;

    return angles;
}

/**
 * @brief Converts XYZ order Euler Angles to a Quaternion.
 * 
 * This function takes Euler angles with XYZ rotation order (Roll-Pitch-Yaw) as an input 
 * and converts them to a Quaternion. The resulting Quaternion is normalized before returning.
 * The function Quaternion_normalize is used for this purpose.
 *
 * @param angles The input Euler angles in XYZ order
 * @return Quaternion The output normalized Quaternion
 */
Quaternion euler_to_quaternion_XYZ(EulerAngles angles) {
    Quaternion q;

    // Abbreviations for the various angular functions
    double cy = cos(angles.z * 0.5);
    double sy = sin(angles.z * 0.5);
    double cp = cos(angles.y * 0.5);
    double sp = sin(angles.y * 0.5);
    double cr = cos(angles.x * 0.5);
    double sr = sin(angles.x * 0.5);

    q.w = cr * cp * cy - sr * sp * sy;
    q.x = sr * cp * cy + cr * sp * sy;
    q.y = cr * sp * cy - sr * cp * sy;
    q.z = cr * cp * sy + sr * sp * cy;

    // Normalize the Quaternion
    q = Quaternion_normalize(q);

    return q;
}

/**
 * @brief Converts a Quaternion to XYZ order Euler Angles.
 * 
 * This function takes a Quaternion as an input and converts it to Euler angles 
 * with XYZ rotation order. It returns an EulerAngles structure.
 *
 * @param q The input Quaternion
 * @return EulerAngles The output Euler angles in XYZ order
 */
EulerAngles Quaternion2EulerXYZ(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w * q.x - q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2.0 * (q.w * q.y + q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.y = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w * q.z - q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    angles.z = atan2(siny_cosp, cosy_cosp);

    angles.type = XYZ;

    return angles;
}

#endif