//============ Copyright (c) Valve Corporation, All rights reserved. ============
#pragma once

#include "openvr_driver.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.1415926535
#endif

#define DEG_TO_RAD( degrees ) ( ( degrees )*M_PI / 180.0 )
#define RAD_TO_DEG( radians ) ( ( radians )*180.0 / M_PI )

static const vr::HmdQuaternion_t HmdQuaternion_Identity = { 1.f, 0.f, 0.f, 0.f };

// right hand coordinate system
static const vr::HmdVector3_t HmdVector3_Right = { 1.f, 0, 0 };
static const vr::HmdVector3_t HmdVector3_Left = { -1.f, 0, 0 };
static const vr::HmdVector3_t HmdVector3_Up = { 0, 1.f, 0 };
static const vr::HmdVector3_t HmdVector3_Down = { 0, -1.f, 0 };
static const vr::HmdVector3_t HmdVector3_Forward = { 0, 0, -1.f };
static const vr::HmdVector3_t HmdVector3_Backward = { 0, 0, 1.f };

// 3x3 or 3x4 matrix
template < class T >
vr::HmdQuaternion_t HmdQuaternion_FromMatrix( const T &matrix )
{
	vr::HmdQuaternion_t q{};

	q.w = sqrt( fmax( 0, 1 + matrix.m[ 0 ][ 0 ] + matrix.m[ 1 ][ 1 ] + matrix.m[ 2 ][ 2 ] ) ) / 2;
	q.x = sqrt( fmax( 0, 1 + matrix.m[ 0 ][ 0 ] - matrix.m[ 1 ][ 1 ] - matrix.m[ 2 ][ 2 ] ) ) / 2;
	q.y = sqrt( fmax( 0, 1 - matrix.m[ 0 ][ 0 ] + matrix.m[ 1 ][ 1 ] - matrix.m[ 2 ][ 2 ] ) ) / 2;
	q.z = sqrt( fmax( 0, 1 - matrix.m[ 0 ][ 0 ] - matrix.m[ 1 ][ 1 ] + matrix.m[ 2 ][ 2 ] ) ) / 2;

	q.x = copysign( q.x, matrix.m[ 2 ][ 1 ] - matrix.m[ 1 ][ 2 ] );
	q.y = copysign( q.y, matrix.m[ 0 ][ 2 ] - matrix.m[ 2 ][ 0 ] );
	q.z = copysign( q.z, matrix.m[ 1 ][ 0 ] - matrix.m[ 0 ][ 1 ] );

	return q;
}

// Utility function for quaternion multiplication
inline vr::HmdQuaternion_t MultiplyQuaternions(const vr::HmdQuaternion_t &q1, const vr::HmdQuaternion_t &q2)
{
    vr::HmdQuaternion_t result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

// Utility function for vector addition
inline vr::HmdVector3_t AddVectors(const vr::HmdVector3_t &v1, const vr::HmdVector3_t &v2)
{
    vr::HmdVector3_t result;
    result.v[0] = v1.v[0] + v2.v[0];
    result.v[1] = v1.v[1] + v2.v[1];
    result.v[2] = v1.v[2] + v2.v[2];
    return result;
}

// Utility function to rotate a vector by a quaternion
inline vr::HmdVector3_t RotateVectorByQuaternion(const vr::HmdVector3_t &vec, const vr::HmdQuaternion_t &quat)
{
    vr::HmdQuaternion_t vecQuat = {0, vec.v[0], vec.v[1], vec.v[2]};
    vr::HmdQuaternion_t quatConjugate = {quat.w, -quat.x, -quat.y, -quat.z};

    vr::HmdQuaternion_t resultQuat = MultiplyQuaternions(MultiplyQuaternions(quat, vecQuat), quatConjugate);

    vr::HmdVector3_t result = {resultQuat.x, resultQuat.y, resultQuat.z};
    return result;
}

static vr::HmdQuaternion_t HmdQuaternion_FromSwingTwist( const vr::HmdVector2_t &swing, const float twist )
{
	vr::HmdQuaternion_t result{};

	const float swing_squared = swing.v[ 0 ] * swing.v[ 0 ] + swing.v[ 1 ] * swing.v[ 1 ];

	if ( swing_squared > 0.f )
	{
		const float theta_swing = std::sqrt( swing_squared );

		const float cos_half_theta_swing = std::cos( theta_swing * 0.5f );
		const float cos_half_theta_twist = std::cos( twist * 0.5f );

		const float sin_half_theta_twist = std::sin( twist * 0.5f );

		const float sin_half_theta_swing_over_theta = std::sin( theta_swing * 0.5f ) / theta_swing;

		result.w = cos_half_theta_swing * cos_half_theta_twist;

		result.x = cos_half_theta_swing * sin_half_theta_twist;

		result.y = ( swing.v[ 1 ] * cos_half_theta_twist * sin_half_theta_swing_over_theta ) - ( swing.v[ 0 ] * sin_half_theta_twist * sin_half_theta_swing_over_theta );

		result.z = ( swing.v[ 0 ] * cos_half_theta_twist * sin_half_theta_swing_over_theta ) + ( swing.v[ 1 ] * sin_half_theta_twist * sin_half_theta_swing_over_theta );
	}
	else
	{
		float half_twist = twist * 0.5f;
		float cos_half_twist = cos( half_twist );
		float sin_half_twist = sin( half_twist );
		float sin_half_theta_over_theta = 0.5f;

		result.w = cos_half_twist;
		result.x = sin_half_twist;
		result.y = ( swing.v[ 1 ] * cos_half_twist * sin_half_theta_over_theta ) - ( swing.v[ 0 ] * sin_half_twist * sin_half_theta_over_theta );
		result.z = ( swing.v[ 0 ] * cos_half_twist * sin_half_theta_over_theta ) + ( swing.v[ 1 ] * sin_half_twist * sin_half_theta_over_theta );
	}

	return result;
}

static vr::HmdQuaternion_t HmdQuaternion_Normalize(const vr::HmdQuaternion_t &q)
{
    vr::HmdQuaternion_t result{};
    double n = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

    if (n == 0.0) {
        return result;  // Return the default zero quaternion if magnitude is zero
    }

    result.w = q.w / n;
    result.x = q.x / n;
    result.y = q.y / n;
    result.z = q.z / n;

    return result;
}

template < class T, class Q >
void HmdQuaternion_ConvertQuaternion( const T &in_quaternion, Q &out_quaternion )
{
	out_quaternion.w = in_quaternion.w;
	out_quaternion.x = in_quaternion.x;
	out_quaternion.y = in_quaternion.y;
	out_quaternion.z = in_quaternion.z;
}

static vr::HmdQuaternion_t operator-( const vr::HmdQuaternion_t &q )
{
	return { q.w, -q.x, -q.y, -q.z };
}

static vr::HmdQuaternion_t operator*( const vr::HmdQuaternion_t &lhs, const vr::HmdQuaternion_t &rhs )
{
	return {
		lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
		lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
		lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
		lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
	};
}

static vr::HmdVector3_t HmdVector3_From34Matrix( const vr::HmdMatrix34_t &matrix )
{
	return { matrix.m[ 0 ][ 3 ], matrix.m[ 1 ][ 3 ], matrix.m[ 2 ][ 3 ] };
}

static vr::HmdVector3_t operator+( const vr::HmdMatrix34_t &matrix, const vr::HmdVector3_t &vec )
{
	vr::HmdVector3_t vector{};

	vector.v[ 0 ] = matrix.m[ 0 ][ 3 ] + vec.v[ 0 ];
	vector.v[ 1 ] = matrix.m[ 1 ][ 3 ] + vec.v[ 1 ];
	vector.v[ 2 ] = matrix.m[ 2 ][ 3 ] + vec.v[ 2 ];

	return vector;
}

static vr::HmdVector3_t operator*( const vr::HmdMatrix33_t &matrix, const vr::HmdVector3_t &vec )
{
	vr::HmdVector3_t result{};

	result.v[ 0 ] = matrix.m[ 0 ][ 0 ] * vec.v[ 0 ] + matrix.m[ 0 ][ 1 ] * vec.v[ 1 ] + matrix.m[ 0 ][ 2 ] * vec.v[ 2 ];
	result.v[ 1 ] = matrix.m[ 1 ][ 0 ] * vec.v[ 0 ] + matrix.m[ 1 ][ 1 ] * vec.v[ 1 ] + matrix.m[ 1 ][ 2 ] * vec.v[ 2 ];
	result.v[ 2 ] = matrix.m[ 2 ][ 0 ] * vec.v[ 0 ] + matrix.m[ 2 ][ 1 ] * vec.v[ 1 ] + matrix.m[ 2 ][ 2 ] * vec.v[ 2 ];

	return result;
}

static vr::HmdVector3_t operator-( const vr::HmdVector3_t &vec, const vr::HmdMatrix34_t &matrix )
{
	return { vec.v[ 0 ] - matrix.m[ 0 ][ 3 ], vec.v[ 1 ] - matrix.m[ 1 ][ 3 ], vec.v[ 2 ] - matrix.m[ 2 ][ 3 ] };
}

static vr::HmdVector3d_t operator+( const vr::HmdVector3d_t &vec1, const vr::HmdVector3d_t &vec2 )
{
	return { vec1.v[ 0 ] + vec2.v[ 0 ], vec1.v[ 1 ] + vec2.v[ 1 ], vec1.v[ 2 ] + vec2.v[ 2 ] };
}


static vr::HmdVector3_t operator+( const vr::HmdVector3_t &vec1, const vr::HmdVector3_t &vec2 )
{
	return { vec1.v[ 0 ] + vec2.v[ 0 ], vec1.v[ 1 ] + vec2.v[ 1 ], vec1.v[ 2 ] + vec2.v[ 2 ] };
}

static vr::HmdVector3d_t operator-( const vr::HmdVector3d_t &vec1, const vr::HmdVector3d_t &vec2 )
{
	return { vec1.v[ 0 ] - vec2.v[ 0 ], vec1.v[ 1 ] - vec2.v[ 1 ], vec1.v[ 2 ] - vec2.v[ 2 ] };
}

static vr::HmdVector3_t operator*( const vr::HmdVector3_t &vec, const vr::HmdQuaternion_t &q )
{
	const vr::HmdQuaternion_t qvec = { 0.0, vec.v[ 0 ], vec.v[ 1 ], vec.v[ 2 ] };

	const vr::HmdQuaternion_t qResult = (q * qvec) * (-q);

	return { static_cast< float >( qResult.x ), static_cast< float >( qResult.y ), static_cast< float >( qResult.z ) };
}

template < class T, class V >
void HmdVector3_CovertVector( const T &in_vector, V &out_vector )
{
	out_vector.v[ 0 ] = in_vector.v[ 0 ];
	out_vector.v[ 1 ] = in_vector.v[ 1 ];
	out_vector.v[ 2 ] = in_vector.v[ 2 ];
}

static vr::HmdQuaternion_t EulerToQuaternion(float roll, float pitch, float yaw) // roll (x), pitch (y), yaw (z), angles are in degrees
{
	//Euler angles: Body 3-2-1 sequence
	//The airplane first does yaw(Body - Z) turn during taxiing onto the runway
	//Then pitches(Body - Y) during take-off
	//Thenfinally rolls(Body - X) in the air. 

	double cr = cos(DEG_TO_RAD(roll) * 0.5);
	double sr = sin(DEG_TO_RAD(roll) * 0.5);
	double cp = cos(DEG_TO_RAD(pitch) * 0.5);
	double sp = sin(DEG_TO_RAD(pitch) * 0.5);
	double cy = cos(DEG_TO_RAD(yaw) * 0.5);
	double sy = sin(DEG_TO_RAD(yaw) * 0.5);

	vr::HmdQuaternion_t q;
	q.w = (float)(cr * cp * cy + sr * sp * sy);
	q.x = (float)(sr * cp * cy - cr * sp * sy);
	q.y = (float)(cr * sp * cy + sr * cp * sy);
	q.z = (float)(cr * cp * sy - sr * sp * cy);

	return q;
}