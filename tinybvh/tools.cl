// Common constants
#define PI			3.14159265358979323846264f
#define INVPI		0.31830988618379067153777f
#define INV2PI		0.15915494309189533576888f
#define TWOPI		6.28318530717958647692528f
#define EPSILON		0.0001f // for a 100^3 world

// Xor32 RNG
uint WangHash( uint s ) { s = (s ^ 61) ^ (s >> 16), s *= 9, s = s ^ (s >> 4), s *= 0x27d4eb2d; return s ^ (s >> 15); }
uint RandomUInt( uint* seed ) { *seed ^= *seed << 13, * seed ^= *seed >> 17, * seed ^= *seed << 5; return *seed; }
float RandomFloat( uint* seed ) { return RandomUInt( seed ) * 2.3283064365387e-10f; }

// Color conversion
float3 rgb32_to_vec3( uint c )
{
	return (float3)((float)((c >> 16) & 255), (float)((c >> 8) & 255), (float)(c & 255)) * 0.00392f;
}

// Specular reflection
float3 Reflect( const float3 D, const float3 N ) { return D - 2.0f * N * dot( N, D ); }

// DiffuseReflection: Uniform random bounce in the hemisphere
float3 DiffuseReflection( float3 N, uint* seed )
{
	float3 R;
	do
	{
		R = (float3)(RandomFloat( seed ) * 2 - 1, RandomFloat( seed ) * 2 - 1, RandomFloat( seed ) * 2 - 1);
	} while (dot( R, R ) > 1);
	return fast_normalize( dot( R, N ) > 0 ? R : -R );
}

// CosWeightedDiffReflection: Cosine-weighted random bounce in the hemisphere
float3 CosWeightedDiffReflection( const float3 N, const float r0, const float r1 )
{
	const float r = sqrt( 1 - r1 * r1 ), phi = 4 * PI * r0;
	const float3 R = (float3)( cos( phi ) * r, sin( phi ) * r, r1);
	return fast_normalize( N + R );
}

// TransformPoint - apply a 4x4 matrix transform to a 3D position
float3 TransformPoint( const float3 v, const float* T )
{
	const float3 res = (float3)(
		T[0] * v.x + T[1] * v.y + T[2] * v.z + T[3],
		T[4] * v.x + T[5] * v.y + T[6] * v.z + T[7],
		T[8] * v.x + T[9] * v.y + T[10] * v.z + T[11] 
	);
	const float w = T[12] * v.x + T[13] * v.y + T[14] * v.z + T[15];
	if (w == 1.0f) return res; else return res * (1.0f / w);
}

// TransformVector - apply a 4x4 matrix transform to a 3D vector.
float3 TransformVector( const float3 v, const float* T )
{
	return (float3)( T[0] * v.x + T[1] * v.y + T[2] * v.z, T[4] * v.x +
		T[5] * v.y + T[6] * v.z, T[8] * v.x + T[9] * v.y + T[10] * v.z );
}