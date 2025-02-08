// This example shows how to build a GPU path tracer with instancing
// using a TLAS. TinyOCL is used to render on the GPU using OpenCL.

#define FENSTER_APP_IMPLEMENTATION
#define SCRWIDTH 1280
#define SCRHEIGHT 720
#include "external/fenster.h" // https://github.com/zserge/fenster

#define SCENE 2

#if SCENE == 1

// At GRIDSIZE == 15, make sure to set INST_IDX_BITS to 12 in tiny_bvh.h.
#define GRIDSIZE 15
#define DRAGONS (GRIDSIZE * GRIDSIZE * GRIDSIZE)
// #define AUTOCAM

#else

#define DRAGONS 100
#define AUTOCAM

#endif

// This application uses tinybvh - And this file will include the implementation.
#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"
using namespace tinybvh;

// This application uses tinyocl - And this file will include the implementation.
#define TINY_OCL_IMPLEMENTATION
#include "tiny_ocl.h"

// Other includes
#include <fstream>
#include <cstdlib>
#include <cstdio>

// Application variables

static BVH8_CWBVH bistro, dragon;
static BVHBase* blasList[] = { &bistro, &dragon };
static BVH_GPU tlas;
static BLASInstance instance[DRAGONS + 1];
static bvhvec4* verts = 0, * dragonVerts = 0;
static int triCount = 0, dragonTriCount = 0, frameIdx = 0, spp = 0;
static Kernel* init, * clear, * rayGen, * extend, * shade;
static Kernel* updateCounters1, * updateCounters2, * traceShadows, * finalize;
static Buffer* pixels, * accumulator, * raysIn, * raysOut, * connections;
static Buffer* bistroNodes = 0, * bistroTris = 0, * bistroVerts, * dragonNodes = 0, * dragonTris = 0, * drVerts, * noise = 0;
static Buffer* tlasNodes = 0, * tlasIndices = 0, * blasInstances = 0;
static size_t computeUnits;
static uint32_t* blueNoise = new uint32_t[128 * 128 * 8];

// View pyramid for a pinhole camera
struct RenderData
{
	bvhvec4 eye = bvhvec4( 0, 30, 0, 0 ), view = bvhvec4( -1, 0, 0, 0 ), C, p0, p1, p2;
	uint32_t frameIdx, dummy1, dummy2, dummy3;
} rd;

// Splines
static bvhvec3 spline[] = {
	bvhvec3( -3.378f, 0, -38.44f ),bvhvec3( -1.91f, 0, -36.10f ),
	bvhvec3( -1.775f, 0, -31.95f ),bvhvec3( -3.15f, 0, -28.50f ),
	bvhvec3( -6.027f, 0, -24.12f ),bvhvec3( -9.32f, 0, -19.11f ),
	bvhvec3( -12.40f, 0, -15.25f ),bvhvec3( -15.40f, 0, -12.02f ),
	bvhvec3( -18.42f, 0, -7.97f ),bvhvec3( -20.30f, 0, -3.28f ),
	bvhvec3( -19.36f, 0, 0.809f ),bvhvec3( -16.90f, 0, 2.53f ),
	bvhvec3( -13.10f, 0, 4.788f ),bvhvec3( -8.25f, 0, 6.87f ),
	bvhvec3( -3.060f, 0, 9.029f ),bvhvec3( 5.6988f, 0, 12.67f ),
	bvhvec3( 12.176f, 0, 15.38f ),bvhvec3( 17.394f, 0, 18.44f ),
	bvhvec3( 20.821f, 0, 21.96f ),bvhvec3( 25.406f, 0, 25.10f ),
	bvhvec3( 29.196f, 0, 23.99f ),bvhvec3( 31.381f, 0, 19.60f ),
	bvhvec3( 28.708f, 0, 14.89f ),bvhvec3( 21.821f, 0, 13.16f )
};
static bvhvec3 splinePos( float t )
{
	uint32_t s = (uint32_t)t;
	t -= (float)s;
	const bvhvec3 P = spline[s - 1], Q = spline[s], R = spline[s + 1], S = spline[s + 2];
	const bvhvec3 a = 2 * Q, b = R - P, c = 2 * P - 5 * Q + 4 * R - S;
	return 0.5f * (a + (b * t) + (c * t * t) + ((3 * Q - 3 * R + S - P) * t * t * t));
}
static bvhvec3 cam[] = {
	bvhvec3( 1.86f, -7.21f, -31.52f ), bvhvec3( 0.97f, -7.25f, -31.08f ),
	bvhvec3( -1.02f, -3.32f, -29.35f ), bvhvec3( -1.77f, -3.69f, -28.79f ),
	bvhvec3( -7.80f, -4.41f, -21.00f ), bvhvec3( -8.15f, -5.24f, -20.57f ),
	bvhvec3( -14.72f, -6.29f, -15.06f ), bvhvec3( -14.78f, -6.97f, -14.32f ),
	bvhvec3( -20.33f, -6.91f, -10.36f ), bvhvec3( -19.81f, -7.57f, -9.81f ),
	bvhvec3( -23.23f, -7.53f, -3.91f ), bvhvec3( -22.36f, -8.02f, -3.95f ),
	bvhvec3( -21.29f, -8.22f, 3.83f ), bvhvec3( -20.51f, -8.38f, 3.23f ),
	bvhvec3( -15.81f, -8.34f, 6.93f ), bvhvec3( -15.37f, -8.50f, 6.05f ),
	bvhvec3( -11.27f, -8.34f, 8.90f ), bvhvec3( -10.94f, -8.50f, 7.97f ),
	bvhvec3( -1.07f, -5.07f, 9.41f ), bvhvec3( -1.98f, -5.29f, 9.051f ),
	bvhvec3( 9.77f, -7.38f, 13.64f ), bvhvec3( 8.84f, -7.33f, 13.28f ),
	bvhvec3( 12.93f, 0.06f, 19.42f ), bvhvec3( 12.28f, -0.48f, 18.89f ),
	bvhvec3( 19.39f, 8.09f, 23.33f ), bvhvec3( 18.89f, 7.33f, 22.92f ),
};
static void splineCam( float t )
{
	uint32_t s = (uint32_t)t;
	t -= (float)s, s *= 2;
	const bvhvec3 Pp = cam[s - 2], Qp = cam[s], Rp = cam[s + 2], Sp = cam[s + 4];
	const bvhvec3 Pt = cam[s - 1], Qt = cam[s + 1], Rt = cam[s + 3], St = cam[s + 5];
	bvhvec3 a = 2 * Qp, b = Rp - Pp, c = 2 * Pp - 5 * Qp + 4 * Rp - Sp;
	rd.eye = 0.5f * (a + (b * t) + (c * t * t) + ((3 * Qp - 3 * Rp + Sp - Pp) * t * t * t));
	a = 2 * Qt, b = Rt - Pt, c = 2 * Pt - 5 * Qt + 4 * Rt - St;
	bvhvec3 target = 0.5f * (a + (b * t) + (c * t * t) + ((3 * Qt - 3 * Rt + St - Pt) * t * t * t));
	rd.view = tinybvh_normalize( target - bvhvec3( rd.eye ) );
}
static float uniform_rand() { return (float)rand() / (float)RAND_MAX; }

// Scene management - Append a file, with optional position, scale and color override, tinyfied
void AddMesh( const char* file, float scale = 1, bvhvec3 pos = {}, int c = 0, int N = 0 )
{
	std::fstream s{ file, s.binary | s.in }; s.read( (char*)&N, 4 );
	bvhvec4* data = (bvhvec4*)tinybvh::malloc64( (N + triCount) * 48 );
	if (verts) memcpy( data, verts, triCount * 48 ), tinybvh::free64( verts );
	verts = data, s.read( (char*)verts + triCount * 48, N * 48 ), triCount += N;
	for (int* b = (int*)verts + (triCount - N) * 12, i = 0; i < N * 3; i++)
		*(bvhvec3*)b = *(bvhvec3*)b * scale + pos, b[3] = c ? c : b[3], b += 4;
}
void AddQuad( const bvhvec3 pos, const float w, const float d, int c )
{
	bvhvec4* data = (bvhvec4*)tinybvh::malloc64( (triCount + 2) * 48 );
	if (verts) memcpy( data + 6, verts, triCount * 48 ), tinybvh::free64( verts );
	data[0] = bvhvec3( -w, 0, -d ), data[1] = bvhvec3( w, 0, -d );
	data[2] = bvhvec3( w, 0, d ), data[3] = bvhvec3( -w, 0, -d ), verts = data;
	data[4] = bvhvec3( w, 0, d ), data[5] = bvhvec3( -w, 0, d ), triCount += 2;
	for (int i = 0; i < 6; i++) data[i] = 0.5f * data[i] + pos, data[i].w = *(float*)&c;
}

// Blue noise from file
void LoadBlueNoise()
{
	std::fstream s{ "./testdata/blue_noise_128x128x8_2d.raw", s.binary | s.in };
	s.read( (char*)blueNoise, 128 * 128 * 8 * 4 );
}

// Application init
void Init()
{
	// create OpenCL kernels
	init = new Kernel( "wavefront2.cl", "SetRenderData" );
	clear = new Kernel( "wavefront2.cl", "Clear" );
	rayGen = new Kernel( "wavefront2.cl", "Generate" );
	extend = new Kernel( "wavefront2.cl", "Extend" );
	shade = new Kernel( "wavefront2.cl", "Shade" );
	updateCounters1 = new Kernel( "wavefront2.cl", "UpdateCounters1" );
	updateCounters2 = new Kernel( "wavefront2.cl", "UpdateCounters2" );
	traceShadows = new Kernel( "wavefront2.cl", "Connect" );
	finalize = new Kernel( "wavefront2.cl", "Finalize" );

	// we need the 'compute unit' or 'SM' count for wavefront rendering; ask OpenCL for it.
	clGetDeviceInfo( init->GetDeviceID(), CL_DEVICE_MAX_COMPUTE_UNITS, sizeof( size_t ), &computeUnits, NULL );

	// create OpenCL buffers for wavefront path tracing
	int N = SCRWIDTH * SCRHEIGHT;
	raysIn = new Buffer( N * sizeof( bvhvec4 ) * 4 );
	raysOut = new Buffer( N * sizeof( bvhvec4 ) * 4 );
	connections = new Buffer( N * 3 * sizeof( bvhvec4 ) * 3 );
	accumulator = new Buffer( N * sizeof( bvhvec4 ) );
	pixels = new Buffer( N * sizeof( uint32_t ) );
	LoadBlueNoise();
	noise = new Buffer( 128 * 128 * 8 * sizeof( uint32_t ), blueNoise );
	noise->CopyToDevice();

#if SCENE == 1

	// load dragon mesh
	AddMesh( "./testdata/dragon.bin", 1, bvhvec3( 0 ) );
	swap( verts, dragonVerts );
	swap( triCount, dragonTriCount );
	dragon.Build( dragonVerts, dragonTriCount );

	// dragon grid
	for (int b = 1, x = 0; x < GRIDSIZE; x++) for (int y = 0; y < GRIDSIZE; y++) for (int z = 0; z < GRIDSIZE; z++, b++)
	{
		instance[b] = BLASInstance( 1 /* dragon */ );
		BLASInstance& i = instance[b];
		i.transform[0] = i.transform[5] = i.transform[10] = 0.07f;
		i.transform[3] = x, i.transform[7] = y, i.transform[11] = z;
	}

	// vertex data for static scenery
	AddQuad( bvhvec3( -22, 12, 2 ), 9, 5, 0x1ffffff ); // hard-coded light source
	AddMesh( "./testdata/bistro_ext_part1.bin", 1, bvhvec3( 500 ), 0xffffff );
	bistro.Build( verts, 300 );

#else

	// load dragon mesh
	AddMesh( "./testdata/dragon.bin", 1, bvhvec3( 0 ) );
	swap( verts, dragonVerts );
	swap( triCount, dragonTriCount );
	dragon.Build( dragonVerts, dragonTriCount );

	// create dragon instances
	for (int d = 0; d < DRAGONS; d++)
	{
		instance[d + 1] = BLASInstance( 1 /* dragon */ );
		BLASInstance& i = instance[d + 1];
		float t = (float)d * 0.17f + 1.0f;
		float size = 0.1f + 0.075f * uniform_rand();
		bvhvec3 pos = splinePos( t );
		bvhvec3 D = -size * tinybvh_normalize( splinePos( t + 0.01f ) - pos );
		bvhvec3 U( 0, size, 0 );
		bvhvec3 N( -D.z, 0, D.x );
		pos += N * 20.0f * (uniform_rand() - 0.5f);
		i.transform[0] = N.x, i.transform[1] = N.y, i.transform[2] = N.z;
		i.transform[4] = U.x, i.transform[5] = U.y, i.transform[6] = U.z;
		i.transform[8] = D.x, i.transform[9] = D.y, i.transform[10] = D.z;
		i.transform[3] = pos.x;
		i.transform[7] = -9.2f;
		i.transform[11] = pos.z;
	}
	// load vertex data for static scenery
	AddQuad( bvhvec3( -22, 12, 2 ), 9, 5, 0x1ffffff ); // hard-coded light source
	AddMesh( "./testdata/bistro_ext_part1.bin", 1, bvhvec3( 0 ) );
	AddMesh( "./testdata/bistro_ext_part2.bin", 1, bvhvec3( 0 ) );
	bistro.Build( verts, triCount );

#endif

	// build bvh (here: 'compressed wide bvh', for efficient GPU rendering)
	instance[0] = BLASInstance( 0 /* static geometry */ );

	// finalize scene: build tlas
	tlas.Build( instance, DRAGONS + 1, blasList, 2 );

	// create OpenCL buffers for BVH data
	tlasNodes = new Buffer( tlas.allocatedNodes /* could change! */ * sizeof( BVH_GPU::BVHNode ), tlas.bvhNode );
	tlasIndices = new Buffer( tlas.bvh.idxCount * sizeof( uint32_t ), tlas.bvh.primIdx );
	tlasNodes->CopyToDevice();
	tlasIndices->CopyToDevice();
	blasInstances = new Buffer( (DRAGONS + 1) * sizeof( BLASInstance ), instance );
	blasInstances->CopyToDevice();
	dragonNodes = new Buffer( dragon.usedBlocks * sizeof( bvhvec4 ), dragon.bvh8Data );
	dragonTris = new Buffer( dragon.idxCount * 3 * sizeof( bvhvec4 ), dragon.bvh8Tris );
	drVerts = new Buffer( dragonTriCount * 3 * sizeof( bvhvec4 ), dragonVerts );
	dragonNodes->CopyToDevice();
	dragonTris->CopyToDevice();
	drVerts->CopyToDevice();
	bistroNodes = new Buffer( bistro.usedBlocks * sizeof( bvhvec4 ), bistro.bvh8Data );
	bistroTris = new Buffer( bistro.idxCount * 3 * sizeof( bvhvec4 ), bistro.bvh8Tris );
	bistroVerts = new Buffer( triCount * 3 * sizeof( bvhvec4 ), verts );
	bistroNodes->CopyToDevice();
	bistroTris->CopyToDevice();
	bistroVerts->CopyToDevice();

	// load camera position / direction from file
	std::fstream t = std::fstream{ "camera_gpu.bin", t.binary | t.in };
	if (!t.is_open()) return;
	t.read( (char*)&rd, sizeof( rd ) );
}

// Keyboard handling
bool UpdateCamera( float delta_time_s, fenster& f )
{
	bvhvec3 right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), rd.view ) );
	bvhvec3 up = 0.8f * tinybvh_cross( rd.view, right );
#ifdef AUTOCAM
	// playback camera spline
	static float ct = 0, moved = 1;
	ct += delta_time_s * 0.25f;
	if (ct > 10) ct -= 10;
	splineCam( ct + 1 );
#else
	// emit camera pos & dir
	static bool pdown = false;
	if (!GetAsyncKeyState( 'P' )) pdown = false; else if (!pdown)
	{
		FILE* f = fopen( "path.txt", "a" );
		fprintf( f, "bvhvec3( %.3ff, %.3ff, %.3ff ),", rd.eye.x, rd.eye.y, rd.eye.z );
		bvhvec3 t = rd.eye + rd.view;
		fprintf( f, "bvhvec3( %.3ff, %.3ff, %.3ff ),\n", t.x, t.y, t.z );
		printf( "point emitted.\n" );
		fclose( f );
		pdown = true;
	}
	// get camera controls
	float moved = 0, spd = 10.0f * delta_time_s;
	if (f.keys['A'] || f.keys['D']) rd.eye += right * (f.keys['D'] ? spd : -spd), moved = 1;
	if (f.keys['W'] || f.keys['S']) rd.eye += rd.view * (f.keys['W'] ? spd : -spd), moved = 1;
	if (f.keys['R'] || f.keys['F']) rd.eye += up * 2.0f * (f.keys['R'] ? spd : -spd), moved = 1;
	if (f.keys[20]) rd.view = tinybvh_normalize( rd.view + right * -0.1f * spd ), moved = 1;
	if (f.keys[19]) rd.view = tinybvh_normalize( rd.view + right * 0.1f * spd ), moved = 1;
	if (f.keys[17]) rd.view = tinybvh_normalize( rd.view + up * -0.1f * spd ), moved = 1;
	if (f.keys[18]) rd.view = tinybvh_normalize( rd.view + up * 0.1f * spd ), moved = 1;
#endif
	// recalculate right, up
	right = tinybvh_normalize( tinybvh_cross( bvhvec3( 0, 1, 0 ), rd.view ) );
	up = 0.8f * tinybvh_cross( rd.view, right );
	bvhvec3 C = rd.eye + 1.2f * rd.view;
	rd.p0 = C - right + up, rd.p1 = C + right + up, rd.p2 = C - right - up;
	return moved > 0;
}

// Application Tick
void Tick( float delta_time_s, fenster& f, uint32_t* buf )
{
	// handle user input and update camera
	int N = SCRWIDTH * SCRHEIGHT;
	if (UpdateCamera( delta_time_s, f ) || frameIdx++ == 0)
	{
		clear->SetArguments( accumulator );
		clear->Run( N );
		spp = 1;
	}
	// wavefront step 0: render on the GPU
	init->SetArguments( N, rd.eye, rd.p0, rd.p1, rd.p2,
		frameIdx, SCRWIDTH, SCRHEIGHT,
		bistroNodes, bistroTris, bistroVerts, dragonNodes, dragonTris, drVerts,
		tlasNodes, tlasIndices, blasInstances,
		noise
	);
	init->Run( 1 ); // init atomic counters, set buffer ptrs etc.
	rayGen->SetArguments( raysOut, spp * 19191 );
	rayGen->Run2D( oclint2( SCRWIDTH, SCRHEIGHT ) );
	for (int i = 0; i < 3; i++)
	{
		swap( raysOut, raysIn );
		extend->SetArguments( raysIn );
		extend->Run( computeUnits * 64 * 16, 64 );
		updateCounters1->Run( 1 );
		shade->SetArguments( accumulator, raysIn, raysOut, connections, spp - 1 );
		shade->Run( computeUnits * 64 * 16, 64 );
		updateCounters2->Run( 1 );
	}
	traceShadows->SetArguments( accumulator, connections );
	traceShadows->Run( computeUnits * 64 * 8, 64 );
	finalize->SetArguments( accumulator, 1.0f / (float)spp++, pixels );
	finalize->Run2D( oclint2( SCRWIDTH, SCRHEIGHT ) );
	pixels->CopyFromDevice();
	memcpy( buf, pixels->GetHostPtr(), N * sizeof( uint32_t ) );
	// print frame time / rate in window title
	char title[50];
	sprintf( title, "tiny_bvh %.2f s %.2f Hz", delta_time_s, 1.0f / delta_time_s );
	fenster_update_title( &f, title );
}

// Application Shutdown
void Shutdown()
{
	// save camera position / direction to file
	std::fstream s = std::fstream{ "camera_gpu.bin", s.binary | s.out };
	s.write( (char*)&rd, sizeof( rd ) );
}