#include <embree3/rtcore.h>
#include <stdio.h>
#include <math.h>
#include <limits>

#include "AOEmbree.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

# define M_PI 3.14159265358979323846

#include <chrono>
using namespace std::chrono;

#include <tbb/parallel_for.h>

void errorFunction(void* userPtr, enum RTCError error, const char* str)
{
	printf("error %d: %s\n", error, str);
}


RTCDevice initializeDevice()
{
	RTCDevice device = rtcNewDevice(NULL);

	if (!device)
		printf("error %d: cannot create device\n", rtcGetDeviceError(NULL));

	rtcSetDeviceErrorFunction(device, errorFunction, NULL);
	return device;
}

RTCScene initializeScene(RTCDevice device)
{
	RTCScene scene = rtcNewScene(device);

	/*
	 * Create a triangle mesh geometry, and initialize a single triangle.
	 * You can look up geometry types in the API documentation to
	 * find out which type expects which buffers.
	 *
	 * We create buffers directly on the device, but you can also use
	 * shared buffers. For shared buffers, special care must be taken
	 * to ensure proper alignment and padding. This is described in
	 * more detail in the API documentation.
	 */
	RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);


	float* vertices = (float*) rtcSetNewGeometryBuffer(geom,
	                  RTC_BUFFER_TYPE_VERTEX,
	                  0,
	                  RTC_FORMAT_FLOAT3,
	                  3 * sizeof(float),
	                  3);

	unsigned* indices = (unsigned*) rtcSetNewGeometryBuffer(geom,
	                    RTC_BUFFER_TYPE_INDEX,
	                    0,
	                    RTC_FORMAT_UINT3,
	                    3 * sizeof(unsigned),
	                    1);

	if (vertices && indices)
	{
		vertices[0] = 0.f; vertices[1] = 0.f; vertices[2] = 0.f;
		vertices[3] = 1.f; vertices[4] = 0.f; vertices[5] = 0.f;
		vertices[6] = 0.f; vertices[7] = 1.f; vertices[8] = 0.f;

		indices[0] = 0; indices[1] = 1; indices[2] = 2;
	}

	/*
	 * You must commit geometry objects when you are done setting them up,
	 * or you will not get any intersections.
	 */
	rtcCommitGeometry(geom);

	/*
	 * In rtcAttachGeometry(...), the scene takes ownership of the geom
	 * by increasing its reference count. This means that we don't have
	 * to hold on to the geom handle, and may release it. The geom object
	 * will be released automatically when the scene is destroyed.
	 *
	 * rtcAttachGeometry() returns a geometry ID. We could use this to
	 * identify intersected objects later on.
	 */
	rtcAttachGeometry(scene, geom);
	rtcReleaseGeometry(geom);

	/*
	 * Like geometry objects, scenes must be committed. This lets
	 * Embree know that it may start building an acceleration structure.
	 */
	rtcCommitScene(scene);

	return scene;
}

void rotate_vector_by_quaternion(const vec3& v, const quat& q, vec3& vprime)
{
	// Extract the vector part of the quaternion
	vec3 u(q.x, q.y, q.z);

	// Extract the scalar part of the quaternion
	float s = q.w;

	// Do the math
	vprime = 2.0f * dot(u, v) * u
	         + (s * s - dot(u, u)) * v
	         + 2.0f * s * cross(u, v);
}


void computeAOPerVert(float *verts, float *norms, int *tris, float *result,
                      int vcount, int icount,
                      int samplesAO, float maxDist) {

	auto timerstart = high_resolution_clock::now();

	RTCDevice device = initializeDevice();

	RTCScene scene = rtcNewScene(device);

	RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

	float* vertices = (float*) rtcSetNewGeometryBuffer(geom,
	                  RTC_BUFFER_TYPE_VERTEX,
	                  0,
	                  RTC_FORMAT_FLOAT3,
	                  3 * sizeof(float),
	                  vcount);

	unsigned* indices = (unsigned*) rtcSetNewGeometryBuffer(geom,
	                    RTC_BUFFER_TYPE_INDEX,
	                    0,
	                    RTC_FORMAT_UINT3,
	                    3 * sizeof(unsigned),
	                    icount);


	for (int i = 0; i < vcount * 3; i++) {
		vertices[i] = verts[i];
	}
	for (int i = 0; i < icount * 3; i++) {
		indices[i] = tris[i];
	}

	rtcCommitGeometry(geom);
	rtcAttachGeometry(scene, geom);
	rtcReleaseGeometry(geom);

	rtcCommitScene(scene);

	std::vector<vec3> rayDir;

	//Compute semi-sphere ray directions
	int samples = samplesAO * 2;
	float golden_angle = M_PI * (3 - sqrtf(5));
	float start =  1 - 1.0f / (int)samples;
	float end = 1.0f / (int)samples - 1;

	for (int i = 0; i < (int)samples; i++) {
		float theta = golden_angle * i;
		float z = start + i * (end - start) / (int)samples;
		float radius = sqrtf(1 - z * z);
		float y = radius * sin(theta);
		//Only keep the upper half of the sphere
		if (y > 0.01f) {
			float x = radius * cos(theta);
			float y = radius * sin(theta);
			rayDir.push_back(normalize(vec3(x, y, z)));
		}
	}

	float step = 1.0f / samplesAO;

	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	tbb::parallel_for( tbb::blocked_range<int>(0, vcount),
	                   [&](tbb::blocked_range<int> r)
	{
		RTCRay *rays = new RTCRay[rayDir.size()];

		for (int i = r.begin(); i < r.end(); ++i)
		{
			vec3 oriVec(0, 1, 0);

			vec3 normal(norms[i * 3], norms[i * 3 + 1], norms[i * 3 + 2]);

			quat q = glm::rotation(oriVec, normal);

			for (int s = 0; s < rayDir.size(); s++) {
				vec3 dir = rayDir[s];
				vec3 rotatedDir;

				rotate_vector_by_quaternion(dir, q, rotatedDir);

				rays[s].org_x = vertices[i * 3];
				rays[s].org_y = vertices[i * 3 + 1];
				rays[s].org_z = vertices[i * 3 + 2];
				rays[s].dir_x = rotatedDir.x;
				rays[s].dir_y = rotatedDir.y;
				rays[s].dir_z = rotatedDir.z;
				rays[s].tnear = 0.01;
				rays[s].tfar = maxDist;
				rays[s].mask = 0;
				rays[s].flags = 0;

			}

			rtcOccluded1M(scene, &context, &rays[0], rayDir.size(), sizeof(RTCRay));

			float totalAO = 0.0f;

			for (int s = 0; s < rayDir.size(); s++) {
				if (rays[s].tfar < 0.0f) {
					totalAO += 1.0f;
				}
			}

			result[i] = 1.0f - (totalAO / samplesAO);
		}
	});

	/* Though not strictly necessary in this example, you should
	 * always make sure to release resources allocated through Embree. */
	rtcReleaseScene(scene);
	rtcReleaseDevice(device);

	auto timerstop = high_resolution_clock::now();
	auto duration = duration_cast<milliseconds>(timerstop - timerstart).count();
	std::cerr << duration << " ms" << std::endl;
}


int main(int argc, char **argv) {

	int samplesAO = 128;
	float maxDist = 20.0f;

	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::string warn;
	std::string err;

	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, argv[1]);

	if (!warn.empty()) {
		std::cout << warn << std::endl;
	}

	if (!err.empty()) {
		std::cerr << err << std::endl;
	}

	std::vector<float> verts;
	std::vector<float> norms;
	std::vector<int> ids;

	verts = attrib.vertices;
	norms = attrib.normals;
	for (int s = 0; s < shapes.size(); s++) {
		size_t index_offset = 0;
		for (int f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
			int fv = shapes[s].mesh.num_face_vertices[f];

			for (size_t v = 0; v < fv; v++) {
				tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
				ids.push_back(idx.vertex_index);
			}
			index_offset += fv;
		}
	}

	float *result = new float[verts.size() / 3];
	computeAOPerVert(verts.data(), norms.data(), ids.data(), result,
	                 verts.size() / 3, ids.size() / 3,
	                 samplesAO, maxDist);


	for (int i = 0; i < verts.size() / 3; i++) {
		printf("v %.6f %.6f %.6f %.3f %.3f %.3f\n",
		       verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2], result[i], result[i], result[i]);
		printf("vn %.6f %.6f %.6f\n", norms[i * 3], norms[i * 3 + 1], norms[i * 3 + 2]);

	}

	for (int i = 0; i < ids.size() / 3; i++) {
		printf("f %d %d %d\n", ids[i * 3] + 1, ids[i * 3 + 1] + 1, ids[i * 3 + 2] + 1);
	}

	return 0;
}
