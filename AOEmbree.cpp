#include <embree3/rtcore.h>
#include <stdio.h>
#include <math.h>
#include <limits>

#include "AOEmbree.h"


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

bool castRay(RTCScene scene,
             float ox, float oy, float oz,
             float dx, float dy, float dz, float maxDist)
{
	/*
	 * The intersect context can be used to set intersection
	 * filters or flags, and it also contains the instance ID stack
	 * used in multi-level instancing.
	 */
	struct RTCIntersectContext context;
	rtcInitIntersectContext(&context);

	/*
	 * The ray hit structure holds both the ray and the hit.
	 * The user must initialize it properly -- see API documentation
	 * for rtcIntersect1() for details.
	 */
	struct RTCRayHit rayhit;
	rayhit.ray.org_x = ox;
	rayhit.ray.org_y = oy;
	rayhit.ray.org_z = oz;
	rayhit.ray.dir_x = dx;
	rayhit.ray.dir_y = dy;
	rayhit.ray.dir_z = dz;
	rayhit.ray.tnear = 0;
	rayhit.ray.tfar = maxDist;
	rayhit.ray.mask = 0;
	rayhit.ray.flags = 0;
	rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
	rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

	/*
	 * There are multiple variants of rtcIntersect. This one
	 * intersects a single ray with the scene.
	 */
	rtcIntersect1(scene, &context, &rayhit);

	// printf("%f, %f, %f: ", ox, oy, oz);
	if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
	{
		/* Note how geomID and primID identify the geometry we just hit.
		 * We could use them here to interpolate geometry information,
		 * compute shading, etc.
		 * Since there is only a single triangle in this scene, we will
		 * get geomID=0 / primID=0 for all hits.
		 * There is also instID, used for instancing. See
		 * the instancing tutorials for more information */
		// printf("Found intersection on geometry %d, primitive %d at tfar=%f\n",
		//        rayhit.hit.geomID,
		//        rayhit.hit.primID,
		//        rayhit.ray.tfar);
		return true;
	}
	return false;
	// else
	// printf("Did not find any intersection.\n");
}


void computeAOPerVert(float *verts, float *norms, int *tris, float *result,
                      int vcount, int icount,
                      int samplesAO, float maxDist) {


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
		float x = radius * cos(theta);
		float y = radius * sin(theta);
		if (y > 0.0f)//Only keep the upper half of the sphere
			rayDir.push_back(vec3(x, y, z));
	}

	float step = 1.0f / samplesAO;

	for (int i = 0 ; i < vcount; i++) {

		float totalAO = 0.0f;

		vec3 oriVec(0, 1, 0);

		vec3 normal(norms[i * 3], norms[i * 3 + 1], norms[i * 3 + 2]);

		quat q = glm::rotation(oriVec, normal);

		for (int s = 0; s < rayDir.size(); s++) {
			vec3 dir (
			    rayDir[s].x + vertices[i * 3],
			    rayDir[s].y + vertices[i * 3 + 1],
			    rayDir[s].z + vertices[i * 3 + 2]);


			quat dirq = quat(dir.x, dir.y, dir.z, 0.0f);
			quat tmp = q * dirq * conjugate(q);
			vec3 rotatedDir(tmp.x, tmp.y, tmp.z);
			rotatedDir = normalize(rotatedDir);


			bool inter = castRay(scene, vertices[i * 3], vertices[i * 3 + 1], vertices[i * 3 + 2],
			                     rotatedDir.x, rotatedDir.y, rotatedDir.z, maxDist);


			if (inter) {
				totalAO += step;
			}
		}
		result[i] = totalAO;
	}

	/* Though not strictly necessary in this example, you should
	 * always make sure to release resources allocated through Embree. */
	rtcReleaseScene(scene);
	rtcReleaseDevice(device);



}

int main(int argc, char **argv) {

	int samplesAO = 128;
	float maxDist = 10.0f;

	std::ifstream infile(argv[1]);

	std::vector<float> verts;
	std::vector<float> norms;
	std::vector<int> ids;

	std::string line;
	while (std::getline(infile, line))
	{

		if (line.size() > 0 && line[0] == 'v' && line[1] == ' ') {
			std::vector<std::string> words;
			split1(line.c_str(), words);
			float x = std::stof(words[1]);
			float y = std::stof(words[2]);
			float z = std::stof(words[3]);
			verts.push_back(x);
			verts.push_back(y);
			verts.push_back(z);
		}
		else if (line.size() > 0 && line[0] == 'f') {
			std::vector<std::string> words;
			split1(line.c_str(), words);
			ids.push_back(std::stoi(words[1]) - 1);
			ids.push_back(std::stoi(words[2]) - 1);
			ids.push_back(std::stoi(words[3]) - 1);
		}
		if (line.size() > 0 && line[0] == 'v' && line[1] == 'n') {
			std::vector<std::string> words;
			split1(line.c_str(), words);
			float x = std::stof(words[1]);
			float y = std::stof(words[2]);
			float z = std::stof(words[3]);
			norms.push_back(x);
			norms.push_back(y);
			norms.push_back(z);
		}
	}
	float *result = new float[verts.size() / 3];
	computeAOPerVert(verts.data(), norms.data(), ids.data(), result,
	                 verts.size() / 3, ids.size() / 3,
	                 samplesAO, maxDist);


	for (int i = 0; i < verts.size() / 3; i++) {
		printf("v %.6f %.6f %.6f %.3f %.3f %.3f\n",
		       verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2], result[i], result[i], result[i]);
	}

	for (int i = 0; i < ids.size() / 3; i++) {
		printf("f %d %d %d\n", ids[i * 3] + 1, ids[i * 3 + 1] + 1, ids[i * 3 + 2] + 1);
	}

	return 0;
}
