#include <embree3/rtcore.h>
#include <stdio.h>
#include <math.h>
#include <limits>
#include "cxxopts.hpp"

#include "AOEmbree.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

# define M_PI 3.14159265358979323846

#include <chrono>
using namespace std::chrono;

#include <tbb/parallel_for.h>

#define DEBUG 0


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

static void CalcNormal(float N[3], float v0[3], float v1[3], float v2[3]) {
    float v10[3];
    v10[0] = v1[0] - v0[0];
    v10[1] = v1[1] - v0[1];
    v10[2] = v1[2] - v0[2];

    float v20[3];
    v20[0] = v2[0] - v0[0];
    v20[1] = v2[1] - v0[1];
    v20[2] = v2[2] - v0[2];

    N[0] = v10[1] * v20[2] - v10[2] * v20[1];
    N[1] = v10[2] * v20[0] - v10[0] * v20[2];
    N[2] = v10[0] * v20[1] - v10[1] * v20[0];

    float len2 = N[0] * N[0] + N[1] * N[1] + N[2] * N[2];
    if (len2 > 0.0f) {
        float len = sqrtf(len2);

        N[0] /= len;
        N[1] /= len;
        N[2] /= len;
    }
}

std::vector<float> computeVertexNormals(const tinyobj::attrib_t& attrib, const tinyobj::shape_t& shape) {

    std::vector<float> result;
    result.reserve(attrib.vertices.size());

    for (int i = 0; i < attrib.vertices.size(); i++) {
        result.push_back(0.0f);
    }

    for (size_t f = 0; f < shape.mesh.indices.size() / 3; f++) {
        // Get the three indexes of the face (all faces are triangular)
        tinyobj::index_t idx0 = shape.mesh.indices[3 * f + 0];
        tinyobj::index_t idx1 = shape.mesh.indices[3 * f + 1];
        tinyobj::index_t idx2 = shape.mesh.indices[3 * f + 2];

        // Get the three vertex indexes and coordinates
        int vi[3];      // indexes
        float v[3][3];  // coordinates

        for (int k = 0; k < 3; k++) {
            vi[0] = idx0.vertex_index;
            vi[1] = idx1.vertex_index;
            vi[2] = idx2.vertex_index;
            assert(vi[0] >= 0);
            assert(vi[1] >= 0);
            assert(vi[2] >= 0);

            v[0][k] = attrib.vertices[3 * vi[0] + k];
            v[1][k] = attrib.vertices[3 * vi[1] + k];
            v[2][k] = attrib.vertices[3 * vi[2] + k];
        }

        // Compute the normal of the face
        float normal[3];
        CalcNormal(normal, v[0], v[1], v[2]);

        for (size_t i = 0; i < 3; ++i) {
            vec3 n(result[vi[i] * 3] + normal[0], result[vi[i] * 3 + 1] + normal[1], result[vi[i] * 3 + 2] + normal[2]);

            n = glm::normalize(n);

            result[vi[i] * 3] = n.x;
            result[vi[i] * 3 + 1] = n.y;
            result[vi[i] * 3 + 2] = n.z;
        }
    }

    return result;

}

void computeAOPerVert(float *verts, float *norms, int *tris, float *result,
                      int vcount, int icount,
                      int samplesAO, float maxDist) {

#if DEBUG
    auto timerstart = high_resolution_clock::now();
#endif

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

            quat q = glm::rotation(oriVec, normalize(normal));

            for (int s = 0; s < rayDir.size(); s++) {
                vec3 dir = rayDir[s];

                vec3 rotatedDir = q * dir;

                rays[s].org_x = vertices[i * 3];
                rays[s].org_y = vertices[i * 3 + 1];
                rays[s].org_z = vertices[i * 3 + 2];
                rays[s].dir_x = rotatedDir.x;
                rays[s].dir_y = rotatedDir.y;
                rays[s].dir_z = rotatedDir.z;
                rays[s].tnear = 0.01f;
                rays[s].tfar = maxDist;
                rays[s].mask = 0;
                rays[s].flags = 0;

            }

            rtcOccluded1M(scene, &context, &rays[0], rayDir.size(), sizeof(RTCRay));

            float totalAO = 0.0f;

            for (int s = 0; s < rayDir.size(); s++) {
                if (rays[s].tfar < 0.0f) {//Hit
                    totalAO += 1.0f;
                }
            }

            result[i] = 1.0f - (totalAO / rayDir.size());
        }
    });

    /* Though not strictly necessary in this example, you should
     * always make sure to release resources allocated through Embree. */
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

#if DEBUG
    auto timerstop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(timerstop - timerstart).count();
    std::cerr << duration << " ms" << std::endl;
#endif
}


int main(int argc, char **argv) {



    cxxopts::Options options("AOEmbree", "Compute per vertex ambient occlusion using embree");

    bool onlyAO = false;
    options.add_options()
    ("i,input", "OBJ input file", cxxopts::value<std::string>())
    ("s,samples", "Number of ray sample for each vertex", cxxopts::value<int>()->default_value("128"))
    ("d,dist", "Maximum ray distance", cxxopts::value<float>()->default_value("20.0"))
    ("a,ao", "Only output per vertex AO values", cxxopts::value<bool>(onlyAO))
    ("h,help", "Print usage")
    ;

    auto argsresult = options.parse(argc, argv);

    if (argsresult.count("help"))
    {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    int samplesAO = argsresult["samples"].as<int>();
    float maxDist = argsresult["dist"].as<float>();
    std::string path;
    if (argsresult.count("input"))
        path = argsresult["input"].as<std::string>();
    else {
        std::cout << options.help() << std::endl;
        exit(0);
    }


    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str());

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
        if (attrib.normals.size() == 0) {
            norms = computeVertexNormals(attrib, shapes[s]);
        }

        for (int f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];

            for (size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                ids.push_back(idx.vertex_index);
            }
            index_offset += fv;
        }
        break;
    }

    float *result = new float[verts.size() / 3];
    computeAOPerVert(verts.data(), norms.data(), ids.data(), result,
                     verts.size() / 3, ids.size() / 3,
                     samplesAO, maxDist);

    if (onlyAO) {
        for (int i = 0; i < verts.size() / 3; i++) {
            printf("%.3f ", result[i]);
        }
        return 0;
    }

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
