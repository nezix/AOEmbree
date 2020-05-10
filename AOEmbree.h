#include <vector>
#include <embree3/rtcore.h>

#include <fstream>
#include <sstream>
#include <string>

#include <iostream>

#include "glm/glm.hpp"
// #include <glm/gtx/quaternion.hpp>
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"
using namespace glm;

// struct float3
// {
// 	float x;
// 	float y;
// 	float z;
// };


void computeAOPerVert(float *vertices, float *normals, int *indices, float *result,
                      int vcount, int icount,
                      int samplesAO, float maxDist);


template <class Container>
void split1(const std::string& str, Container& cont)
{
    std::istringstream iss(str);
    std::copy(std::istream_iterator<std::string>(iss),
         std::istream_iterator<std::string>(),
         std::back_inserter(cont));
}