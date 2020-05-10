# AOEmbree

Compute AO per vertex using Embree

- Distribute points on a hemisphere used as ray directions

- Translate and rotate these directions to the vertex position and vertex normal

- Sum the number of rays intersecting a triangle divided by the number of rays

- Output AO as vertex color

