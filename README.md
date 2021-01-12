# AOEmbree

<img src="/pictures/Screenshot3.jpg" width="500"/>
<img src="/pictures/Screenshot1.jpg" width="500"/>
<img src="/pictures/Screenshot2.JPG" width="300"/>


## Compute AO per vertex using Embree

- Distribute points on a hemisphere used as ray directions
- Translate and rotate these directions to the vertex position and vertex normal
- Sum the number of rays intersecting a triangle divided by the number of rays
- Output AO as vertex color

## Compilation

Download or compile embree: https://github.com/embree/embree

```bash
$> git clone https://github.com/nezix/AOEmbree
$> cd AOEmbree
$> mkdir build && cd build
$> cmake .. -Dembree_DIR="/mypath/embree" -Dtbb_DIR="/mypath/tbb/cmake" #set embree_DIR and tbb_DIR
$> make -j #or open visual studio solution with "open AOEmbree.sln" and compile the project
```
## Run it

```bash
$> ./AOEmbree -i /mypath/to/file.obj -s 128 -d 20.0 > output.obj #Use 128 sample points for each vertex and maximum ray distance is 20.0 units
```
It should work on Window10/Linux/MacOs.
