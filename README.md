# C++ CAD

This is a small library for outputting CAD models from C++.  It contains
implementations of tesselations of a couple of basic primitives
(sphere, cone / cylinder), and basic helper functions.

Most of the functionality comes from the
[CGAL library](https://www.cgal.org/), such as:

* Constructive solid geometry binary operations (difference, union).
* Convex hulls.
* Minkowski sums of solids.

CGAL supports input and output in a few formats, such as Waveform OBJ, OFF,
VRML.


## Example

```C++
#include <cpp_cad.h>


typedef cpp_cad::Nef_polyhedron_3 Nef_polyhedron_3;
typedef cpp_cad::Polyhedron_3 Polyhedron_3;


// Outputs the difference between a cube and a sphere as an OBJ file:
int main()
{
    // A 10 x 10 x 10 unit cube:
    Nef_polyhedron_3 nef_cube = Nef_polyhedron_3::make_cube(10, 10, 10);
    // A sphere with radius 10 units, tessalated with 16 divisions per great
    // circle:
    Nef_polyhedron_3 nef_sphere = Nef_polyhedron_3::make_sphere(10, 16);
    // The sphere subtracted from the cube:
    Nef_polyhedron_3 diff = nef_cube - nef_sphere;

    diff.write_to_obj_file("output.obj");

    return 0;
}
```


## Requirements

* C++ 11
* libCGAL


## License

Apache 2.0.


## Installation

### Ubuntu 16

Build a Debian package:

```BASH
./build_packages.sh
```

Install the package:

```BASH
sudo dpkg -i build/libcppcad_0.1.0.deb
```


### Other Platforms

* Package C++ CAD for your platform.
* Install the package.
* (Contribute packaging.)
