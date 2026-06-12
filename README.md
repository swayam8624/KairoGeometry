# KairoGeometry

`KairoGeometry` is the Foundation geometry layer built on top of
`Kairo.Foundation.Math.Vector`.

The module is intentionally small and value-type oriented. Geometry primitives
are transparent `struct`s, allocation-free, trivially copyable, and suitable for
engine math, editor tooling, serialization, broad-phase collision, and later
render/ray-query systems.

## Modules

Import the umbrella module when you want the full geometry surface:

```cpp
import Kairo.Foundation.Geometry;
```

Import individual modules when you want narrower dependencies:

```cpp
import Kairo.Foundation.Geometry.Ray;
import Kairo.Foundation.Geometry.Line;
import Kairo.Foundation.Geometry.Segment;
import Kairo.Foundation.Geometry.Plane;
import Kairo.Foundation.Geometry.Sphere;
import Kairo.Foundation.Geometry.Triangle;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Geometry.Capsule;
import Kairo.Foundation.Geometry.OBB;
import Kairo.Foundation.Geometry.Frustum;
import Kairo.Foundation.Geometry.ClosestPoint;
import Kairo.Foundation.Geometry.Distance;
import Kairo.Foundation.Geometry.Intersection;
```

## Conventions

- All primitives use `kairo::foundation::math::Vector3<T>` and the `FloatingPoint`
  concept from `KairoMath`.
- `T` must be a floating-point scalar.
- Distances and intersection helpers assume normalized directions/normals unless
  a function explicitly normalizes input.
- `NearlyEqual()` overloads use KairoMath scalar/vector tolerance helpers.
- Type aliases use the existing math naming style: `Rayf`, `Planef`, `Spheref`,
  `Trianglef`, and matching `double` aliases.

## Current Primitives

| Primitive | Representation | Main Tasks |
| --- | --- | --- |
| `Ray<T>` | origin and direction | point evaluation, normalization, closest point, point distance |
| `Line<T>` | point and infinite direction | analytic closest point and distance queries |
| `Segment<T>` | finite start and end points | point distance, segment distance, capsule support, constraint/contact foundations |
| `Plane<T>` | normal and signed distance | signed distance, projection, reflection, normalization |
| `Sphere<T>` | center and radius | containment, closest point, signed distance, expansion, merge, intersection |
| `Triangle<T>` | vertices `A`, `B`, `C` | edges, area, centroid, normal, plane conversion, barycentric coordinates, containment |
| `AABB<T>` | min and max corners | containment, closest point, distance, merge, intersection, transform |
| `Capsule<T>` | segment endpoints and radius | axis projection, closest point, signed distance, containment, surface area, volume |
| `OBB<T>` | center, orthonormal axes, half extents | local/world conversion, closest point, containment, AABB conversion, SAT intersection support |
| `Frustum<T>` | six inward-facing planes | camera culling for points, spheres, AABBs, OBBs, and debug corners |

## Query Modules

`ClosestPoint`, `Distance`, and `Intersection` provide named cross-primitive
helpers such as `ClosestPointOnTriangle()`, `ClosestPointOnSegment()`,
`DistanceToCapsule()`, `DistanceBetweenSegments()`, `RayAABB()`, `RayOBB()`,
`RayCapsule()`, `SphereTriangle()`, `AABBCapsule()`, `TriangleAABB()`,
`TriangleOBB()`, `TriangleCapsule()`, `TriangleTriangle()`, `CapsuleCapsule()`,
and `OBBOBB()`.

The intersection layer covers the current primitive collision matrix for rays,
planes, spheres, triangles, AABBs, OBBs, and capsules. Symmetric pair overloads
are provided for `Intersects(...)` where call order should not matter.

### Verified Foundation Status

The current Phase A geometry foundation is complete for primitive definitions,
closest-point queries, distance queries, ray casts, and the narrow-phase
intersection matrix across `Ray`, `Plane`, `Sphere`, `Triangle`, `AABB`, `OBB`,
and `Capsule`.

Generic overloads that already belong to primitive modules are not redeclared in
the aggregate query modules. This keeps C++ module ownership unambiguous while
still exposing readable named helpers for shared query code.

## Build

`KairoGeometry` is a C++23 module library. It depends on `KairoMath` but does
not vendor or copy any KairoMath source into this repository.

Clone with submodules when you want the repo to bring its pinned `KairoMath`
dependency with it:

```sh
git clone --recursive https://github.com/swayam8624/KairoGeometry.git
```

If you already cloned without submodules, initialize the dependency with:

```sh
git submodule update --init --recursive
```

Dependency resolution order:

1. `-DKAIRO_MATH_SOURCE_DIR=/path/to/KairoMath` when you want to point at an
   explicit local checkout.
2. `external/KairoMath` when the repository is cloned recursively.
3. `../KairoMath` when developing inside the `Kairo/Foundation` workspace.
4. `https://github.com/swayam8624/KairoMath.git` through CMake `FetchContent`
   when no local checkout is available.

Configure with Ninja and a compiler that supports CMake C++ module dependency
scanning:

```sh
cmake -G Ninja \
  -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ \
  -S Foundation/KairoGeometry \
  -B Foundation/KairoGeometry/cmake-build-geometry-llvm

cmake --build Foundation/KairoGeometry/cmake-build-geometry-llvm
./Foundation/KairoGeometry/cmake-build-geometry-llvm/KairoGeometrySmoke
```

The `cmake-build-*` folders are generated build artifacts. They can contain
compiled `KairoMath` objects because CMake builds dependencies under the active
build directory; those files are ignored and should not be committed.
