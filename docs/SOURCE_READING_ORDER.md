# Teaching source map — KairoGeometry

Geometric primitives and intersection predicates; built on KairoMath.

Follow source links sequentially. For each API/function, identify its input contract, owning data structure, called modules, failure modes, outputs, test and downstream consumers.

## Read in this order

This is a **conceptual sequence**, not CMake's dependency order. Before
changing public APIs, cross-check imports and tests. Begin with the
[KairoMath source guide](https://github.com/swayam8624/KairoMath/blob/docs/teaching-walkthrough-20261121/docs/SOURCE_READING_ORDER.md)
for vector, matrix, transform and floating-point conventions.

### Part 1 — Build and primitive meaning

1. [`CMakeLists.txt`](../CMakeLists.txt): module-file set, targets and test setup.
2. [`KairoGeometry.cppm`](../KairoGeometry.cppm): the public umbrella and its re-export contracts.
3. [`Ray.cppm`](../Ray.cppm): ray origin/direction and valid half-line parameter values.
4. [`Line.cppm`](../Line.cppm): infinite-line domain; why a line hit is not necessarily a ray hit.
5. [`Segment.cppm`](../Segment.cppm): endpoint domain and finite-length assumptions.
6. [`Plane.cppm`](../Plane.cppm): normal/distance convention and signed-side queries.
7. [`Triangle.cppm`](../Triangle.cppm): vertex winding, barycentric meaning and degeneracy.

**Exercise:** identify in source which methods expect normalized directions,
whether endpoints are included, and how parallel or degenerate inputs behave.
Do not assume that all APIs share one epsilon.

### Part 2 — Boundaries, orientations and visibility

8. [`Sphere.cppm`](../Sphere.cppm): radial distance and squared-distance shortcuts.
9. [`Capsule.cppm`](../Capsule.cppm): segment-plus-radius representation.
10. [`AABB.cppm`](../AABB.cppm): axis-aligned min/max contracts for broadphase culling.
11. [`OBB.cppm`](../OBB.cppm): orientation and local/world coordinate conversion.
12. [`Frustum.cppm`](../Frustum.cppm): camera clip-volume classification.

**Exercise:** compare an AABB and an OBB around the same rotated object.
A bounding-volume overlap is a *candidate*, not proof of actual triangle contact.

### Part 3 — Geometric queries and tests

13. [`ClosestPoint.cppm`](../ClosestPoint.cppm): a geometric location as output.
14. [`Distance.cppm`](../Distance.cppm): scalar distances and their unit/scale.
15. [`Intersection.cppm`](../Intersection.cppm): boolean/hit tests and edge cases.
16. [`tests/GeometryTests.cpp`](../tests/GeometryTests.cpp): common-case assertions.
17. [`tests/CertificationTests.cpp`](../tests/CertificationTests.cpp): deterministic certification checks.
18. [`tests/smoke.cpp`](../tests/smoke.cpp): minimal consumer integration.
19. [`visual/VisualTests.cpp`](../visual/VisualTests.cpp): visual experiment entry point.
20. [`visual/server.py`](../visual/server.py): local visualization bridge.

**Exercise:** for a ray and triangle, check hits, misses, parallel rays,
back-face policy, grazing edges and zero-area triangles against the
implementation and tests. Record where behavior is unspecified rather than
inventing a universal guarantee.

## How this repository interacts with others

Spatial acceleration and ray tracing consume these primitive/intersection types; physics narrowphase depends on stable contact geometry. Separate shape representation from robust intersection predicates.

## Verification and study rules

- Read the public C++23 module/entrypoint first, then its implementation and tests.
- Explain invariants, ownership, error states, concurrency or numerical assumptions in comments, not trivial syntax.
- A passing CI job covers only tests actually run on that commit/host; source presence is not runtime certification.
- Check [STATUS.md](../STATUS.md) and [STATUS.yaml](../STATUS.yaml) for current explicit verification limitations.
- Original work may be modified and extended under the repository license; third-party content retains upstream terms.


## Complete source-file inventory

After the curated teaching path, follow this deterministic inventory of first-party source, scripts, shaders, and tests. Grouping is a suggested learning order, not an indication that every function is already annotated. Vendored dependencies, ROMs, binaries, data, media, and generated evidence are excluded.

### Entry points and build (1)

1. [`CMakeLists.txt`](../CMakeLists.txt)

### Interfaces and modules (14)

1. [`AABB.cppm`](../AABB.cppm)
2. [`Capsule.cppm`](../Capsule.cppm)
3. [`ClosestPoint.cppm`](../ClosestPoint.cppm)
4. [`Distance.cppm`](../Distance.cppm)
5. [`Frustum.cppm`](../Frustum.cppm)
6. [`Intersection.cppm`](../Intersection.cppm)
7. [`KairoGeometry.cppm`](../KairoGeometry.cppm)
8. [`Line.cppm`](../Line.cppm)
9. [`OBB.cppm`](../OBB.cppm)
10. [`Plane.cppm`](../Plane.cppm)
11. [`Ray.cppm`](../Ray.cppm)
12. [`Segment.cppm`](../Segment.cppm)
13. [`Sphere.cppm`](../Sphere.cppm)
14. [`Triangle.cppm`](../Triangle.cppm)

### Implementation and other source (2)

1. [`visual/server.py`](../visual/server.py)
2. [`visual/VisualTests.cpp`](../visual/VisualTests.cpp)

### Verification and tests (3)

1. [`tests/CertificationTests.cpp`](../tests/CertificationTests.cpp)
2. [`tests/GeometryTests.cpp`](../tests/GeometryTests.cpp)
3. [`tests/smoke.cpp`](../tests/smoke.cpp)

Snapshot totals: 20 first-party source/build/test files, 30 tracked files overall. Re-audit when files are renamed or added.
