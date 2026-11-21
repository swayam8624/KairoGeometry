# Teaching source map — KairoGeometry

Geometric primitives and intersection predicates; built on KairoMath.

Follow source links sequentially. For each API/function, identify its input contract, owning data structure, called modules, failure modes, outputs, test and downstream consumers.

## Read in this order

1. [`CMakeLists.txt`](../CMakeLists.txt)
2. [`KairoGeometry.cppm`](../KairoGeometry.cppm)
3. [`Ray.cppm`](../Ray.cppm)
4. [`Line.cppm`](../Line.cppm)
5. [`Segment.cppm`](../Segment.cppm)
6. [`Plane.cppm`](../Plane.cppm)
7. [`Triangle.cppm`](../Triangle.cppm)
8. [`Sphere.cppm`](../Sphere.cppm)
9. [`Capsule.cppm`](../Capsule.cppm)
10. [`AABB.cppm`](../AABB.cppm)
11. [`OBB.cppm`](../OBB.cppm)
12. [`Frustum.cppm`](../Frustum.cppm)
13. [`ClosestPoint.cppm`](../ClosestPoint.cppm)
14. [`Distance.cppm`](../Distance.cppm)
15. [`Intersection.cppm`](../Intersection.cppm)
16. [`tests`](../tests)
17. [`visual`](../visual)

## How this repository interacts with others

Spatial acceleration and ray tracing consume these primitive/intersection types; physics narrowphase depends on stable contact geometry. Separate shape representation from robust intersection predicates.

## Verification and study rules

- Read the public C++23 module/entrypoint first, then its implementation and tests.
- Explain invariants, ownership, error states, concurrency or numerical assumptions in comments, not trivial syntax.
- A passing CI job covers only tests actually run on that commit/host; source presence is not runtime certification.
- Check [STATUS.md](../STATUS.md) and [STATUS.yaml](../STATUS.yaml) for current explicit verification limitations.
- Original work may be modified and extended under the repository license; third-party content retains upstream terms.
