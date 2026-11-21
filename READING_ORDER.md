# KairoGeometry — source reading order and API notebook

Start with geometric primitives and bounding boxes, then ray/plane/triangle and shape intersections, closest points and distances, and finally view/frustum bounds. This library consumes KairoMath and supplies immutable geometry contracts to KairoSpatial, PhysicsMath, Renderer and RayTracer.

**Before reading:** [README.md](README.md) for setup, [CMakeLists.txt](CMakeLists.txt) if present for module boundaries, [LICENSE](LICENSE) for original-code reuse. Read each exported/public interface, implementation, its imported sibling APIs and matching tests. The sequence below links **all tracked implementation and test source files** in a suggested conceptual order; it is not an execution trace or a claim of complete verification.

## Core API and interaction questions

API contracts: rays versus segments, normalized directions, epsilon handling, inside/outside tests, degenerate geometry, world/local spaces, and robust intersection outcomes.

## Ordered source files (19)

1. [`KairoGeometry.cppm`](KairoGeometry.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
2. [`Ray.cppm`](Ray.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
3. [`Plane.cppm`](Plane.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
4. [`Line.cppm`](Line.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
5. [`AABB.cppm`](AABB.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
6. [`OBB.cppm`](OBB.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
7. [`Capsule.cppm`](Capsule.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
8. [`Triangle.cppm`](Triangle.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
9. [`Distance.cppm`](Distance.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
10. [`ClosestPoint.cppm`](ClosestPoint.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
11. [`Intersection.cppm`](Intersection.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
12. [`Frustum.cppm`](Frustum.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
13. [`Segment.cppm`](Segment.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
14. [`Sphere.cppm`](Sphere.cppm) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
15. [`tests/CertificationTests.cpp`](tests/CertificationTests.cpp) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
16. [`tests/GeometryTests.cpp`](tests/GeometryTests.cpp) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
17. [`tests/smoke.cpp`](tests/smoke.cpp) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
18. [`visual/server.py`](visual/server.py) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.
19. [`visual/VisualTests.cpp`](visual/VisualTests.cpp) — identify this file's public imports/includes, state ownership, exported functions, edge cases, and call sites in the next files or tests.

## Use it as a learning exercise

Choose one API, write its preconditions/postconditions and units, trace the caller in the next layer, and create a minimal deterministic test for the happy case, an invalid-input case and a precision/boundary case. Comment each function's **reason, invariant, output and interaction with other files**, not merely its syntax. Original code is [MIT-licensed](LICENSE) for studying and further development; external dependencies retain their own licenses. Do not infer a finished product or a passed native build from this inventory.
