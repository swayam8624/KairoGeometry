# KairoGeometry: teaching source reading order

This file is an executable-path-oriented study sequence, **not** a build-order claim. Read [README](../README.md), [CMakeLists](../CMakeLists.txt), and [tests](../tests/) before assuming all optional features compile on every platform.

This package covers value-type geometry, distances, closest-point and intersection queries. Read the modules in the following progression, then return to their tests and dependent Kairo packages.

1. [Ray.cppm](../Ray.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
2. [Line.cppm](../Line.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
3. [Segment.cppm](../Segment.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
4. [Plane.cppm](../Plane.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
5. [Sphere.cppm](../Sphere.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
6. [Triangle.cppm](../Triangle.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
7. [AABB.cppm](../AABB.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
8. [Capsule.cppm](../Capsule.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
9. [OBB.cppm](../OBB.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
10. [Frustum.cppm](../Frustum.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
11. [ClosestPoint.cppm](../ClosestPoint.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
12. [Distance.cppm](../Distance.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
13. [Intersection.cppm](../Intersection.cppm) — read public types, inputs/outputs, invariants and callers before proceeding
14. [KairoGeometry.cppm](../KairoGeometry.cppm) — umbrella C++23 module exporting the focused units

## Cross-repository data flow

KairoMath vectors → primitive shapes → ClosestPoint/Distance/Intersection → KairoSpatial BVH/indices and KairoPhysicsEngine collision detection.

## How to read each function

For each class/function, identify its **input and output types**, valid ranges, ownership/lifetimes, numerical convention, exceptions/failure states, asymptotic cost where applicable, and the next consumer. Follow `import` declarations to the exact modules they depend on; C++23 `export import` re-exports an interface, and does not turn a dependency into an implementation of the current module.

## Exercises

1. Choose a public API and trace at least two actual caller/callee boundaries through the associated modules.
2. Find a degenerate case (empty data, singular geometry, invalid timestep, zero axis, or stale handle, as applicable) and locate the validated behaviour.
3. Run the tests for the API and write down the invariant each test proves; avoid inferring platform-wide correctness from one passing smoke test.
4. Compare the API's data representation against its downstream physics/rendering consumer, especially coordinate conventions and ownership.
5. Inspect compiler flags and module build requirements in CMake before benchmarking.

## Licensing and provenance

Original repository-authored source and this guide are intended for study, reuse, modification, and further development under [LICENSE](../LICENSE). This does not relicense third-party code, tools, datasets or bundled assets, if present; retain external notices and verify their provenance.
