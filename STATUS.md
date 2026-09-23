# KairoGeometry Status

Wave: A — foundation certification  
Frozen v1 target: 95/100  
Source gate: complete  
Execution gate: `cmake --build <build> && ctest --test-dir <build> --output-on-failure`

## Frozen v1 scope

Geometry v1 owns value-type Ray, Line, Segment, Plane, Sphere, Triangle, AABB, Capsule, OBB and Frustum primitives plus closest-point, distance, ray and audited pairwise intersection queries. Spatial indexing, physics manifolds and renderer culling policy stay in their owning repositories.

## 95 exit evidence

- The intended primitive/query matrix is implemented.
- Existing deterministic unit/stress tests cover analytic behavior and camera/frustum cases.
- `KairoGeometryCertification` executes 25,000 seeded property checks over symmetric intersection contracts and closest-point containment.
- The layer remains allocation-free/value-oriented and depends only on KairoMath.
- No v1 feature category remains planned.

## Verification policy

The certification executable emits a machine-readable `kairo.geometry.certification.v1` result. A release is marked verified only when the tests run at the exact pinned SHA.
