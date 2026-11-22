# Geometry laboratory: why crossing a bounding box does not mean hitting a triangle

Start at [AABB.cppm](../AABB.cppm), [Ray.cppm](../Ray.cppm), [Plane.cppm](../Plane.cppm), [Triangle.cppm](../Triangle.cppm) and [Intersection.cppm](../Intersection.cppm). Existing [geometric witnesses and intersections](GEOMETRIC_WITNESSES_AND_INTERSECTIONS_LAB.md) develops closest points and SAT in more detail. The downstream consumers are [KairoSpatial BVHTraversal](https://github.com/swayam8624/KairoSpatial/blob/main/BVHTraversal.cppm), [KairoRayTracer Scene](https://github.com/swayam8624/KairoRayTracer/blob/main/Scene.cppm) and [KairoPhysicsEngine Narrowphase](https://github.com/swayam8624/KairoPhysicsEngine/blob/main/Narrowphase.cppm).

## 1. Different domains: line, ray and segment

The parametric line equation is P(t)=O+tD, allowing every real t. A ray restricts t>=0; a segment additionally restricts t to a finite interval, often [0,1] when D=end−start. Mixing their domains silently accepts intersections behind a camera or beyond a finite sweep.

RayPlane uses plane equation n·x+d=0, so

\[
t=-\frac{n\cdot O+d}{n\cdot D}.
\]

Near-zero n·D is parallel or nearly parallel and rejected under the function epsilon. Negative t is behind the ray origin, even though the corresponding infinite line would intersect the plane.

**Example:** plane z=2, ray O=(0,0,0), D=(0,0,1): t=2. Reverse direction to (0,0,-1): t=-2 and the *ray* must miss.

## 2. Three ray slab intervals form one box hit

An AABB stores independent intervals [minX,maxX], [minY,maxY], [minZ]. RayAABB tests entry/exit t intervals for each axis and intersects those intervals into a common [tMin,tMax], beginning at tMin=0 for the forward ray.

For each nonparallel axis:

\[
t_1=(\min-O_i)/D_i,\qquad
t_2=(\max-O_i)/D_i.
\]

If the ray direction is negative, swap near/far. If D_i is nearly zero, the origin must already lie within that coordinate slab or the ray cannot reach the box.

**Worked example:** box [-1,1]^3, ray origin (-3,0,0), direction (1,0,0). X enters at t=2 and exits at t=4; Y/Z are parallel but their origins are inside the slabs. The ray hits. Change origin to (-3,2,0): the Y slab fails.

The box only proves intersection with the *axis-aligned volume*. A triangle enclosed by that box might still be missed by the same ray.

## 3. Exact triangle intersection adds barycentric constraints

RayTriangle uses Moller–Trumbore: edges AB and AC, a cross product with ray direction and one determinant. Its scalar triple products recover t, u and v in one consistent solution.

A valid hit requires t>=0 and barycentric weights u>=0, v>=0, u+v<=1. The third weight is w=1−u−v, and the hit point is wA+uB+vC. A determinant close to zero is rejected because nearly parallel or degenerate triangles amplify floating-point error.

The function uses absolute determinant to permit either triangle winding, instead of implicitly back-face culling. This is a two-sided triangle query policy; another renderer may deliberately choose one-sided culling.

**Worked example:** triangle A=(0,0,0), B=(1,0,0), C=(0,1,0), ray origin=(0.25,0.25,1) and direction=(0,0,-1). The hit is (0.25,0.25,0), t=1, (w,u,v)=(0.5,0.25,0.25).

Move origin to (0.9,0.9,1) while keeping the same direction. It still intersects the triangle **AABB** [0,1]×[0,1]×{0}, but u+v=1.8>1, so the actual triangle is missed. This is the critical distinction between broad-phase pruning and exact shape intersection.

## 4. Closest points and fat AABBs

AABB::ClosestPoint clamps a query position independently to each axis interval. The squared distance to that point is useful for a sphere-box test; it avoids a square root unless the caller needs actual distance.

AABB::ContainsAABB checks the two min/max opposite corners of another valid AABB because those extremal coordinates determine the entire axis-aligned volume. Broad-phase caches often enlarge the current tight box to a *fat envelope* and reuse that envelope while it still contains the current box.

This can be safe for candidate coverage yet expensive: fat envelopes include space beyond the actual shape and create false-positive pairs. To improve efficiency, KairoSpatial can rebuild a proxy when containment fails. KairoPhysicsEngine and KairoRayTracer still need exact geometry tests after acceleration candidates.

**Exercise:** take a tight sphere box of radius 0.5, expand by 0.1 and move the sphere center by 0.05 versus 0.15. Check containment and discuss when a dynamic-tree update must occur. Do not infer actual sphere-sphere contact solely from box overlap.

## 5. Floating-point invariants are not universal

Intersection.cppm uses operation-specific epsilons for determinant, nearly parallel axes and other cases. A fixed epsilon can be too permissive at tiny scales or too strict at huge scales; treating it as a universal robust-predicate guarantee is incorrect.

A ray tangent to a sphere or grazing a triangle edge lies near an ill-conditioned boundary. Test at multiple scales, include nonfinite input behavior where documented, and check whether endpoints/box faces are included. Geometry predicate success is a separate contract from physics solver stability.

## 6. Verify the actual consumer chain

1. Geometry supplies AABB and ray/shape predicates.
2. KairoSpatial BVH culls nodes with ray-box intersection and delegates exact hit to callback.
3. KairoRayTracer Scene intersects the winning primitive again for material, UV and normal data.
4. KairoPhysicsEngine narrowphase uses geometry routines to construct contact witnesses before impulses.

A BVH node's bounding-box hit is not proof that a triangle was hit. A triangle ray hit is not a contact impulse. A contact manifold is not proof that continuous-time motion is intersection-free.

Start with [GeometryTests.cpp](../tests/GeometryTests.cpp) and [CertificationTests.cpp](../tests/CertificationTests.cpp). Add ray-plane behind-origin, parallel slab, triangle AABB false positive, back-side triangle and degenerate area cases. Cross-check accelerated query results against brute-force exact geometry.

*Teaching derivation from source. No compiler, renderer or physics runtime was executed during this documentation pass.*
