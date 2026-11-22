# Geometry laboratory: closest witnesses, ray intersections and separating axes

Read [ClosestPoint.cppm](../ClosestPoint.cppm), [Intersection.cppm](../Intersection.cppm), [OBB.cppm](../OBB.cppm), [AABB.cppm](../AABB.cppm) and [Triangle.cppm](../Triangle.cppm) alongside this lesson. KairoSpatial accelerates *which primitives to inspect*; KairoGeometry decides whether a primitive actually contains, touches or intersects another. KairoPhysicsEngine then turns some of those results into contact manifolds and impulses.

## 1. Closest point on a finite segment

For endpoints A and B and query point P, parameterize the segment as

\[
Q(t)=A+t(B-A),\qquad 0\leq t\leq1.
\]

The unconstrained minimizer of squared distance is

\[
t^*=\frac{(P-A)\cdot(B-A)}{\|B-A\|^2}.
\]

Clamp to [0,1], giving Q. If A and B coincide, the denominator is zero and the correct closest point is A; ClosestPointOnSegment returns it as a degeneracy fallback.

**Worked example:** A=(0,0,0), B=(2,0,0), P=(3,1,0). The unclamped t is 1.5, so the finite segment's closest witness is B=(2,0,0), and squared distance is \(1^2+1^2=2\). Treating the segment as an infinite line would instead give (3,0,0), which is wrong for a capsule endpoint.

**Exercise:** test projections inside the segment, beyond both endpoints and onto a zero-length segment. KairoPhysicsEngine's capsule/box and capsule/sphere routines depend on correct finite segment witnesses.

## 2. Closest point on a triangle: seven regions

The finite triangle ABC divides the surrounding space into three vertex Voronoi regions, three edge regions and one face region. ClosestPointOnTriangle tests dot products of AP, BP, CP with AB and AC to identify which region contains the projected query.

If the projection is inside the face, compute barycentric weights and return

\[
Q=A+v(B-A)+w(C-A),\quad v\geq0,\ w\geq0,\ v+w\leq1.
\]

For a point in an edge region, return the clamped segment witness. For a point beyond a corner, return that vertex. If triangle area becomes numerically degenerate, the source compares all three finite edge witnesses and returns the closest, avoiding division by an almost-zero barycentric denominator.

**Worked example:** A=(0,0,0), B=(1,0,0), C=(0,1,0), P=(0.2,0.3,1). The vertical projection lies in the face, so Q=(0.2,0.3,0) and distance is 1. If P=(2,0,0), the closest is vertex B=(1,0,0).

**Exercise:** construct a nearly collinear triangle and compare with a brute-force sample of its three edges. Document numerical tolerance and expected fallback.

## 3. Ray versus triangle: solve for t and barycentrics

RayTriangle uses the Möller–Trumbore style determinant calculation to solve

\[
O+tD=A+u(B-A)+v(C-A).
\]

Here t≥0 is ray distance along direction D, and u,v satisfy u≥0, v≥0, u+v≤1 for the triangle interior. A determinant near zero signals a parallel ray or degenerate triangle; source code rejects it using its epsilon parameter. It uses the absolute determinant, so there is no automatic back-face culling.

**Worked example:** ray origin (0.25,0.25,1), direction (0,0,−1), and triangle in z=0 with A=(0,0,0), B=(1,0,0), C=(0,1,0). Intersection is at t=1, u=v=0.25, world position (0.25,0.25,0). Change origin to (2,2,1) and the plane intersection remains but lies outside the triangle.

**Exercise:** test both triangle winding directions, a parallel ray, a ray starting behind the triangle and a hit exactly on an edge. Choose epsilon based on numerical scale, not arbitrary visual tolerance.

## 4. Ray versus AABB: three intersecting slabs

Each coordinate axis of an axis-aligned box contributes an allowed interval of ray parameters. For direction component \(D_i\neq0\),

\[
t_{i,1}=\frac{min_i-O_i}{D_i},\qquad
t_{i,2}=\frac{max_i-O_i}{D_i}.
\]

Swap the two times for negative direction, then intersect all three intervals with the ray's t≥0 interval. If any new lower bound exceeds the upper bound, there is no ray-box hit.

For nearly zero D_i, the ray is parallel to that slab. It can intersect only if its origin coordinate is already within [min_i,max_i].

**Worked example:** AABB x,y,z∈[−1,1], O=(−3,0,0), D=(1,0,0). x slab gives t=2 to 4; y/z parallel slab checks pass, so first hit is t=2 at (−1,0,0). Change origin y to 2 and the ray misses, even though it is pointed along +x.

The dynamic AABB tree uses this only for candidate pruning. A ray can hit a sphere's enclosing box while missing the curved sphere itself; only a primitive ray query gives the final surface result.

## 5. Oriented box versus oriented box: 15 axes

An OBB has center, three orthonormal local axes and half extents. Two arbitrary OBBs can be separated by a plane normal aligned with either box's three face axes, or one of nine cross products of pairs of edges. The SAT test checks at most **15 candidate axes**.

Intersection.cppm constructs a relative orientation matrix

\[
R_{ij}=a_i\cdot b_j,
\]

where a_i and b_j are the world-space box axes. It projects center separation and each box's half extents onto the candidate axes. If projected centers are farther apart than the sum of radii for any one axis, the boxes do not overlap. A small epsilon is added to absolute orientation values to reduce instability near parallel edge pairs.

**Exercise:** first test two axis-aligned 1×1×1 half-extents at centers (0,0,0) and (2.1,0,0): x separates them. Move second center to (1.5,0,0): axes overlap. Then rotate one box about Y and show why checking only six face normals can miss an edge-edge separation.

A positive SAT result indicates overlap, not a complete collision manifold with positions and stable contact features. KairoPhysicsEngine's Narrowphase constructs and reduces those witnesses separately.

## 6. Triangle versus triangle and special cases

TriangleTriangle first compares each triangle's vertices to the other triangle's plane; three vertices strictly on the same exterior side allow early rejection. If the two triangles are coplanar, the code dispatches to a dedicated planar intersection helper. Otherwise, it tests finite edges of both triangles against the opposite triangle.

A bounding-box test alone is insufficient: two AABBs may overlap even if the finite triangle surfaces do not. Numerically coplanar and degenerate triangles need explicit regression cases, especially with coordinates spanning very large or tiny magnitudes.

## 7. Ownership and the call graph

The math layer owns vectors, dot/cross products, and transformations. The geometry layer owns closest points and exact primitive overlap predicates. Spatial BVH/dynamic tree owns candidate lookup, not shape surface contact. PhysicsEngine owns dynamic collider IDs and contact response. RayTracer uses ray/geometry intersections to choose optical surface hits rather than impulses.

A correct integration trace is:

- a ray goes through the spatial hierarchy;
- the hierarchy returns conservative candidate primitive IDs;
- geometry performs actual RayTriangle/RayAABB/shape-specific tests;
- the caller chooses the appropriate nearest hit or contact manifold;
- physics/renderer handles the hit according to its domain.

## 8. Tests and reproducibility

Read [SOURCE_READING_ORDER.md](SOURCE_READING_ORDER.md) for each module's test files. Add cases for degenerate/parallel/ray-behind triangle, triangle permutation symmetry, nearly coplanar triangles, zero-length segments, OBB SAT edge-edge separation and AABB slab parallelism. Validate numerical results against an independent high-precision or brute-force oracle and record exact compiler/source SHA, floating-point type and epsilon policy.

*Source-based teaching artifact; no new C++ build, property-based test or geometry benchmark is claimed.*
