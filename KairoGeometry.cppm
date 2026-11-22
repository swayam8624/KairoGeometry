// TEACHING MODULE: Public aggregation layer for the geometric contracts used
// by KairoSpatial, KairoRayTracer and physics collision/narrowphase code.
// It re-exports declarations; it does not calculate intersections here.
//
// Dependency boundary: KairoMath supplies vectors, transforms and scalar math.
// KairoGeometry supplies geometric *meaning* (rays, triangles, volumes) and
// queries (closest point, separation/distance and intersection). Spatial owns
// indexing/acceleration, while Physics owns state integration and contact
// resolution. Those responsibilities must not be conflated.
//
// In C++23, `export import M;` republishes M's exported declarations for
// callers of `import Kairo.Foundation.Geometry;`. A narrow import such as
// `import Kairo.Foundation.Geometry.Ray;` avoids depending on the complete
// umbrella. Build rules must still account for each named module interface.
//
// Study path: start with Ray, Line and Segment (infinite versus finite
// parameter domains); then Plane/Triangle and bounded primitives; finally
// ClosestPoint, Distance and Intersection. For each test, record degenerate
// shapes, tolerance policy and whether outputs are points, distances or hits.
// See docs/SOURCE_READING_ORDER.md and tests/GeometryTests.cpp.
export module Kairo.Foundation.Geometry;

// Parametric primitives: rays have a direction and a half-line domain;
// lines extend in both directions; segments have two bounded endpoints.
// Query algorithms must not interchange their parameter ranges.
export import Kairo.Foundation.Geometry.Ray;
export import Kairo.Foundation.Geometry.Line;
export import Kairo.Foundation.Geometry.Segment;

// Surfaces and volumes: normals/winding influence sidedness; bounding
// volumes support broadphase and visibility but are not contact solvers.
export import Kairo.Foundation.Geometry.Plane;
export import Kairo.Foundation.Geometry.Sphere;
export import Kairo.Foundation.Geometry.Triangle;
export import Kairo.Foundation.Geometry.AABB;
export import Kairo.Foundation.Geometry.Capsule;
export import Kairo.Foundation.Geometry.OBB;
export import Kairo.Foundation.Geometry.Frustum;

// Separate three kinds of query: nearest location, scalar separation, and
// overlap/hit classification. Verify the actual endpoint/tangency conventions
// in each function and its tests before composing queries downstream.
export import Kairo.Foundation.Geometry.ClosestPoint;
export import Kairo.Foundation.Geometry.Distance;
export import Kairo.Foundation.Geometry.Intersection;
