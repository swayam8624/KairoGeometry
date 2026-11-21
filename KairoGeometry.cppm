// TEACHING MAP — KairoGeometry.cppm
// Public C++23 geometry umbrella. Re-exports primitive, intersection, distance and frustum
// contracts; KairoSpatial, PhysicsEngine, Renderer and RayTracer import these types.
// See READING_ORDER.md for public math APIs, caller modules and test coverage.

export module Kairo.Foundation.Geometry;

export import Kairo.Foundation.Geometry.Ray;
export import Kairo.Foundation.Geometry.Line;
export import Kairo.Foundation.Geometry.Segment;
export import Kairo.Foundation.Geometry.Plane;
export import Kairo.Foundation.Geometry.Sphere;
export import Kairo.Foundation.Geometry.Triangle;
export import Kairo.Foundation.Geometry.AABB;
export import Kairo.Foundation.Geometry.Capsule;
export import Kairo.Foundation.Geometry.OBB;
export import Kairo.Foundation.Geometry.Frustum;
export import Kairo.Foundation.Geometry.ClosestPoint;
export import Kairo.Foundation.Geometry.Distance;
export import Kairo.Foundation.Geometry.Intersection;
