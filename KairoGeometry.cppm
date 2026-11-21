// TEACHING NOTE:
// Umbrella exports value-type shapes, distance and intersection APIs built on KairoMath.Vector; it
// does not own a spatial tree, physics body or renderer.
// See docs/SOURCE_READING_ORDER.md for related modules and tests.

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
