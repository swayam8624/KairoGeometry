// TEACHING MODULE: Umbrella C++23 module that reexports primitive, closest-point, distance
// and intersection interfaces. Narrow imports reduce compile coupling;
// there is no hidden geometry implementation in this file.
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
