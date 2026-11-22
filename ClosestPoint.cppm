// TEACHING MODULE: Shape-specific closest point queries reduce many collision/distance tests
// to a single geometric witness. Ray clamps to t>=0; Segment clamps to [0,1];
// Plane projects along normal. Degenerate triangles and zero-length axes need
// guarded fallback. Higher-level physics code consumes returned points.
module;

#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>

export module Kairo.Foundation.Geometry.ClosestPoint;

import Kairo.Foundation.Math.Vector;

import Kairo.Foundation.Geometry.Ray;
import Kairo.Foundation.Geometry.Line;
import Kairo.Foundation.Geometry.Segment;
import Kairo.Foundation.Geometry.Plane;
import Kairo.Foundation.Geometry.Sphere;
import Kairo.Foundation.Geometry.Triangle;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Geometry.Capsule;
import Kairo.Foundation.Geometry.OBB;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Closest Point - Point / Ray
    //=========================================================

    /// Input: ray and point.
    /// Output: closest point on the ray, clamped so the result is never behind
    /// the ray origin.
    /// Task: provide a readable named wrapper around Ray's point projection
    /// helper for aggregate query code.
    template<FloatingPoint T>
    [[nodiscard]]
    // Ray parameter is constrained to t >= 0, unlike an infinite
    // line. This wrapper delegates to Ray's own closest-point
    // semantics; downstream collision code expects the finite
    // start of the ray to be respected.
    Vector3<T> ClosestPointOnRay(
        const Ray<T>& ray,
        const Vector3<T>& point) noexcept
    {
        return ClosestPoint(
            ray,
            point);
    }

    //=========================================================
    // Closest Point - Point / Plane
    //=========================================================

    /// Input: plane and point.
    /// Output: orthogonal projection of the point onto the plane.
    /// Task: expose point/plane projection using the shared closest-point
    /// naming convention.
    template<FloatingPoint T>
    [[nodiscard]]
    // Orthogonally project onto the infinite plane using the plane's
    // signed distance/normal convention. An unbounded plane has no
    // edge clamping unlike a triangle or finite segment.
    constexpr Vector3<T> ClosestPointOnPlane(
        const Plane<T>& plane,
        const Vector3<T>& point) noexcept
    {
        return plane.ClosestPoint(
            point);
    }

    //=========================================================
    // Closest Point - Point / Sphere
    //=========================================================

    /// Input: sphere and point.
    /// Output: closest point on the sphere surface.
    /// Task: route callers through Sphere's robust closest-point behavior,
    /// including the center-point fallback direction.
    template<FloatingPoint T>
    [[nodiscard]]
    // A sphere witness lies on its curved surface, not merely on
    // an enclosing AABB. Use Sphere::ClosestPoint's own policy for
    // inside/center degeneracy rather than inventing a new one here.
    Vector3<T> ClosestPointOnSphere(
        const Sphere<T>& sphere,
        const Vector3<T>& point) noexcept
    {
        return sphere.ClosestPoint(
            point);
    }

    //=========================================================
    // Closest Point - Point / AABB
    //=========================================================

    /// Input: axis-aligned box and point.
    /// Output: point clamped to the box volume.
    /// Task: provide the standard point/AABB query used by distance and
    /// intersection helpers.
    template<FloatingPoint T>
    [[nodiscard]]
    // Clamp each coordinate independently to axis-aligned min/max
    // extents. This remains a geometric nearest point, not a shape
    // penetration normal or a physics impulse.
    constexpr Vector3<T> ClosestPointOnAABB(
        const AABB<T>& box,
        const Vector3<T>& point) noexcept
    {
        return box.ClosestPoint(
            point);
    }

    //=========================================================
    // Closest Point - Point / Capsule
    //=========================================================

    /// Input: capsule and point.
    /// Output: closest point on the capsule surface.
    /// Task: expose the capsule member query through the shared closest-point
    /// naming scheme used by the aggregate geometry query module.
    template<FloatingPoint T>
    [[nodiscard]]
    /// TEACHING API: ClosestPointOnCapsule — Find closest point on capsule axis then offset toward the query by capsule radius.
    // The capsule is a swept sphere along a segment. Its closest
    // surface witness depends on both the closest axis point and
    // radius; delegate this detail to the Capsule type to preserve
    // one consistent degeneracy policy across engine consumers.
    Vector3<T> ClosestPointOnCapsule(
        const Capsule<T>& capsule,
        const Vector3<T>& point) noexcept
    {
        return capsule.ClosestPoint(point);
    }

    //=========================================================
    // Closest Point - Point / OBB
    //=========================================================

    /// Input: oriented box and point.
    /// Output: closest point on or inside the oriented box.
    /// Task: clamp in OBB local space while keeping callers independent from
    /// the box's internal world/local conversion details.
    template<FloatingPoint T>
    [[nodiscard]]
    /// TEACHING API: ClosestPointOnOBB — Transform query into OBB local coordinates, clamp to a local AABB, then transform back.
    // OBB closest-point logic operates in its oriented basis:
    // project to local axes, clamp by half-extents, then transform
    // the witness back. Keep the normalized-axes precondition.
    constexpr Vector3<T> ClosestPointOnOBB(
        const OBB<T>& box,
        const Vector3<T>& point) noexcept
    {
        return box.ClosestPoint(point);
    }

    //=========================================================
    // Closest Point - Point / Segment
    //=========================================================

    /// Input: segment and point.
    /// Output: closest point on the finite segment.
    /// Task: named query helper for point/segment distance and capsule-backed
    /// algorithms.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPointOnSegment(
        const Segment<T>& segment,
        const Vector3<T>& point) noexcept
    {
        return segment.ClosestPoint(point);
    }

    //=========================================================
    // Closest Point - Point / Line
    //=========================================================

    /// Input: infinite line and point.
    /// Output: closest point on the infinite line.
    /// Task: named query helper for analytic geometry and debug tools.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPointOnLine(
        const Line<T>& line,
        const Vector3<T>& point) noexcept
    {
        return line.ClosestPoint(point);
    }

    //=========================================================
    // Closest Point - Point / Segment
    // Internal helper for triangle closest-point logic.
    //=========================================================

    /// Input: segment endpoints and point.
    /// Output: closest point on the finite segment from start to end.
    /// Task: support triangle Voronoi fallback logic without requiring the
    /// Segment module. Degenerate segments return start.
    template<FloatingPoint T>
    [[nodiscard]]
    // Parameterize finite segment p(t)=start+t*(end-start),
    // minimize squared distance via dot/length^2, and clamp t
    // to [0,1]. A zero-length segment falls back to its start.
    // KairoPhysicsEngine uses this for capsule/segment witnesses.
    Vector3<T> ClosestPointOnSegment(
        const Vector3<T>& start,
        const Vector3<T>& end,
        const Vector3<T>& point) noexcept
    {
        const Vector3<T> segment =
            end - start;

        const T lengthSquared =
            segment.LengthSquared();

        if (lengthSquared <=
            std::numeric_limits<T>::epsilon())
        {
            return start;
        }

        const T t =
            Clamp(
                Dot(point - start, segment) / lengthSquared,
                T(0),
                T(1));

        return
            start +
            segment * t;
    }

    //=========================================================
    // Closest Point - Point / Triangle
    //
    // Based on the Voronoi-region method from Real-Time
    // Collision Detection. This is robust and avoids projecting
    // first and then separately repairing edge cases.
    //=========================================================

    /// Input: triangle and point.
    /// Output: closest point on the triangle surface, edge, or vertex.
    /// Task: compute robust point/triangle closest points using Voronoi regions
    /// so degenerate and edge-region cases are handled explicitly.
    template<FloatingPoint T>
    [[nodiscard]]
    /// TEACHING API: ClosestPointOnTriangle — Partition space into triangle vertex, edge and face Voronoi regions; find the correct closest geometric witness.
    // Partition 3D point space into the triangle's seven Voronoi
    // regions: three vertex, three edge, one face. Dot products
    // decide the region without first projecting outside triangle.
    // A nearly degenerate face falls back to the closest of its
    // three finite edges. This is useful for sphere/triangle contacts.
    Vector3<T> ClosestPointOnTriangle(
        const Triangle<T>& triangle,
        const Vector3<T>& point) noexcept
    {
        const Vector3<T>& a =
            triangle.A;

        const Vector3<T>& b =
            triangle.B;

        const Vector3<T>& c =
            triangle.C;

        const Vector3<T> ab =
            b - a;

        const Vector3<T> ac =
            c - a;

        const Vector3<T> ap =
            point - a;

        // Signs of d1 and d2 determine whether the query projects
        // beyond vertex A along both outgoing triangle directions.
        // Similar tests for B and C cover the other vertex regions.
        const T d1 =
            Dot(ab, ap);

        const T d2 =
            Dot(ac, ap);

        if (d1 <= T(0) &&
            d2 <= T(0))
        {
            return a;
        }

        const Vector3<T> bp =
            point - b;

        const T d3 =
            Dot(ab, bp);

        const T d4 =
            Dot(ac, bp);

        if (d3 >= T(0) &&
            d4 <= d3)
        {
            return b;
        }

        // Barycentric area expressions identify the AB edge region.
        // If v_c <= 0 with the endpoint inequalities satisfied,
        // return the clamped witness on AB rather than face projection.
        const T vc =
            d1 * d4 -
            d3 * d2;

        if (vc <= T(0) &&
            d1 >= T(0) &&
            d3 <= T(0))
        {
            const T v =
                d1 /
                (d1 - d3);

            return
                a +
                ab * v;
        }

        const Vector3<T> cp =
            point - c;

        const T d5 =
            Dot(ab, cp);

        const T d6 =
            Dot(ac, cp);

        if (d6 >= T(0) &&
            d5 <= d6)
        {
            return c;
        }

        const T vb =
            d5 * d2 -
            d1 * d6;

        if (vb <= T(0) &&
            d2 >= T(0) &&
            d6 <= T(0))
        {
            const T w =
                d2 /
                (d2 - d6);

            return
                a +
                ac * w;
        }

        const T va =
            d3 * d6 -
            d5 * d4;

        if (va <= T(0) &&
            (d4 - d3) >= T(0) &&
            (d5 - d6) >= T(0))
        {
            const T w =
                (d4 - d3) /
                (
                    (d4 - d3) +
                    (d5 - d6)
                );

            return
                b +
                (c - b) * w;
        }

        // When the three signed region-area terms sum near zero,
        // the triangle cannot provide a stable 2D face basis.
        // Compare all three edge witnesses instead of dividing by
        // a nearly zero barycentric denominator.
        const T denominator =
            va + vb + vc;

        if (std::abs(denominator) <=
            std::numeric_limits<T>::epsilon())
        {
            const Vector3<T> closestAB =
                ClosestPointOnSegment(
                    a,
                    b,
                    point);

            const Vector3<T> closestBC =
                ClosestPointOnSegment(
                    b,
                    c,
                    point);

            const Vector3<T> closestCA =
                ClosestPointOnSegment(
                    c,
                    a,
                    point);

            const T distanceAB =
                DistanceSquared(
                    point,
                    closestAB);

            const T distanceBC =
                DistanceSquared(
                    point,
                    closestBC);

            const T distanceCA =
                DistanceSquared(
                    point,
                    closestCA);

            if (distanceAB <= distanceBC &&
                distanceAB <= distanceCA)
            {
                return closestAB;
            }

            if (distanceBC <= distanceCA)
            {
                return closestBC;
            }

            return closestCA;
        }

        const T inverseDenominator =
            T(1) / denominator;

        const T v =
            vb * inverseDenominator;

        const T w =
            vc * inverseDenominator;

        return
            a +
            ab * v +
            ac * w;
    }

} // namespace kairo::foundation::geometry
