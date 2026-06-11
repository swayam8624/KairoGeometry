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
