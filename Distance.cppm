module;

#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>

export module Kairo.Foundation.Geometry.Distance;

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
import Kairo.Foundation.Geometry.ClosestPoint;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Distance - Point / Ray
    //=========================================================

    /// Input: function parameters as declared by `DistanceSquaredToRay`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredToRay` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquaredToRay(
        const Vector3<T>& point,
        const Ray<T>& ray) noexcept
    {
        return kairo::foundation::math::DistanceSquared(
            point,
            ClosestPointOnRay(
                ray,
                point));
    }

    /// Input: function parameters as declared by `DistanceToRay`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceToRay` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceToRay(
        const Vector3<T>& point,
        const Ray<T>& ray) noexcept
    {
        return std::sqrt(
            DistanceSquaredToRay(
                point,
                ray));
    }

    //=========================================================
    // Distance - Point / Plane
    //=========================================================

    /// Input: function parameters as declared by `SignedDistanceToPlane`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SignedDistanceToPlane` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T SignedDistanceToPlane(
        const Vector3<T>& point,
        const Plane<T>& plane) noexcept
    {
        return plane.SignedDistanceToPoint(
            point);
    }

    /// Input: function parameters as declared by `DistanceToPlane`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceToPlane` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceToPlane(
        const Vector3<T>& point,
        const Plane<T>& plane) noexcept
    {
        return std::abs(
            SignedDistanceToPlane(
                point,
                plane));
    }

    //=========================================================
    // Distance - Point / Sphere
    //=========================================================

    /// Input: function parameters as declared by `SignedDistanceToSphere`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SignedDistanceToSphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T SignedDistanceToSphere(
        const Vector3<T>& point,
        const Sphere<T>& sphere) noexcept
    {
        return
            kairo::foundation::math::Distance(
                point,
                sphere.Center)
            - sphere.Radius;
    }

    /// Input: function parameters as declared by `DistanceToSphere`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceToSphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceToSphere(
        const Vector3<T>& point,
        const Sphere<T>& sphere) noexcept
    {
        return Max(
            SignedDistanceToSphere(
                point,
                sphere),
            T(0));
    }

    //=========================================================
    // Distance - Point / Triangle
    //=========================================================

    /// Input: function parameters as declared by `DistanceSquaredToTriangle`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredToTriangle` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquaredToTriangle(
        const Vector3<T>& point,
        const Triangle<T>& triangle) noexcept
    {
        return kairo::foundation::math::DistanceSquared(
            point,
            ClosestPointOnTriangle(
                triangle,
                point));
    }

    /// Input: function parameters as declared by `DistanceToTriangle`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceToTriangle` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceToTriangle(
        const Vector3<T>& point,
        const Triangle<T>& triangle) noexcept
    {
        return std::sqrt(
            DistanceSquaredToTriangle(
                point,
                triangle));
    }

    //=========================================================
    // Distance - Point / AABB
    //=========================================================

    /// Input: function parameters as declared by `DistanceSquaredToAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredToAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T DistanceSquaredToAABB(
        const Vector3<T>& point,
        const AABB<T>& box) noexcept
    {
        return kairo::foundation::math::DistanceSquared(
            point,
            ClosestPointOnAABB(
                box,
                point));
    }

    /// Input: function parameters as declared by `DistanceToAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceToAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceToAABB(
        const Vector3<T>& point,
        const AABB<T>& box) noexcept
    {
        return std::sqrt(
            DistanceSquaredToAABB(
                point,
                box));
    }

    //=========================================================
    // Distance - Point / Capsule
    //=========================================================

    /// Input: function parameters as declared by `SignedDistanceToCapsule`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SignedDistanceToCapsule` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T SignedDistanceToCapsule(
        const Vector3<T>& point,
        const Capsule<T>& capsule) noexcept
    {
        return capsule.SignedDistanceToPoint(point);
    }

    /// Input: function parameters as declared by `DistanceSquaredToCapsule`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredToCapsule` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquaredToCapsule(
        const Vector3<T>& point,
        const Capsule<T>& capsule) noexcept
    {
        return capsule.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `DistanceToCapsule`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceToCapsule` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceToCapsule(
        const Vector3<T>& point,
        const Capsule<T>& capsule) noexcept
    {
        return capsule.DistanceToPoint(point);
    }

    //=========================================================
    // Distance - Point / OBB
    //=========================================================

    /// Input: function parameters as declared by `DistanceSquaredToOBB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredToOBB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T DistanceSquaredToOBB(
        const Vector3<T>& point,
        const OBB<T>& box) noexcept
    {
        return box.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `DistanceToOBB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceToOBB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceToOBB(
        const Vector3<T>& point,
        const OBB<T>& box) noexcept
    {
        return box.DistanceToPoint(point);
    }

    //=========================================================
    // Distance - Point / Segment
    //=========================================================

    /// Input: function parameters as declared by `DistanceSquaredToSegment`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredToSegment` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquaredToSegment(
        const Vector3<T>& point,
        const Segment<T>& segment) noexcept
    {
        return segment.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `DistanceToSegment`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceToSegment` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceToSegment(
        const Vector3<T>& point,
        const Segment<T>& segment) noexcept
    {
        return segment.DistanceToPoint(point);
    }

    //=========================================================
    // Distance - Point / Line
    //=========================================================

    /// Input: function parameters as declared by `DistanceSquaredToLine`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredToLine` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquaredToLine(
        const Vector3<T>& point,
        const Line<T>& line) noexcept
    {
        return line.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `DistanceToLine`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceToLine` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceToLine(
        const Vector3<T>& point,
        const Line<T>& line) noexcept
    {
        return line.DistanceToPoint(point);
    }

    //=========================================================
    // Distance - Ray / Ray
    //
    // Computes shortest distance between two half-infinite rays.
    // This is useful for picking, debug tools, and later geometry
    // queries. If rays are nearly parallel, it falls back to endpoint
    // projection.
    //=========================================================

    /// Input: function parameters as declared by `DistanceSquaredBetweenRays`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredBetweenRays` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquaredBetweenRays(
        const Ray<T>& lhs,
        const Ray<T>& rhs) noexcept
    {
        const Vector3<T> p =
            lhs.Origin;

        const Vector3<T> q =
            rhs.Origin;

        const Vector3<T> d1 =
            lhs.Direction;

        const Vector3<T> d2 =
            rhs.Direction;

        const Vector3<T> r =
            p - q;

        const T a =
            Dot(d1, d1);

        const T e =
            Dot(d2, d2);

        const T f =
            Dot(d2, r);

        const T epsilon =
            std::numeric_limits<T>::epsilon()
            * T(10);

        T s = T(0);
        T t = T(0);

        if (a <= epsilon &&
            e <= epsilon)
        {
            return kairo::foundation::math::DistanceSquared(
                p,
                q);
        }

        if (a <= epsilon)
        {
            s = T(0);
            t = Max(
                f / e,
                T(0));
        }
        else
        {
            const T c =
                Dot(d1, r);

            if (e <= epsilon)
            {
                t = T(0);
                s = Max(
                    -c / a,
                    T(0));
            }
            else
            {
                const T b =
                    Dot(d1, d2);

                const T denominator =
                    a * e -
                    b * b;

                if (std::abs(denominator) > epsilon)
                {
                    s =
                        Max(
                            (b * f - c * e) / denominator,
                            T(0));
                }
                else
                {
                    s = T(0);
                }

                t =
                    (b * s + f) / e;

                if (t < T(0))
                {
                    t = T(0);
                    s =
                        Max(
                            -c / a,
                            T(0));
                }
            }
        }

        const Vector3<T> closestOnLhs =
            p + d1 * s;

        const Vector3<T> closestOnRhs =
            q + d2 * t;

        return kairo::foundation::math::DistanceSquared(
            closestOnLhs,
            closestOnRhs);
    }

    /// Input: function parameters as declared by `DistanceBetweenRays`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceBetweenRays` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceBetweenRays(
        const Ray<T>& lhs,
        const Ray<T>& rhs) noexcept
    {
        return std::sqrt(
            DistanceSquaredBetweenRays(
                lhs,
                rhs));
    }

    //=========================================================
    // Distance - Sphere / Sphere
    //=========================================================

    /// Input: function parameters as declared by `SignedDistanceBetweenSpheres`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SignedDistanceBetweenSpheres` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T SignedDistanceBetweenSpheres(
        const Sphere<T>& lhs,
        const Sphere<T>& rhs) noexcept
    {
        return
            kairo::foundation::math::Distance(
                lhs.Center,
                rhs.Center)
            - lhs.Radius
            - rhs.Radius;
    }

    /// Input: function parameters as declared by `DistanceBetweenSpheres`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceBetweenSpheres` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceBetweenSpheres(
        const Sphere<T>& lhs,
        const Sphere<T>& rhs) noexcept
    {
        return Max(
            SignedDistanceBetweenSpheres(
                lhs,
                rhs),
            T(0));
    }

    //=========================================================
    // Distance - AABB / AABB
    //=========================================================

    /// Input: function parameters as declared by `DistanceSquaredBetweenAABBs`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredBetweenAABBs` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T DistanceSquaredBetweenAABBs(
        const AABB<T>& lhs,
        const AABB<T>& rhs) noexcept
    {
        T distanceSquared =
            T(0);

        if (lhs.Max.x < rhs.Min.x)
        {
            const T delta =
                rhs.Min.x - lhs.Max.x;

            distanceSquared +=
                delta * delta;
        }
        else if (rhs.Max.x < lhs.Min.x)
        {
            const T delta =
                lhs.Min.x - rhs.Max.x;

            distanceSquared +=
                delta * delta;
        }

        if (lhs.Max.y < rhs.Min.y)
        {
            const T delta =
                rhs.Min.y - lhs.Max.y;

            distanceSquared +=
                delta * delta;
        }
        else if (rhs.Max.y < lhs.Min.y)
        {
            const T delta =
                lhs.Min.y - rhs.Max.y;

            distanceSquared +=
                delta * delta;
        }

        if (lhs.Max.z < rhs.Min.z)
        {
            const T delta =
                rhs.Min.z - lhs.Max.z;

            distanceSquared +=
                delta * delta;
        }
        else if (rhs.Max.z < lhs.Min.z)
        {
            const T delta =
                lhs.Min.z - rhs.Max.z;

            distanceSquared +=
                delta * delta;
        }

        return distanceSquared;
    }

    /// Input: function parameters as declared by `DistanceBetweenAABBs`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceBetweenAABBs` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceBetweenAABBs(
        const AABB<T>& lhs,
        const AABB<T>& rhs) noexcept
    {
        return std::sqrt(
            DistanceSquaredBetweenAABBs(
                lhs,
                rhs));
    }

    //=========================================================
    // Distance - Segment / Segment
    //=========================================================

    /// Input: function parameters as declared by `DistanceSquaredBetweenSegments`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquaredBetweenSegments` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquaredBetweenSegments(
        const Segment<T>& lhs,
        const Segment<T>& rhs) noexcept
    {
        return ClosestPoints(lhs, rhs).DistanceSquared;
    }

    /// Input: function parameters as declared by `DistanceBetweenSegments`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceBetweenSegments` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceBetweenSegments(
        const Segment<T>& lhs,
        const Segment<T>& rhs) noexcept
    {
        return std::sqrt(
            DistanceSquaredBetweenSegments(lhs, rhs));
    }

} // namespace kairo::foundation::geometry
