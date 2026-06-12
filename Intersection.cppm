module;

#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <optional>
#include <type_traits>

export module Kairo.Foundation.Geometry.Intersection;

import Kairo.Foundation.Math.Vector;

import Kairo.Foundation.Geometry.Ray;
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
    // Ray Hit Result
    //=========================================================

    template<FloatingPoint T>
    struct RayHit final
    {
        T Distance =
            T(0);

        Vector3<T> Point =
            Vector3<T>::Zero();

        Vector3<T> Normal =
            Vector3<T>::Up();
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(
            const RayHit&) const noexcept = default;
    };

    using RayHitf =
        RayHit<float>;

    using RayHitd =
        RayHit<double>;

    //=========================================================
    // Ray / Plane
    //=========================================================

    /// Input: function parameters as declared by `RayPlane`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `RayPlane` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    std::optional<RayHit<T>> RayPlane(
        const Ray<T>& ray,
        const Plane<T>& plane,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        const T denominator =
            Dot(
                plane.Normal,
                ray.Direction);

        if (std::abs(denominator) <= epsilon)
        {
            return std::nullopt;
        }

        const T t =
            -(
                Dot(
                    plane.Normal,
                    ray.Origin)
                + plane.Distance
            )
            / denominator;

        if (t < T(0))
        {
            return std::nullopt;
        }

        const Vector3<T> point =
            ray.GetPoint(t);

        return RayHit<T>
        {
            t,
            point,
            plane.Normal
        };
    }

    //=========================================================
    // Ray / Sphere
    //=========================================================

    /// Input: function parameters as declared by `RaySphere`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `RaySphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    std::optional<RayHit<T>> RaySphere(
        const Ray<T>& ray,
        const Sphere<T>& sphere,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        const Vector3<T> originToCenter =
            ray.Origin - sphere.Center;

        const T a =
            Dot(
                ray.Direction,
                ray.Direction);

        const T b =
            T(2) *
            Dot(
                originToCenter,
                ray.Direction);

        const T c =
            Dot(
                originToCenter,
                originToCenter)
            - sphere.Radius * sphere.Radius;

        const T discriminant =
            b * b -
            T(4) * a * c;

        if (discriminant < T(0))
        {
            return std::nullopt;
        }

        const T sqrtDiscriminant =
            std::sqrt(discriminant);

        const T denominator =
            T(2) * a;

        if (std::abs(denominator) <= epsilon)
        {
            return std::nullopt;
        }

        const T t0 =
            (-b - sqrtDiscriminant) /
            denominator;

        const T t1 =
            (-b + sqrtDiscriminant) /
            denominator;

        T t =
            t0;

        if (t < T(0))
        {
            t = t1;
        }

        if (t < T(0))
        {
            return std::nullopt;
        }

        const Vector3<T> point =
            ray.GetPoint(t);

        const Vector3<T> normal =
            SafeNormalize(
                point - sphere.Center,
                Vector3<T>::Up());

        return RayHit<T>
        {
            t,
            point,
            normal
        };
    }

    //=========================================================
    // Ray / Triangle
    //
    // Moller-Trumbore intersection.
    // Returns front and back face hits.
    //=========================================================

    /// Input: function parameters as declared by `RayTriangle`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `RayTriangle` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    std::optional<RayHit<T>> RayTriangle(
        const Ray<T>& ray,
        const Triangle<T>& triangle,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        const Vector3<T> edgeAB =
            triangle.B - triangle.A;

        const Vector3<T> edgeAC =
            triangle.C - triangle.A;

        const Vector3<T> pvec =
            Cross(
                ray.Direction,
                edgeAC);

        const T determinant =
            Dot(
                edgeAB,
                pvec);

        if (std::abs(determinant) <= epsilon)
        {
            return std::nullopt;
        }

        const T inverseDeterminant =
            T(1) / determinant;

        const Vector3<T> tvec =
            ray.Origin - triangle.A;

        const T u =
            Dot(
                tvec,
                pvec)
            * inverseDeterminant;

        if (u < T(0) ||
            u > T(1))
        {
            return std::nullopt;
        }

        const Vector3<T> qvec =
            Cross(
                tvec,
                edgeAB);

        const T v =
            Dot(
                ray.Direction,
                qvec)
            * inverseDeterminant;

        if (v < T(0) ||
            u + v > T(1))
        {
            return std::nullopt;
        }

        const T t =
            Dot(
                edgeAC,
                qvec)
            * inverseDeterminant;

        if (t < T(0))
        {
            return std::nullopt;
        }

        const Vector3<T> point =
            ray.GetPoint(t);

        Vector3<T> normal =
            triangle.Normal();

        if (Dot(normal, ray.Direction) > T(0))
        {
            normal = -normal;
        }

        return RayHit<T>
        {
            t,
            point,
            normal
        };
    }

    //=========================================================
    // Ray / AABB
    //
    // Slab method.
    //=========================================================

    /// Input: function parameters as declared by `RayAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `RayAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    std::optional<RayHit<T>> RayAABB(
        const Ray<T>& ray,
        const AABB<T>& box,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        if (!box.IsValid())
        {
            return std::nullopt;
        }

        T tMin =
            T(0);

        T tMax =
            std::numeric_limits<T>::max();

        Vector3<T> hitNormal =
            Vector3<T>::Zero();

        auto testAxis =
            [&](
                T origin,
                T direction,
                T minValue,
                T maxValue,
                const Vector3<T>& negativeNormal,
                const Vector3<T>& positiveNormal) noexcept -> bool
            {
                if (std::abs(direction) <= epsilon)
                {
                    return
                        origin >= minValue &&
                        origin <= maxValue;
                }

                const T inverseDirection =
                    T(1) / direction;

                T t1 =
                    (minValue - origin) *
                    inverseDirection;

                T t2 =
                    (maxValue - origin) *
                    inverseDirection;

                Vector3<T> normalForNear =
                    negativeNormal;

                if (t1 > t2)
                {
                    const T temp =
                        t1;

                    t1 = t2;
                    t2 = temp;

                    normalForNear =
                        positiveNormal;
                }

                if (t1 > tMin)
                {
                    tMin =
                        t1;

                    hitNormal =
                        normalForNear;
                }

                tMax =
                    Min(
                        tMax,
                        t2);

                return tMin <= tMax;
            };

        if (!testAxis(
                ray.Origin.x,
                ray.Direction.x,
                box.Min.x,
                box.Max.x,
                -Vector3<T>::UnitX(),
                Vector3<T>::UnitX()))
        {
            return std::nullopt;
        }

        if (!testAxis(
                ray.Origin.y,
                ray.Direction.y,
                box.Min.y,
                box.Max.y,
                -Vector3<T>::UnitY(),
                Vector3<T>::UnitY()))
        {
            return std::nullopt;
        }

        if (!testAxis(
                ray.Origin.z,
                ray.Direction.z,
                box.Min.z,
                box.Max.z,
                -Vector3<T>::UnitZ(),
                Vector3<T>::UnitZ()))
        {
            return std::nullopt;
        }

        if (tMax < T(0))
        {
            return std::nullopt;
        }

        const T t =
            tMin >= T(0)
            ? tMin
            : tMax;

        Vector3<T> normal =
            hitNormal;

        if (normal.IsZero())
        {
            const Vector3<T> point =
                ray.GetPoint(t);

            const Vector3<T> center =
                box.Center();

            const Vector3<T> local =
                point - center;

            const Vector3<T> extent =
                box.Extent();

            const Vector3<T> normalized
            {
                extent.x > epsilon ? local.x / extent.x : T(0),
                extent.y > epsilon ? local.y / extent.y : T(0),
                extent.z > epsilon ? local.z / extent.z : T(0)
            };

            const Vector3<T> absNormalized =
                Abs(normalized);

            if (absNormalized.x >= absNormalized.y &&
                absNormalized.x >= absNormalized.z)
            {
                normal =
                    normalized.x >= T(0)
                    ? Vector3<T>::UnitX()
                    : -Vector3<T>::UnitX();
            }
            else if (absNormalized.y >= absNormalized.z)
            {
                normal =
                    normalized.y >= T(0)
                    ? Vector3<T>::UnitY()
                    : -Vector3<T>::UnitY();
            }
            else
            {
                normal =
                    normalized.z >= T(0)
                    ? Vector3<T>::UnitZ()
                    : -Vector3<T>::UnitZ();
            }
        }

        return RayHit<T>
        {
            t,
            ray.GetPoint(t),
            normal
        };
    }

    //=========================================================
    // Ray / OBB
    //
    // Converts the ray into OBB local space and uses the AABB
    // slab test there. The reported point and normal are converted back to the
    // caller's world/local space.
    //=========================================================

    /// Input: function parameters as declared by `RayOBB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `RayOBB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    std::optional<RayHit<T>> RayOBB(
        const Ray<T>& ray,
        const OBB<T>& box,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        const Vector3<T> localOrigin =
            box.WorldToLocalPoint(ray.Origin);

        const Vector3<T> localDirection =
            box.WorldToLocalDirection(ray.Direction);

        const AABB<T> localBox =
            AABB<T>::FromMinMax(
                -box.HalfExtents,
                box.HalfExtents);

        const Ray<T> localRay
        {
            localOrigin,
            localDirection
        };

        const std::optional<RayHit<T>> localHit =
            RayAABB(
                localRay,
                localBox,
                epsilon);

        if (!localHit)
        {
            return std::nullopt;
        }

        const Vector3<T> worldPoint =
            box.LocalToWorldPoint(
                localHit->Point);

        const Vector3<T> worldNormal =
            SafeNormalize(
                box.LocalToWorldDirection(
                    localHit->Normal),
                Vector3<T>::Up());

        return RayHit<T>
        {
            localHit->Distance,
            worldPoint,
            worldNormal
        };
    }

    //=========================================================
    // Intersection Detail Helpers
    //=========================================================

    namespace intersection_detail
    {
        template<FloatingPoint T>
        [[nodiscard]]
        constexpr bool OverlapsOnAxis(
            const Triangle<T>& triangle,
            const AABB<T>& box,
            const Vector3<T>& axis,
            T epsilon) noexcept
        {
            const T axisLengthSquared =
                axis.LengthSquared();

            if (axisLengthSquared <= epsilon * epsilon)
            {
                return true;
            }

            const T p0 =
                Dot(triangle.A, axis);

            const T p1 =
                Dot(triangle.B, axis);

            const T p2 =
                Dot(triangle.C, axis);

            const T triangleMin =
                Min(
                    p0,
                    Min(p1, p2));

            const T triangleMax =
                Max(
                    p0,
                    Max(p1, p2));

            const Vector3<T> center =
                box.Center();

            const Vector3<T> extent =
                box.Extent();

            const T boxCenter =
                Dot(center, axis);

            const T boxRadius =
                extent.x * std::abs(axis.x) +
                extent.y * std::abs(axis.y) +
                extent.z * std::abs(axis.z);

            return
                triangleMin <= boxCenter + boxRadius + epsilon &&
                triangleMax >= boxCenter - boxRadius - epsilon;
        }

        template<FloatingPoint T>
        [[nodiscard]]
        constexpr bool TriangleAABBOverlap(
            const Triangle<T>& triangle,
            const AABB<T>& box,
            T epsilon) noexcept
        {
            if (!box.IsValid())
            {
                return false;
            }

            const Vector3<T> edge0 =
                triangle.B - triangle.A;

            const Vector3<T> edge1 =
                triangle.C - triangle.B;

            const Vector3<T> edge2 =
                triangle.A - triangle.C;

            const Vector3<T> axes[13]
            {
                Vector3<T>::UnitX(),
                Vector3<T>::UnitY(),
                Vector3<T>::UnitZ(),
                Cross(edge0, edge1),
                Cross(edge0, Vector3<T>::UnitX()),
                Cross(edge0, Vector3<T>::UnitY()),
                Cross(edge0, Vector3<T>::UnitZ()),
                Cross(edge1, Vector3<T>::UnitX()),
                Cross(edge1, Vector3<T>::UnitY()),
                Cross(edge1, Vector3<T>::UnitZ()),
                Cross(edge2, Vector3<T>::UnitX()),
                Cross(edge2, Vector3<T>::UnitY()),
                Cross(edge2, Vector3<T>::UnitZ())
            };

            for (const Vector3<T>& axis : axes)
            {
                if (!OverlapsOnAxis(
                        triangle,
                        box,
                        axis,
                        epsilon))
                {
                    return false;
                }
            }

            return true;
        }

        template<FloatingPoint T>
        [[nodiscard]]
        constexpr Segment<T> AABBEdge(
            const AABB<T>& box,
            std::size_t index) noexcept
        {
            constexpr std::size_t edgeCorners[12][2]
            {
                { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 },
                { 0, 2 }, { 1, 3 }, { 4, 6 }, { 5, 7 },
                { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
            };

            return
            {
                box.Corner(edgeCorners[index][0]),
                box.Corner(edgeCorners[index][1])
            };
        }

        template<FloatingPoint T>
        [[nodiscard]]
        T DistanceSquaredSegmentAABB(
            const Segment<T>& segment,
            const AABB<T>& box) noexcept
        {
            const Ray<T> ray
            {
                segment.Start,
                segment.End - segment.Start
            };

            const std::optional<RayHit<T>> hit =
                RayAABB(
                    ray,
                    box);

            if (hit &&
                hit->Distance >= T(0) &&
                hit->Distance <= T(1))
            {
                return T(0);
            }

            T distanceSquared =
                Min(
                    box.DistanceSquaredToPoint(segment.Start),
                    box.DistanceSquaredToPoint(segment.End));

            for (std::size_t i = 0; i < 12; ++i)
            {
                distanceSquared =
                    Min(
                        distanceSquared,
                        DistanceSquared(
                            segment,
                            AABBEdge(box, i)));
            }

            return distanceSquared;
        }

        template<FloatingPoint T>
        [[nodiscard]]
        bool SegmentTriangle(
            const Segment<T>& segment,
            const Triangle<T>& triangle,
            T epsilon) noexcept
        {
            const Ray<T> ray
            {
                segment.Start,
                segment.End - segment.Start
            };

            const std::optional<RayHit<T>> hit =
                RayTriangle(
                    ray,
                    triangle,
                    epsilon);

            return
                hit &&
                hit->Distance >= -epsilon &&
                hit->Distance <= T(1) + epsilon;
        }

        template<FloatingPoint T>
        [[nodiscard]]
        bool CoplanarTriangleTriangle(
            const Triangle<T>& lhs,
            const Triangle<T>& rhs,
            T epsilon) noexcept
        {
            const Vector3<T> normal =
                Abs(lhs.UnnormalizedNormal());

            int dropAxis = 2;

            if (normal.x >= normal.y &&
                normal.x >= normal.z)
            {
                dropAxis = 0;
            }
            else if (normal.y >= normal.z)
            {
                dropAxis = 1;
            }

            const int axis0 =
                dropAxis == 0 ? 1 : 0;

            const int axis1 =
                dropAxis == 2 ? 1 : 2;

            auto component =
                [](
                    const Vector3<T>& point,
                    int axis) noexcept -> T
                {
                    return axis == 0
                        ? point.x
                        : (axis == 1 ? point.y : point.z);
                };

            auto orient =
                [&](const Vector3<T>& a,
                    const Vector3<T>& b,
                    const Vector3<T>& c) noexcept -> T
                {
                    const T ax =
                        component(a, axis0);
                    const T ay =
                        component(a, axis1);
                    const T bx =
                        component(b, axis0);
                    const T by =
                        component(b, axis1);
                    const T cx =
                        component(c, axis0);
                    const T cy =
                        component(c, axis1);

                    return
                        (bx - ax) * (cy - ay) -
                        (by - ay) * (cx - ax);
                };

            auto pointInTriangle =
                [&](const Vector3<T>& point,
                    const Triangle<T>& triangle) noexcept -> bool
                {
                    const T o0 =
                        orient(triangle.A, triangle.B, point);
                    const T o1 =
                        orient(triangle.B, triangle.C, point);
                    const T o2 =
                        orient(triangle.C, triangle.A, point);

                    return
                        (o0 >= -epsilon && o1 >= -epsilon && o2 >= -epsilon) ||
                        (o0 <= epsilon && o1 <= epsilon && o2 <= epsilon);
                };

            auto segmentsOverlap =
                [&](const Vector3<T>& a0,
                    const Vector3<T>& a1,
                    const Vector3<T>& b0,
                    const Vector3<T>& b1) noexcept -> bool
                {
                    const T o0 =
                        orient(a0, a1, b0);
                    const T o1 =
                        orient(a0, a1, b1);
                    const T o2 =
                        orient(b0, b1, a0);
                    const T o3 =
                        orient(b0, b1, a1);

                    return
                        ((o0 >= -epsilon && o1 <= epsilon) ||
                         (o0 <= epsilon && o1 >= -epsilon)) &&
                        ((o2 >= -epsilon && o3 <= epsilon) ||
                         (o2 <= epsilon && o3 >= -epsilon));
                };

            const Vector3<T> lhsEdges[3][2]
            {
                { lhs.A, lhs.B },
                { lhs.B, lhs.C },
                { lhs.C, lhs.A }
            };

            const Vector3<T> rhsEdges[3][2]
            {
                { rhs.A, rhs.B },
                { rhs.B, rhs.C },
                { rhs.C, rhs.A }
            };

            for (const auto& lhsEdge : lhsEdges)
            {
                for (const auto& rhsEdge : rhsEdges)
                {
                    if (segmentsOverlap(
                            lhsEdge[0],
                            lhsEdge[1],
                            rhsEdge[0],
                            rhsEdge[1]))
                    {
                        return true;
                    }
                }
            }

            return
                pointInTriangle(lhs.A, rhs) ||
                pointInTriangle(rhs.A, lhs);
        }

        template<FloatingPoint T>
        [[nodiscard]]
        T DistanceSquaredSegmentTriangle(
            const Segment<T>& segment,
            const Triangle<T>& triangle,
            T epsilon) noexcept
        {
            if (SegmentTriangle(
                    segment,
                    triangle,
                    epsilon))
            {
                return T(0);
            }

            T distanceSquared =
                Min(
                    DistanceSquared(
                        segment.Start,
                        ClosestPointOnTriangle(
                            triangle,
                            segment.Start)),
                    DistanceSquared(
                        segment.End,
                        ClosestPointOnTriangle(
                            triangle,
                            segment.End)));

            const Segment<T> edges[3]
            {
                { triangle.A, triangle.B },
                { triangle.B, triangle.C },
                { triangle.C, triangle.A }
            };

            for (const Segment<T>& edge : edges)
            {
                distanceSquared =
                    Min(
                        distanceSquared,
                        DistanceSquared(
                            segment,
                            edge));
            }

            return distanceSquared;
        }
    }

    //=========================================================
    // Ray / Capsule
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    std::optional<RayHit<T>> RayCapsule(
        const Ray<T>& ray,
        const Capsule<T>& capsule,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        const Segment<T> axis
        {
            capsule.Start,
            capsule.End
        };

        if (axis.IsPoint(epsilon))
        {
            return RaySphere(
                ray,
                Sphere<T>
                {
                    capsule.Start,
                    capsule.Radius
                },
                epsilon);
        }

        std::optional<RayHit<T>> bestHit =
            std::nullopt;

        auto consider =
            [&](const std::optional<RayHit<T>>& hit) noexcept
            {
                if (hit &&
                    (!bestHit ||
                     hit->Distance < bestHit->Distance))
                {
                    bestHit = hit;
                }
            };

        consider(
            RaySphere(
                ray,
                Sphere<T>{ capsule.Start, capsule.Radius },
                epsilon));

        consider(
            RaySphere(
                ray,
                Sphere<T>{ capsule.End, capsule.Radius },
                epsilon));

        const Vector3<T> d =
            capsule.End - capsule.Start;

        const Vector3<T> m =
            ray.Origin - capsule.Start;

        const Vector3<T> n =
            ray.Direction;

        const T md =
            Dot(m, d);

        const T nd =
            Dot(n, d);

        const T dd =
            Dot(d, d);

        const T nn =
            Dot(n, n);

        const T a =
            dd * nn -
            nd * nd;

        if (std::abs(a) > epsilon)
        {
            const T k =
                Dot(m, m) -
                capsule.Radius * capsule.Radius;

            const T b =
                dd * Dot(m, n) -
                md * nd;

            const T c =
                dd * k -
                md * md;

            const T discriminant =
                b * b -
                a * c;

            if (discriminant >= T(0))
            {
                const T t =
                    (-b - std::sqrt(discriminant)) /
                    a;

                const T y =
                    md + t * nd;

                if (t >= T(0) &&
                    y >= T(0) &&
                    y <= dd)
                {
                    const Vector3<T> point =
                        ray.GetPoint(t);

                    const Vector3<T> axisPoint =
                        capsule.Start +
                        d * (y / dd);

                    const RayHit<T> bodyHit
                    {
                        t,
                        point,
                        SafeNormalize(
                            point - axisPoint,
                            Vector3<T>::Up())
                    };

                    if (!bestHit ||
                        bodyHit.Distance < bestHit->Distance)
                    {
                        bestHit = bodyHit;
                    }
                }
            }
        }

        return bestHit;
    }

    //=========================================================
    // Sphere / Sphere
    //=========================================================

    /// Input: function parameters as declared by `SphereSphere`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SphereSphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool SphereSphere(
        const Sphere<T>& lhs,
        const Sphere<T>& rhs) noexcept
    {
        const T radiusSum =
            lhs.Radius + rhs.Radius;

        return
            DistanceSquared(
                lhs.Center,
                rhs.Center)
            <= radiusSum * radiusSum;
    }

    //=========================================================
    // Sphere / AABB
    //=========================================================

    /// Input: function parameters as declared by `SphereAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SphereAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool SphereAABB(
        const Sphere<T>& sphere,
        const AABB<T>& box) noexcept
    {
        const Vector3<T> closest =
            box.ClosestPoint(
                sphere.Center);

        return
            DistanceSquared(
                sphere.Center,
                closest)
            <= sphere.Radius * sphere.Radius;
    }

    /// Input: function parameters as declared by `AABBSphere`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `AABBSphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool AABBSphere(
        const AABB<T>& box,
        const Sphere<T>& sphere) noexcept
    {
        return SphereAABB(
            sphere,
            box);
    }

    //=========================================================
    // Sphere / Capsule
    //=========================================================

    /// Input: function parameters as declared by `SphereCapsule`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SphereCapsule` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool SphereCapsule(
        const Sphere<T>& sphere,
        const Capsule<T>& capsule) noexcept
    {
        const Vector3<T> closestOnAxis =
            capsule.ClosestPointOnAxis(
                sphere.Center);

        const T radiusSum =
            sphere.Radius +
            capsule.Radius;

        return
            DistanceSquared(
                sphere.Center,
                closestOnAxis)
            <= radiusSum * radiusSum;
    }

    /// Input: function parameters as declared by `CapsuleSphere`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `CapsuleSphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool CapsuleSphere(
        const Capsule<T>& capsule,
        const Sphere<T>& sphere) noexcept
    {
        return SphereCapsule(
            sphere,
            capsule);
    }

    //=========================================================
    // Capsule / Capsule
    //=========================================================

    /// Input: two capsules.
    /// Output: true if their swept-sphere volumes overlap.
    /// Task: reduce capsule/capsule intersection to segment/segment closest
    /// distance plus summed radii.
    template<FloatingPoint T>
    [[nodiscard]]
    bool CapsuleCapsule(
        const Capsule<T>& lhs,
        const Capsule<T>& rhs) noexcept
    {
        const Segment<T> lhsAxis
        {
            lhs.Start,
            lhs.End
        };

        const Segment<T> rhsAxis
        {
            rhs.Start,
            rhs.End
        };

        const T radiusSum =
            lhs.Radius + rhs.Radius;

        return
            DistanceSquared(lhsAxis, rhsAxis)
            <= radiusSum * radiusSum;
    }

    //=========================================================
    // Sphere / Triangle
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    bool SphereTriangle(
        const Sphere<T>& sphere,
        const Triangle<T>& triangle) noexcept
    {
        const Vector3<T> closest =
            ClosestPointOnTriangle(
                triangle,
                sphere.Center);

        return
            DistanceSquared(
                sphere.Center,
                closest)
            <= sphere.Radius * sphere.Radius;
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool TriangleSphere(
        const Triangle<T>& triangle,
        const Sphere<T>& sphere) noexcept
    {
        return SphereTriangle(
            sphere,
            triangle);
    }

    //=========================================================
    // Sphere / OBB
    //=========================================================

    /// Input: function parameters as declared by `SphereOBB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SphereOBB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool SphereOBB(
        const Sphere<T>& sphere,
        const OBB<T>& box) noexcept
    {
        const Vector3<T> closest =
            box.ClosestPoint(
                sphere.Center);

        return
            DistanceSquared(
                sphere.Center,
                closest)
            <= sphere.Radius * sphere.Radius;
    }

    /// Input: function parameters as declared by `OBBSphere`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `OBBSphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool OBBSphere(
        const OBB<T>& box,
        const Sphere<T>& sphere) noexcept
    {
        return SphereOBB(
            sphere,
            box);
    }

    //=========================================================
    // AABB / Capsule
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    bool AABBCapsule(
        const AABB<T>& box,
        const Capsule<T>& capsule) noexcept
    {
        const Segment<T> axis
        {
            capsule.Start,
            capsule.End
        };

        return
            intersection_detail::DistanceSquaredSegmentAABB(
                axis,
                box)
            <= capsule.Radius * capsule.Radius;
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool CapsuleAABB(
        const Capsule<T>& capsule,
        const AABB<T>& box) noexcept
    {
        return AABBCapsule(
            box,
            capsule);
    }

    //=========================================================
    // OBB / Capsule
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    bool OBBCapsule(
        const OBB<T>& box,
        const Capsule<T>& capsule) noexcept
    {
        const Capsule<T> localCapsule
        {
            box.WorldToLocalPoint(capsule.Start),
            box.WorldToLocalPoint(capsule.End),
            capsule.Radius
        };

        const AABB<T> localBox =
            AABB<T>::FromMinMax(
                -box.HalfExtents,
                box.HalfExtents);

        return AABBCapsule(
            localBox,
            localCapsule);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool CapsuleOBB(
        const Capsule<T>& capsule,
        const OBB<T>& box) noexcept
    {
        return OBBCapsule(
            box,
            capsule);
    }

    //=========================================================
    // AABB / AABB
    //=========================================================

    /// Input: function parameters as declared by `AABBAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `AABBAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool AABBAABB(
        const AABB<T>& lhs,
        const AABB<T>& rhs) noexcept
    {
        return Intersects(
            lhs,
            rhs);
    }

    //=========================================================
    // Plane / Sphere
    //=========================================================

    /// Input: function parameters as declared by `PlaneSphere`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `PlaneSphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool PlaneSphere(
        const Plane<T>& plane,
        const Sphere<T>& sphere) noexcept
    {
        return
            std::abs(
                plane.SignedDistanceToPoint(
                    sphere.Center))
            <= sphere.Radius;
    }

    /// Input: function parameters as declared by `SpherePlane`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SpherePlane` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool SpherePlane(
        const Sphere<T>& sphere,
        const Plane<T>& plane) noexcept
    {
        return PlaneSphere(
            plane,
            sphere);
    }

    //=========================================================
    // Plane / AABB
    //
    // Returns true if the plane intersects the box volume.
    //=========================================================

    /// Input: function parameters as declared by `PlaneAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `PlaneAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool PlaneAABB(
        const Plane<T>& plane,
        const AABB<T>& box) noexcept
    {
        const Vector3<T> center =
            box.Center();

        const Vector3<T> extent =
            box.Extent();

        const T projectedRadius =
            extent.x * std::abs(plane.Normal.x) +
            extent.y * std::abs(plane.Normal.y) +
            extent.z * std::abs(plane.Normal.z);

        const T signedDistance =
            plane.SignedDistanceToPoint(
                center);

        return
            std::abs(signedDistance)
            <= projectedRadius;
    }

    /// Input: function parameters as declared by `AABBPlane`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `AABBPlane` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool AABBPlane(
        const AABB<T>& box,
        const Plane<T>& plane) noexcept
    {
        return PlaneAABB(
            plane,
            box);
    }

    //=========================================================
    // Plane / OBB
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    bool PlaneOBB(
        const Plane<T>& plane,
        const OBB<T>& box) noexcept
    {
        const T projectedRadius =
            box.HalfExtents.x * std::abs(Dot(plane.Normal, box.Axes[0])) +
            box.HalfExtents.y * std::abs(Dot(plane.Normal, box.Axes[1])) +
            box.HalfExtents.z * std::abs(Dot(plane.Normal, box.Axes[2]));

        const T signedDistance =
            plane.SignedDistanceToPoint(
                box.Center);

        return
            std::abs(signedDistance)
            <= projectedRadius;
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool OBBPlane(
        const OBB<T>& box,
        const Plane<T>& plane) noexcept
    {
        return PlaneOBB(
            plane,
            box);
    }

    //=========================================================
    // Plane / Capsule
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    bool PlaneCapsule(
        const Plane<T>& plane,
        const Capsule<T>& capsule) noexcept
    {
        const T startDistance =
            plane.SignedDistanceToPoint(
                capsule.Start);

        const T endDistance =
            plane.SignedDistanceToPoint(
                capsule.End);

        if ((startDistance <= T(0) && endDistance >= T(0)) ||
            (startDistance >= T(0) && endDistance <= T(0)))
        {
            return true;
        }

        return
            Min(
                std::abs(startDistance),
                std::abs(endDistance))
            <= capsule.Radius;
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool CapsulePlane(
        const Capsule<T>& capsule,
        const Plane<T>& plane) noexcept
    {
        return PlaneCapsule(
            plane,
            capsule);
    }

    //=========================================================
    // Plane / Triangle
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    bool PlaneTriangle(
        const Plane<T>& plane,
        const Triangle<T>& triangle,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        const T da =
            plane.SignedDistanceToPoint(
                triangle.A);

        const T db =
            plane.SignedDistanceToPoint(
                triangle.B);

        const T dc =
            plane.SignedDistanceToPoint(
                triangle.C);

        const T minDistance =
            Min(
                da,
                Min(db, dc));

        const T maxDistance =
            Max(
                da,
                Max(db, dc));

        return
            minDistance <= epsilon &&
            maxDistance >= -epsilon;
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool TrianglePlane(
        const Triangle<T>& triangle,
        const Plane<T>& plane,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        return PlaneTriangle(
            plane,
            triangle,
            epsilon);
    }

    //=========================================================
    // Triangle / AABB
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool TriangleAABB(
        const Triangle<T>& triangle,
        const AABB<T>& box,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        return intersection_detail::TriangleAABBOverlap(
            triangle,
            box,
            epsilon);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool AABBTriangle(
        const AABB<T>& box,
        const Triangle<T>& triangle,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        return TriangleAABB(
            triangle,
            box,
            epsilon);
    }

    //=========================================================
    // Triangle / OBB
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool TriangleOBB(
        const Triangle<T>& triangle,
        const OBB<T>& box,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        const Triangle<T> localTriangle
        {
            box.WorldToLocalPoint(triangle.A),
            box.WorldToLocalPoint(triangle.B),
            box.WorldToLocalPoint(triangle.C)
        };

        const AABB<T> localBox =
            AABB<T>::FromMinMax(
                -box.HalfExtents,
                box.HalfExtents);

        return TriangleAABB(
            localTriangle,
            localBox,
            epsilon);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool OBBTriangle(
        const OBB<T>& box,
        const Triangle<T>& triangle,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        return TriangleOBB(
            triangle,
            box,
            epsilon);
    }

    //=========================================================
    // Triangle / Capsule
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    bool TriangleCapsule(
        const Triangle<T>& triangle,
        const Capsule<T>& capsule,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        const Segment<T> axis
        {
            capsule.Start,
            capsule.End
        };

        return
            intersection_detail::DistanceSquaredSegmentTriangle(
                axis,
                triangle,
                epsilon)
            <= capsule.Radius * capsule.Radius;
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool CapsuleTriangle(
        const Capsule<T>& capsule,
        const Triangle<T>& triangle,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        return TriangleCapsule(
            triangle,
            capsule,
            epsilon);
    }

    //=========================================================
    // Triangle / Triangle
    //=========================================================

    template<FloatingPoint T>
    [[nodiscard]]
    bool TriangleTriangle(
        const Triangle<T>& lhs,
        const Triangle<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        const Plane<T> lhsPlane =
            lhs.ToPlane();

        const T rda =
            lhsPlane.SignedDistanceToPoint(rhs.A);
        const T rdb =
            lhsPlane.SignedDistanceToPoint(rhs.B);
        const T rdc =
            lhsPlane.SignedDistanceToPoint(rhs.C);

        if ((rda > epsilon && rdb > epsilon && rdc > epsilon) ||
            (rda < -epsilon && rdb < -epsilon && rdc < -epsilon))
        {
            return false;
        }

        const Plane<T> rhsPlane =
            rhs.ToPlane();

        const T lda =
            rhsPlane.SignedDistanceToPoint(lhs.A);
        const T ldb =
            rhsPlane.SignedDistanceToPoint(lhs.B);
        const T ldc =
            rhsPlane.SignedDistanceToPoint(lhs.C);

        if ((lda > epsilon && ldb > epsilon && ldc > epsilon) ||
            (lda < -epsilon && ldb < -epsilon && ldc < -epsilon))
        {
            return false;
        }

        if (std::abs(rda) <= epsilon &&
            std::abs(rdb) <= epsilon &&
            std::abs(rdc) <= epsilon &&
            std::abs(lda) <= epsilon &&
            std::abs(ldb) <= epsilon &&
            std::abs(ldc) <= epsilon)
        {
            return intersection_detail::CoplanarTriangleTriangle(
                lhs,
                rhs,
                epsilon);
        }

        const Segment<T> lhsEdges[3]
        {
            { lhs.A, lhs.B },
            { lhs.B, lhs.C },
            { lhs.C, lhs.A }
        };

        const Segment<T> rhsEdges[3]
        {
            { rhs.A, rhs.B },
            { rhs.B, rhs.C },
            { rhs.C, rhs.A }
        };

        for (const Segment<T>& edge : lhsEdges)
        {
            if (intersection_detail::SegmentTriangle(
                    edge,
                    rhs,
                    epsilon))
            {
                return true;
            }
        }

        for (const Segment<T>& edge : rhsEdges)
        {
            if (intersection_detail::SegmentTriangle(
                    edge,
                    lhs,
                    epsilon))
            {
                return true;
            }
        }

        return false;
    }

    //=========================================================
    // OBB / OBB
    //
    // Separating Axis Theorem with 15 candidate axes:
    // - 3 axes from lhs
    // - 3 axes from rhs
    // - 9 cross-product axes
    //=========================================================

    /// Input: function parameters as declared by `OBBOBB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `OBBOBB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool OBBOBB(
        const OBB<T>& lhs,
        const OBB<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        T rotation[3][3] {};
        T absRotation[3][3] {};

        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 3; ++j)
            {
                rotation[i][j] =
                    Dot(
                        lhs.Axes[i],
                        rhs.Axes[j]);

                absRotation[i][j] =
                    std::abs(rotation[i][j]) +
                    epsilon;
            }
        }

        const Vector3<T> delta =
            rhs.Center - lhs.Center;

        const Vector3<T> translation
        {
            Dot(delta, lhs.Axes[0]),
            Dot(delta, lhs.Axes[1]),
            Dot(delta, lhs.Axes[2])
        };

        const T lhsExtents[3]
        {
            lhs.HalfExtents.x,
            lhs.HalfExtents.y,
            lhs.HalfExtents.z
        };

        const T rhsExtents[3]
        {
            rhs.HalfExtents.x,
            rhs.HalfExtents.y,
            rhs.HalfExtents.z
        };

        T ra {};
        T rb {};

        for (std::size_t i = 0; i < 3; ++i)
        {
            ra =
                lhsExtents[i];

            rb =
                rhsExtents[0] * absRotation[i][0] +
                rhsExtents[1] * absRotation[i][1] +
                rhsExtents[2] * absRotation[i][2];

            if (std::abs(translation[i]) > ra + rb)
            {
                return false;
            }
        }

        for (std::size_t j = 0; j < 3; ++j)
        {
            ra =
                lhsExtents[0] * absRotation[0][j] +
                lhsExtents[1] * absRotation[1][j] +
                lhsExtents[2] * absRotation[2][j];

            rb =
                rhsExtents[j];

            const T projectedTranslation =
                std::abs(
                    translation.x * rotation[0][j] +
                    translation.y * rotation[1][j] +
                    translation.z * rotation[2][j]);

            if (projectedTranslation > ra + rb)
            {
                return false;
            }
        }

        ra = lhsExtents[1] * absRotation[2][0] + lhsExtents[2] * absRotation[1][0];
        rb = rhsExtents[1] * absRotation[0][2] + rhsExtents[2] * absRotation[0][1];
        if (std::abs(translation.z * rotation[1][0] - translation.y * rotation[2][0]) > ra + rb) { return false; }

        ra = lhsExtents[1] * absRotation[2][1] + lhsExtents[2] * absRotation[1][1];
        rb = rhsExtents[0] * absRotation[0][2] + rhsExtents[2] * absRotation[0][0];
        if (std::abs(translation.z * rotation[1][1] - translation.y * rotation[2][1]) > ra + rb) { return false; }

        ra = lhsExtents[1] * absRotation[2][2] + lhsExtents[2] * absRotation[1][2];
        rb = rhsExtents[0] * absRotation[0][1] + rhsExtents[1] * absRotation[0][0];
        if (std::abs(translation.z * rotation[1][2] - translation.y * rotation[2][2]) > ra + rb) { return false; }

        ra = lhsExtents[0] * absRotation[2][0] + lhsExtents[2] * absRotation[0][0];
        rb = rhsExtents[1] * absRotation[1][2] + rhsExtents[2] * absRotation[1][1];
        if (std::abs(translation.x * rotation[2][0] - translation.z * rotation[0][0]) > ra + rb) { return false; }

        ra = lhsExtents[0] * absRotation[2][1] + lhsExtents[2] * absRotation[0][1];
        rb = rhsExtents[0] * absRotation[1][2] + rhsExtents[2] * absRotation[1][0];
        if (std::abs(translation.x * rotation[2][1] - translation.z * rotation[0][1]) > ra + rb) { return false; }

        ra = lhsExtents[0] * absRotation[2][2] + lhsExtents[2] * absRotation[0][2];
        rb = rhsExtents[0] * absRotation[1][1] + rhsExtents[1] * absRotation[1][0];
        if (std::abs(translation.x * rotation[2][2] - translation.z * rotation[0][2]) > ra + rb) { return false; }

        ra = lhsExtents[0] * absRotation[1][0] + lhsExtents[1] * absRotation[0][0];
        rb = rhsExtents[1] * absRotation[2][2] + rhsExtents[2] * absRotation[2][1];
        if (std::abs(translation.y * rotation[0][0] - translation.x * rotation[1][0]) > ra + rb) { return false; }

        ra = lhsExtents[0] * absRotation[1][1] + lhsExtents[1] * absRotation[0][1];
        rb = rhsExtents[0] * absRotation[2][2] + rhsExtents[2] * absRotation[2][0];
        if (std::abs(translation.y * rotation[0][1] - translation.x * rotation[1][1]) > ra + rb) { return false; }

        ra = lhsExtents[0] * absRotation[1][2] + lhsExtents[1] * absRotation[0][2];
        rb = rhsExtents[0] * absRotation[2][1] + rhsExtents[1] * absRotation[2][0];
        if (std::abs(translation.y * rotation[0][2] - translation.x * rotation[1][2]) > ra + rb) { return false; }

        return true;
    }

    //=========================================================
    // AABB / OBB
    //=========================================================

    /// Input: function parameters as declared by `AABBOBB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `AABBOBB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool AABBOBB(
        const AABB<T>& aabb,
        const OBB<T>& obb,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        return OBBOBB(
            OBB<T>::FromAABB(aabb),
            obb,
            epsilon);
    }

    /// Input: function parameters as declared by `OBBAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `OBBAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool OBBAABB(
        const OBB<T>& obb,
        const AABB<T>& aabb,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(100)) noexcept
    {
        return AABBOBB(
            aabb,
            obb,
            epsilon);
    }

    //=========================================================
    // Generic Intersects Overloads
    //=========================================================
    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const Sphere<T>& sphere,
        const AABB<T>& box) noexcept
    {
        return SphereAABB(
            sphere,
            box);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const AABB<T>& box,
        const Sphere<T>& sphere) noexcept
    {
        return SphereAABB(
            sphere,
            box);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Plane<T>& plane,
        const Sphere<T>& sphere) noexcept
    {
        return PlaneSphere(
            plane,
            sphere);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Sphere<T>& sphere,
        const Plane<T>& plane) noexcept
    {
        return PlaneSphere(
            plane,
            sphere);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Plane<T>& plane,
        const AABB<T>& box) noexcept
    {
        return PlaneAABB(
            plane,
            box);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const AABB<T>& box,
        const Plane<T>& plane) noexcept
    {
        return PlaneAABB(
            plane,
            box);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Sphere<T>& sphere,
        const Capsule<T>& capsule) noexcept
    {
        return SphereCapsule(
            sphere,
            capsule);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Capsule<T>& capsule,
        const Sphere<T>& sphere) noexcept
    {
        return SphereCapsule(
            sphere,
            capsule);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Capsule<T>& lhs,
        const Capsule<T>& rhs) noexcept
    {
        return CapsuleCapsule(
            lhs,
            rhs);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Sphere<T>& sphere,
        const Triangle<T>& triangle) noexcept
    {
        return SphereTriangle(
            sphere,
            triangle);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Triangle<T>& triangle,
        const Sphere<T>& sphere) noexcept
    {
        return SphereTriangle(
            sphere,
            triangle);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const Sphere<T>& sphere,
        const OBB<T>& box) noexcept
    {
        return SphereOBB(
            sphere,
            box);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const OBB<T>& box,
        const Sphere<T>& sphere) noexcept
    {
        return SphereOBB(
            sphere,
            box);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const AABB<T>& box,
        const Capsule<T>& capsule) noexcept
    {
        return AABBCapsule(
            box,
            capsule);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Capsule<T>& capsule,
        const AABB<T>& box) noexcept
    {
        return AABBCapsule(
            box,
            capsule);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const OBB<T>& box,
        const Capsule<T>& capsule) noexcept
    {
        return OBBCapsule(
            box,
            capsule);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Capsule<T>& capsule,
        const OBB<T>& box) noexcept
    {
        return OBBCapsule(
            box,
            capsule);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const AABB<T>& aabb,
        const OBB<T>& obb) noexcept
    {
        return AABBOBB(
            aabb,
            obb);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const OBB<T>& obb,
        const AABB<T>& aabb) noexcept
    {
        return AABBOBB(
            aabb,
            obb);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Plane<T>& plane,
        const OBB<T>& box) noexcept
    {
        return PlaneOBB(
            plane,
            box);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const OBB<T>& box,
        const Plane<T>& plane) noexcept
    {
        return PlaneOBB(
            plane,
            box);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Plane<T>& plane,
        const Capsule<T>& capsule) noexcept
    {
        return PlaneCapsule(
            plane,
            capsule);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Capsule<T>& capsule,
        const Plane<T>& plane) noexcept
    {
        return PlaneCapsule(
            plane,
            capsule);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Plane<T>& plane,
        const Triangle<T>& triangle) noexcept
    {
        return PlaneTriangle(
            plane,
            triangle);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Triangle<T>& triangle,
        const Plane<T>& plane) noexcept
    {
        return PlaneTriangle(
            plane,
            triangle);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const Triangle<T>& triangle,
        const AABB<T>& box) noexcept
    {
        return TriangleAABB(
            triangle,
            box);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const AABB<T>& box,
        const Triangle<T>& triangle) noexcept
    {
        return TriangleAABB(
            triangle,
            box);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const Triangle<T>& triangle,
        const OBB<T>& box) noexcept
    {
        return TriangleOBB(
            triangle,
            box);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const OBB<T>& box,
        const Triangle<T>& triangle) noexcept
    {
        return TriangleOBB(
            triangle,
            box);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Triangle<T>& triangle,
        const Capsule<T>& capsule) noexcept
    {
        return TriangleCapsule(
            triangle,
            capsule);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Capsule<T>& capsule,
        const Triangle<T>& triangle) noexcept
    {
        return TriangleCapsule(
            triangle,
            capsule);
    }

    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const Triangle<T>& lhs,
        const Triangle<T>& rhs) noexcept
    {
        return TriangleTriangle(
            lhs,
            rhs);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Intersects(
        const OBB<T>& lhs,
        const OBB<T>& rhs) noexcept
    {
        return OBBOBB(
            lhs,
            rhs);
    }

} // namespace kairo::foundation::geometry
