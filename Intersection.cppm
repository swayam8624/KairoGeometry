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
