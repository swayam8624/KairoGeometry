module;

#include <array>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <optional>
#include <type_traits>

export module Kairo.Foundation.Geometry.Frustum;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;

import Kairo.Foundation.Geometry.Plane;
import Kairo.Foundation.Geometry.Sphere;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Geometry.OBB;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Frustum
    //=========================================================

    /// Camera/view frustum represented by six inward-facing clipping planes.
    ///
    /// Conventions:
    /// - Plane normals point inward, so points inside the frustum have
    ///   non-negative signed distance for every plane.
    /// - Extraction assumes KairoMath's row-major storage, column-vector
    ///   multiplication, and Vulkan/Metal [0, 1] clip-space depth range.
    /// - `FromViewProjection()` expects the combined projection * view matrix.
    template<FloatingPoint T>
    struct Frustum final
    {
        using ValueType = T;

        Plane<T> Left = Plane<T>::FromNormalDistance(Vector3<T>::UnitX(), T(1));
        Plane<T> Right = Plane<T>::FromNormalDistance(-Vector3<T>::UnitX(), T(1));
        Plane<T> Bottom = Plane<T>::FromNormalDistance(Vector3<T>::UnitY(), T(1));
        Plane<T> Top = Plane<T>::FromNormalDistance(-Vector3<T>::UnitY(), T(1));
        Plane<T> Near = Plane<T>::FromNormalDistance(Vector3<T>::UnitZ(), T(0));
        Plane<T> Far = Plane<T>::FromNormalDistance(-Vector3<T>::UnitZ(), T(1));

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------
        /// Input: function parameters as declared by `FromPlanes`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromPlanes` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Frustum FromPlanes(
            const Plane<T>& left,
            const Plane<T>& right,
            const Plane<T>& bottom,
            const Plane<T>& top,
            const Plane<T>& nearPlane,
            const Plane<T>& farPlane) noexcept
        {
            return
            {
                left,
                right,
                bottom,
                top,
                nearPlane,
                farPlane
            };
        }

        /// Input: combined view-projection matrix.
        /// Output: normalized frustum planes.
        /// Task: extract world-space frustum planes from KairoMath's matrix
        /// convention. For column-vector clip math:
        ///   - left:   row3 + row0
        ///   - right:  row3 - row0
        ///   - bottom: row3 + row1
        ///   - top:    row3 - row1
        ///   - near:   row2        for z >= 0
        ///   - far:    row3 - row2 for z <= w
        /// Input: function parameters as declared by `FromViewProjection`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromViewProjection` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
        [[nodiscard]]
        static Frustum FromViewProjection(
            const Matrix4<T>& viewProjection) noexcept
        {
            auto makePlane =
                [](
                    T a,
                    T b,
                    T c,
                    T d) noexcept -> Plane<T>
                {
                    Plane<T> plane
                    {
                        Vector3<T>{ a, b, c },
                        d
                    };

                    plane.Normalize();
                    return plane;
                };

            return
            {
                makePlane(
                    viewProjection(3, 0) + viewProjection(0, 0),
                    viewProjection(3, 1) + viewProjection(0, 1),
                    viewProjection(3, 2) + viewProjection(0, 2),
                    viewProjection(3, 3) + viewProjection(0, 3)),

                makePlane(
                    viewProjection(3, 0) - viewProjection(0, 0),
                    viewProjection(3, 1) - viewProjection(0, 1),
                    viewProjection(3, 2) - viewProjection(0, 2),
                    viewProjection(3, 3) - viewProjection(0, 3)),

                makePlane(
                    viewProjection(3, 0) + viewProjection(1, 0),
                    viewProjection(3, 1) + viewProjection(1, 1),
                    viewProjection(3, 2) + viewProjection(1, 2),
                    viewProjection(3, 3) + viewProjection(1, 3)),

                makePlane(
                    viewProjection(3, 0) - viewProjection(1, 0),
                    viewProjection(3, 1) - viewProjection(1, 1),
                    viewProjection(3, 2) - viewProjection(1, 2),
                    viewProjection(3, 3) - viewProjection(1, 3)),

                makePlane(
                    viewProjection(2, 0),
                    viewProjection(2, 1),
                    viewProjection(2, 2),
                    viewProjection(2, 3)),

                makePlane(
                    viewProjection(3, 0) - viewProjection(2, 0),
                    viewProjection(3, 1) - viewProjection(2, 1),
                    viewProjection(3, 2) - viewProjection(2, 2),
                    viewProjection(3, 3) - viewProjection(2, 3))
            };
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Frustum` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Frustum() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Frustum` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Frustum(
            const Plane<T>& left,
            const Plane<T>& right,
            const Plane<T>& bottom,
            const Plane<T>& top,
            const Plane<T>& nearPlane,
            const Plane<T>& farPlane) noexcept
            : Left(left)
            , Right(right)
            , Bottom(bottom)
            , Top(top)
            , Near(nearPlane)
            , Far(farPlane)
        {
        }

        //-----------------------------------------------------
        // Plane Access
        //-----------------------------------------------------

        /// Input: plane index in [0, 6).
        /// Output: selected plane in order left, right, bottom, top, near, far.
        /// Task: support loops without storing planes in a separate array.
        [[nodiscard]]
        constexpr const Plane<T>& PlaneAt(std::size_t index) const noexcept
        {
            assert(index < 6);

            switch (index)
            {
                case 0: return Left;
                case 1: return Right;
                case 2: return Bottom;
                case 3: return Top;
                case 4: return Near;
                default: return Far;
            }
        }

        //-----------------------------------------------------
        // Containment / Intersection
        //-----------------------------------------------------

        /// Input: point.
        /// Output: true if the point is inside or on every frustum plane.
        /// Task: exact point containment for culling/debug selection.
        [[nodiscard]]
        constexpr bool ContainsPoint(
            const Vector3<T>& point,
            T epsilon = T(0)) const noexcept
        {
            for (std::size_t i = 0; i < 6; ++i)
            {
                if (PlaneAt(i).SignedDistanceToPoint(point) < -epsilon)
                {
                    return false;
                }
            }

            return true;
        }

        /// Input: sphere.
        /// Output: true if sphere is at least partially inside the frustum.
        /// Task: conservative sphere/frustum culling.
        [[nodiscard]]
        constexpr bool IntersectsSphere(
            const Sphere<T>& sphere) const noexcept
        {
            for (std::size_t i = 0; i < 6; ++i)
            {
                if (PlaneAt(i).SignedDistanceToPoint(sphere.Center) < -sphere.Radius)
                {
                    return false;
                }
            }

            return true;
        }

        /// Input: AABB.
        /// Output: true if the box is at least partially inside the frustum.
        /// Task: fast broad-phase camera culling using positive vertex tests.
        [[nodiscard]]
        constexpr bool IntersectsAABB(
            const AABB<T>& box) const noexcept
        {
            if (!box.IsValid())
            {
                return false;
            }

            for (std::size_t i = 0; i < 6; ++i)
            {
                const Plane<T>& plane =
                    PlaneAt(i);

                const Vector3<T> positive
                {
                    plane.Normal.x >= T(0) ? box.Max.x : box.Min.x,
                    plane.Normal.y >= T(0) ? box.Max.y : box.Min.y,
                    plane.Normal.z >= T(0) ? box.Max.z : box.Min.z
                };

                if (plane.SignedDistanceToPoint(positive) < T(0))
                {
                    return false;
                }
            }

            return true;
        }

        /// Input: OBB.
        /// Output: true if the box is at least partially inside the frustum.
        /// Task: conservative culling for rotated bounds using plane-projected
        /// radius tests.
        [[nodiscard]]
        constexpr bool IntersectsOBB(
            const OBB<T>& box) const noexcept
        {
            for (std::size_t i = 0; i < 6; ++i)
            {
                const Plane<T>& plane =
                    PlaneAt(i);

                const T radius =
                    box.HalfExtents.x * std::abs(Dot(plane.Normal, box.Axes[0])) +
                    box.HalfExtents.y * std::abs(Dot(plane.Normal, box.Axes[1])) +
                    box.HalfExtents.z * std::abs(Dot(plane.Normal, box.Axes[2]));

                if (plane.SignedDistanceToPoint(box.Center) < -radius)
                {
                    return false;
                }
            }

            return true;
        }

        //-----------------------------------------------------
        // Corners
        //-----------------------------------------------------

        /// Input: none.
        /// Output: eight frustum corners.
        /// Task: reconstruct corner points for debug drawing and editor tools.
        /// Returns false if any three-plane intersection is degenerate.
        [[nodiscard]]
        bool GetCorners(
            std::array<Vector3<T>, 8>& corners,
            T epsilon =
                std::numeric_limits<T>::epsilon() * T(100)) const noexcept
        {
            auto intersectPlanes =
                [epsilon](
                    const Plane<T>& a,
                    const Plane<T>& b,
                    const Plane<T>& c,
                    Vector3<T>& result) noexcept -> bool
                {
                    const Vector3<T> bc =
                        Cross(b.Normal, c.Normal);

                    const T denominator =
                        Dot(a.Normal, bc);

                    if (std::abs(denominator) <= epsilon)
                    {
                        return false;
                    }

                    result =
                        (
                            bc * (-a.Distance) +
                            Cross(c.Normal, a.Normal) * (-b.Distance) +
                            Cross(a.Normal, b.Normal) * (-c.Distance)
                        ) / denominator;

                    return true;
                };

            return
                intersectPlanes(Left, Bottom, Near, corners[0]) &&
                intersectPlanes(Right, Bottom, Near, corners[1]) &&
                intersectPlanes(Left, Top, Near, corners[2]) &&
                intersectPlanes(Right, Top, Near, corners[3]) &&
                intersectPlanes(Left, Bottom, Far, corners[4]) &&
                intersectPlanes(Right, Bottom, Far, corners[5]) &&
                intersectPlanes(Left, Top, Far, corners[6]) &&
                intersectPlanes(Right, Top, Far, corners[7]);
        }

        /// Input: optional epsilon used to reject degenerate three-plane intersections.
        /// Output: reconstructed corners, or empty when any corner cannot be computed.
        /// Task: provide a release-safe API for callers that accept malformed or
        /// degenerate frustum input at runtime.
        [[nodiscard]]
        std::optional<std::array<Vector3<T>, 8>> TryGetCorners(
            T epsilon =
                std::numeric_limits<T>::epsilon() * T(100)) const noexcept
        {
            std::array<Vector3<T>, 8> corners {};
            if (!GetCorners(corners, epsilon))
            {
                return std::nullopt;
            }

            return corners;
        }

        [[nodiscard]]
        std::array<Vector3<T>, 8> Corners() const noexcept
        {
            std::array<Vector3<T>, 8> corners {};
            const bool valid =
                GetCorners(corners);

            assert(valid);
            return corners;
        }

        //-----------------------------------------------------
        // Mutation / Validation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `NormalizePlanes`.
        /// Output: none; mutates the object or output parameters described by the function.
        /// Task: provide the `NormalizePlanes` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        void NormalizePlanes() noexcept
        {
            Left.Normalize();
            Right.Normalize();
            Bottom.Normalize();
            Top.Normalize();
            Near.Normalize();
            Far.Normalize();
        }
        /// Input: function parameters as declared by `IsValid`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsValid` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool IsValid(
            T epsilon =
                std::numeric_limits<T>::epsilon() * T(100)) const noexcept
        {
            for (std::size_t i = 0; i < 6; ++i)
            {
                if (!PlaneAt(i).IsValid(epsilon))
                {
                    return false;
                }
            }

            return true;
        }
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool operator==(const Frustum&) const noexcept = default;
    };

    //=========================================================
    // Free Functions
    //=========================================================

    /// Input: function parameters as declared by `FromViewProjectionFrustum`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `FromViewProjectionFrustum` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Frustum<T> FromViewProjectionFrustum(
        const Matrix4<T>& viewProjection) noexcept
    {
        return Frustum<T>::FromViewProjection(viewProjection);
    }

    /// Input: function parameters as declared by `Contains`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Contains` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Contains(
        const Frustum<T>& frustum,
        const Vector3<T>& point) noexcept
    {
        return frustum.ContainsPoint(point);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const Frustum<T>& frustum,
        const Sphere<T>& sphere) noexcept
    {
        return frustum.IntersectsSphere(sphere);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const Sphere<T>& sphere,
        const Frustum<T>& frustum) noexcept
    {
        return frustum.IntersectsSphere(sphere);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const Frustum<T>& frustum,
        const AABB<T>& box) noexcept
    {
        return frustum.IntersectsAABB(box);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const AABB<T>& box,
        const Frustum<T>& frustum) noexcept
    {
        return frustum.IntersectsAABB(box);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const Frustum<T>& frustum,
        const OBB<T>& box) noexcept
    {
        return frustum.IntersectsOBB(box);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const OBB<T>& box,
        const Frustum<T>& frustum) noexcept
    {
        return frustum.IntersectsOBB(box);
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const Frustum<T>& lhs,
        const Frustum<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(10)) noexcept
    {
        return
            NearlyEqual(lhs.Left, rhs.Left, epsilon) &&
            NearlyEqual(lhs.Right, rhs.Right, epsilon) &&
            NearlyEqual(lhs.Bottom, rhs.Bottom, epsilon) &&
            NearlyEqual(lhs.Top, rhs.Top, epsilon) &&
            NearlyEqual(lhs.Near, rhs.Near, epsilon) &&
            NearlyEqual(lhs.Far, rhs.Far, epsilon);
    }

    using Frustumf = Frustum<float>;
    using Frustumd = Frustum<double>;

    static_assert(std::is_trivially_copyable_v<Frustumf>);
    static_assert(std::is_standard_layout_v<Frustumf>);

} // namespace kairo::foundation::geometry
