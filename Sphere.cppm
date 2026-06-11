module;

#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <numbers>
#include <type_traits>

export module Kairo.Foundation.Geometry.Sphere;

import Kairo.Foundation.Math.Vector;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Sphere
    //=========================================================

    /// A 3D sphere represented by:
    ///
    ///     Center + Radius
    ///
    /// Convention:
    /// - Radius should be non-negative.
    /// - Radius = 0 represents a point sphere.
    /// - Negative radius is considered invalid input.
    template<FloatingPoint T>
    struct Sphere final
    {
        using ValueType = T;

        Vector3<T> Center =
            Vector3<T>::Zero();

        T Radius =
            T(1);

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Unit`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Unit` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Sphere Unit() noexcept
        {
            return
            {
                Vector3<T>::Zero(),
                T(1)
            };
        }
        /// Input: function parameters as declared by `FromCenterRadius`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromCenterRadius` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Sphere FromCenterRadius(
            const Vector3<T>& center,
            T radius) noexcept
        {
            return
            {
                center,
                radius
            };
        }
        /// Input: function parameters as declared by `FromPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Sphere FromPoint(
            const Vector3<T>& point) noexcept
        {
            return
            {
                point,
                T(0)
            };
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Sphere` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Sphere() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Sphere` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Sphere(
            const Vector3<T>& center,
            T radius) noexcept
            :
            Center(center),
            Radius(radius)
        {
        }

        //-----------------------------------------------------
        // Basic Properties
        //-----------------------------------------------------
        /// Input: function parameters as declared by `RadiusSquared`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `RadiusSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T RadiusSquared() const noexcept
        {
            return Radius * Radius;
        }
        /// Input: function parameters as declared by `Diameter`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Diameter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T Diameter() const noexcept
        {
            return Radius * T(2);
        }
        /// Input: function parameters as declared by `SurfaceArea`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `SurfaceArea` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T SurfaceArea() const noexcept
        {
            return
                T(4) *
                std::numbers::pi_v<T> *
                Radius *
                Radius;
        }
        /// Input: function parameters as declared by `Volume`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Volume` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T Volume() const noexcept
        {
            return
                (T(4) / T(3)) *
                std::numbers::pi_v<T> *
                Radius *
                Radius *
                Radius;
        }

        //-----------------------------------------------------
        // Containment
        //-----------------------------------------------------
        /// Input: function parameters as declared by `ContainsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ContainsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool ContainsPoint(
            const Vector3<T>& point) const noexcept
        {
            return
                DistanceSquared(
                    Center,
                    point)
                <= RadiusSquared();
        }
        /// Input: function parameters as declared by `ContainsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ContainsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool ContainsPoint(
            const Vector3<T>& point,
            T epsilon) const noexcept
        {
            const T expandedRadius =
                Radius + epsilon;

            return
                DistanceSquared(
                    Center,
                    point)
                <= expandedRadius * expandedRadius;
        }
        /// Input: function parameters as declared by `ContainsSphere`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ContainsSphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool ContainsSphere(
            const Sphere& sphere) const noexcept
        {
            const T centerDistance =
                kairo::foundation::math::Distance(
                    Center,
                    sphere.Center);

            return
                centerDistance + sphere.Radius
                <= Radius;
        }

        //-----------------------------------------------------
        // Closest / Distance
        //-----------------------------------------------------
        /// Input: function parameters as declared by `ClosestPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        Vector3<T> ClosestPoint(
            const Vector3<T>& point) const noexcept
        {
            const Vector3<T> centerToPoint =
                point - Center;

            const T lengthSquared =
                centerToPoint.LengthSquared();

            if (lengthSquared <=
                std::numeric_limits<T>::epsilon())
            {
                return
                    Center +
                    Vector3<T>::UnitX() *
                    Radius;
            }

            return
                Center +
                SafeNormalize(
                    centerToPoint,
                    Vector3<T>::UnitX())
                * Radius;
        }
        /// Input: function parameters as declared by `DistanceToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `DistanceToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T DistanceToPoint(
            const Vector3<T>& point) const noexcept
        {
            return Max(
                kairo::foundation::math::Distance(
                    Center,
                    point)
                - Radius,
                T(0));
        }
        /// Input: function parameters as declared by `SignedDistanceToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `SignedDistanceToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T SignedDistanceToPoint(
            const Vector3<T>& point) const noexcept
        {
            return
                kairo::foundation::math::Distance(
                    Center,
                    point)
                - Radius;
        }

        //-----------------------------------------------------
        // Expansion / Mutation
        //-----------------------------------------------------

        /// Input: function parameters as declared by `Expand`.
        /// Output: none; mutates this geometry value in place.
        /// Task: update the value while preserving documented geometry invariants.
        constexpr void Expand(
            T amount) noexcept
        {
            Radius += amount;

            if (Radius < T(0))
            {
                Radius = T(0);
            }
        }
        /// Input: function parameters as declared by `ExpandToInclude`.
        /// Output: none; mutates the object or output parameters described by the function.
        /// Task: provide the `ExpandToInclude` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        void ExpandToInclude(
            const Vector3<T>& point) noexcept
        {
            const T distanceToPoint =
                kairo::foundation::math::Distance(
                    Center,
                    point);

            if (distanceToPoint > Radius)
            {
                Radius = distanceToPoint;
            }
        }

        //-----------------------------------------------------
        // Validation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `IsValid`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsValid` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool IsValid() const noexcept
        {
            return Radius >= T(0);
        }
        /// Input: function parameters as declared by `IsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool IsPoint() const noexcept
        {
            return Radius == T(0);
        }

        //-----------------------------------------------------
        // Equality
        //-----------------------------------------------------
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(
            const Sphere&) const noexcept = default;
    };

    //=========================================================
    // Free Functions
    //=========================================================

    /// Input: function parameters as declared by `Contains`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Contains` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Contains(
        const Sphere<T>& sphere,
        const Vector3<T>& point) noexcept
    {
        return sphere.ContainsPoint(
            point);
    }

    /// Input: function parameters as declared by `Contains`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Contains` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Contains(
        const Sphere<T>& outer,
        const Sphere<T>& inner) noexcept
    {
        return outer.ContainsSphere(
            inner);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPoint(
        const Sphere<T>& sphere,
        const Vector3<T>& point) noexcept
    {
        return sphere.ClosestPoint(
            point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Sphere<T>& sphere,
        const Vector3<T>& point) noexcept
    {
        return sphere.DistanceToPoint(
            point);
    }

    /// Input: function parameters as declared by `SignedDistance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SignedDistance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T SignedDistance(
        const Sphere<T>& sphere,
        const Vector3<T>& point) noexcept
    {
        return sphere.SignedDistanceToPoint(
            point);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
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

    /// Input: function parameters as declared by `Merge`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Merge` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Sphere<T> Merge(
        const Sphere<T>& lhs,
        const Sphere<T>& rhs) noexcept
    {
        const Vector3<T> centerDifference =
            rhs.Center - lhs.Center;

        const T distance =
            centerDifference.Length();

        if (lhs.Radius >= distance + rhs.Radius)
        {
            return lhs;
        }

        if (rhs.Radius >= distance + lhs.Radius)
        {
            return rhs;
        }

        const T newRadius =
            (distance + lhs.Radius + rhs.Radius)
            * T(0.5);

        Vector3<T> newCenter =
            lhs.Center;

        if (distance >
            std::numeric_limits<T>::epsilon())
        {
            newCenter +=
                centerDifference *
                ((newRadius - lhs.Radius) / distance);
        }

        return
        {
            newCenter,
            newRadius
        };
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const Sphere<T>& lhs,
        const Sphere<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon()
            * T(10)) noexcept
    {
        return
            NearlyEqual(
                lhs.Center,
                rhs.Center,
                epsilon)
            &&
            NearlyEqual(
                lhs.Radius,
                rhs.Radius,
                epsilon);
    }

    //=========================================================
    // Aliases
    //=========================================================

    using Spheref = Sphere<float>;
    using Sphered = Sphere<double>;

    //=========================================================
    // Constants
    //=========================================================

    namespace constants
    {
        inline constexpr Spheref UnitSpheref =
            Spheref::Unit();
    }

    //=========================================================
    // Compile-Time Validation
    //=========================================================

    static_assert(
        std::is_trivially_copyable_v<Spheref>);

    static_assert(
        std::is_standard_layout_v<Spheref>);

} // namespace kairo::foundation::geometry
