module;

#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <type_traits>

export module Kairo.Foundation.Geometry.Plane;

import Kairo.Foundation.Math.Vector;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Plane
    //=========================================================

    /// A 3D plane represented by:
    ///
    ///     Dot(Normal, point) + Distance = 0
    ///
    /// Convention:
    /// - Normal should be normalized for distance/projection APIs.
    /// - Positive signed distance means the point lies in the direction of Normal.
    template<FloatingPoint T>
    struct Plane final
    {
        using ValueType = T;

        Vector3<T> Normal =
            Vector3<T>::Up();

        T Distance =
            T(0);

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------
        /// Input: function parameters as declared by `FromNormalDistance`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromNormalDistance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Plane FromNormalDistance(
            const Vector3<T>& normal,
            T distance) noexcept
        {
            return
            {
                normal,
                distance
            };
        }
        /// Input: function parameters as declared by `FromPointNormal`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromPointNormal` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static Plane FromPointNormal(
            const Vector3<T>& point,
            const Vector3<T>& normal) noexcept
        {
            const Vector3<T> normalizedNormal =
                SafeNormalize(
                    normal,
                    Vector3<T>::Up());

            return
            {
                normalizedNormal,
                -Dot(
                    normalizedNormal,
                    point)
            };
        }
        /// Input: function parameters as declared by `FromPoints`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromPoints` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static Plane FromPoints(
            const Vector3<T>& a,
            const Vector3<T>& b,
            const Vector3<T>& c) noexcept
        {
            const Vector3<T> normal =
                SafeNormalize(
                    Cross(
                        b - a,
                        c - a),
                    Vector3<T>::Up());

            return FromPointNormal(
                a,
                normal);
        }
        /// Input: function parameters as declared by `XY`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `XY` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Plane XY() noexcept
        {
            return
            {
                Vector3<T>::UnitZ(),
                T(0)
            };
        }
        /// Input: function parameters as declared by `XZ`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `XZ` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Plane XZ() noexcept
        {
            return
            {
                Vector3<T>::UnitY(),
                T(0)
            };
        }
        /// Input: function parameters as declared by `YZ`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `YZ` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Plane YZ() noexcept
        {
            return
            {
                Vector3<T>::UnitX(),
                T(0)
            };
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Plane` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Plane() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Plane` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Plane(
            const Vector3<T>& normal,
            T distance) noexcept
            :
            Normal(normal),
            Distance(distance)
        {
        }

        //-----------------------------------------------------
        // Signed Distance
        //-----------------------------------------------------
        /// Input: function parameters as declared by `SignedDistanceToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `SignedDistanceToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T SignedDistanceToPoint(
            const Vector3<T>& point) const noexcept
        {
            return
                Dot(
                    Normal,
                    point)
                + Distance;
        }
        /// Input: function parameters as declared by `DistanceToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `DistanceToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T DistanceToPoint(
            const Vector3<T>& point) const noexcept
        {
            return std::abs(
                SignedDistanceToPoint(
                    point));
        }

        //-----------------------------------------------------
        // Projection / Closest Point
        //-----------------------------------------------------
        /// Input: function parameters as declared by `ProjectPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ProjectPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> ProjectPoint(
            const Vector3<T>& point) const noexcept
        {
            return
                point -
                Normal *
                SignedDistanceToPoint(point);
        }
        /// Input: function parameters as declared by `ClosestPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> ClosestPoint(
            const Vector3<T>& point) const noexcept
        {
            return ProjectPoint(point);
        }

        //-----------------------------------------------------
        // Reflection
        //-----------------------------------------------------
        /// Input: function parameters as declared by `ReflectPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ReflectPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> ReflectPoint(
            const Vector3<T>& point) const noexcept
        {
            return
                point -
                Normal *
                (T(2) * SignedDistanceToPoint(point));
        }
        /// Input: function parameters as declared by `ReflectVector`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ReflectVector` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> ReflectVector(
            const Vector3<T>& vector) const noexcept
        {
            return kairo::foundation::math::Reflect(
                vector,
                Normal);
        }

        //-----------------------------------------------------
        // Mutation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Normalize`.
        /// Output: none; mutates the object or output parameters described by the function.
        /// Task: provide the `Normalize` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        void Normalize() noexcept
        {
            const T length =
                Normal.Length();

            if (length <=
                std::numeric_limits<T>::epsilon())
            {
                Normal =
                    Vector3<T>::Up();

                Distance =
                    T(0);

                return;
            }

            const T inverseLength =
                T(1) / length;

            Normal *= inverseLength;
            Distance *= inverseLength;
        }

        //-----------------------------------------------------
        // Validation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `IsNormalNormalized`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsNormalNormalized` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool IsNormalNormalized(
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(10)) const noexcept
        {
            return NearlyEqual(
                Normal.LengthSquared(),
                T(1),
                epsilon);
        }
        /// Input: function parameters as declared by `IsValid`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsValid` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool IsValid(
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(10)) const noexcept
        {
            return
                Normal.LengthSquared() >
                epsilon;
        }

        //-----------------------------------------------------
        // Equality
        //-----------------------------------------------------
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(
            const Plane&) const noexcept = default;
    };

    //=========================================================
    // Free Functions
    //=========================================================

    /// Input: function parameters as declared by `NormalizedPlane`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NormalizedPlane` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Plane<T> NormalizedPlane(
        const Plane<T>& plane) noexcept
    {
        Plane<T> result =
            plane;

        result.Normalize();

        return result;
    }

    /// Input: function parameters as declared by `SignedDistance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SignedDistance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T SignedDistance(
        const Plane<T>& plane,
        const Vector3<T>& point) noexcept
    {
        return plane.SignedDistanceToPoint(
            point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Plane<T>& plane,
        const Vector3<T>& point) noexcept
    {
        return plane.DistanceToPoint(
            point);
    }

    /// Input: function parameters as declared by `ProjectPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ProjectPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> ProjectPoint(
        const Plane<T>& plane,
        const Vector3<T>& point) noexcept
    {
        return plane.ProjectPoint(
            point);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> ClosestPoint(
        const Plane<T>& plane,
        const Vector3<T>& point) noexcept
    {
        return plane.ClosestPoint(
            point);
    }

    /// Input: function parameters as declared by `ReflectPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ReflectPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> ReflectPoint(
        const Plane<T>& plane,
        const Vector3<T>& point) noexcept
    {
        return plane.ReflectPoint(
            point);
    }

    /// Input: function parameters as declared by `ReflectVector`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ReflectVector` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> ReflectVector(
        const Plane<T>& plane,
        const Vector3<T>& vector) noexcept
    {
        return plane.ReflectVector(
            vector);
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const Plane<T>& lhs,
        const Plane<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon()
            * T(10)) noexcept
    {
        return
            NearlyEqual(
                lhs.Normal,
                rhs.Normal,
                epsilon)
            &&
            NearlyEqual(
                lhs.Distance,
                rhs.Distance,
                epsilon);
    }

    //=========================================================
    // Aliases
    //=========================================================

    using Planef = Plane<float>;
    using Planed = Plane<double>;

    //=========================================================
    // Constants
    //=========================================================

    namespace constants
    {
        inline constexpr Planef XYPlanef =
            Planef::XY();

        inline constexpr Planef XZPlanef =
            Planef::XZ();

        inline constexpr Planef YZPlanef =
            Planef::YZ();
    }

    //=========================================================
    // Compile-Time Validation
    //=========================================================

    static_assert(
        std::is_trivially_copyable_v<Planef>);

    static_assert(
        std::is_standard_layout_v<Planef>);

} // namespace kairo::foundation::geometry