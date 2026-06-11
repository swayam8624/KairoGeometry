module;

#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <type_traits>

export module Kairo.Foundation.Geometry.Ray;

import Kairo.Foundation.Math.Vector;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Ray
    //=========================================================

    /// A half-infinite 3D ray.
    ///
    /// Ray equation:
    ///     P(t) = Origin + Direction * t
    ///
    /// Convention:
    /// - t >= 0 is in front of the ray origin.
    /// - Direction is expected to be normalized for distance/intersection APIs.
    /// - Use NormalizedRay() or NormalizeDirection() if unsure.
    template<FloatingPoint T>
    struct Ray final
    {
        using ValueType = T;

        Vector3<T> Origin =
            Vector3<T>::Zero();

        Vector3<T> Direction =
            Vector3<T>::Forward();

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------
        /// Input: function parameters as declared by `FromOriginDirection`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromOriginDirection` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Ray FromOriginDirection(
            const Vector3<T>& origin,
            const Vector3<T>& direction) noexcept
        {
            return
            {
                origin,
                direction
            };
        }
        /// Input: function parameters as declared by `FromOriginTarget`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromOriginTarget` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static Ray FromOriginTarget(
            const Vector3<T>& origin,
            const Vector3<T>& target) noexcept
        {
            return
            {
                origin,
                SafeNormalize(
                    target - origin,
                    Vector3<T>::Forward())
            };
        }
        /// Input: function parameters as declared by `XAxis`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `XAxis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Ray XAxis() noexcept
        {
            return
            {
                Vector3<T>::Zero(),
                Vector3<T>::UnitX()
            };
        }
        /// Input: function parameters as declared by `YAxis`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `YAxis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Ray YAxis() noexcept
        {
            return
            {
                Vector3<T>::Zero(),
                Vector3<T>::UnitY()
            };
        }
        /// Input: function parameters as declared by `ZAxis`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ZAxis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Ray ZAxis() noexcept
        {
            return
            {
                Vector3<T>::Zero(),
                Vector3<T>::UnitZ()
            };
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Ray` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Ray() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Ray` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Ray(
            const Vector3<T>& origin,
            const Vector3<T>& direction) noexcept
            :
            Origin(origin),
            Direction(direction)
        {
        }

        //-----------------------------------------------------
        // Evaluation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `GetPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `GetPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> GetPoint(
            T distance) const noexcept
        {
            return
                Origin +
                Direction * distance;
        }
        /// Input: function parameters as declared by `operator()`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator()` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> operator()(
            T distance) const noexcept
        {
            return GetPoint(
                distance);
        }

        //-----------------------------------------------------
        // Mutation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `NormalizeDirection`.
        /// Output: none; mutates the object or output parameters described by the function.
        /// Task: provide the `NormalizeDirection` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        void NormalizeDirection() noexcept
        {
            Direction =
                SafeNormalize(
                    Direction,
                    Vector3<T>::Forward());
        }

        //-----------------------------------------------------
        // Validation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `IsDirectionNormalized`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsDirectionNormalized` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool IsDirectionNormalized(
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(10)) const noexcept
        {
            return NearlyEqual(
                Direction.LengthSquared(),
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
                Direction.LengthSquared() >
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
            const Ray&) const noexcept = default;
    };

    //=========================================================
    // Free Functions
    //=========================================================

    /// Input: function parameters as declared by `NormalizedRay`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NormalizedRay` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Ray<T> NormalizedRay(
        const Ray<T>& ray) noexcept
    {
        return
        {
            ray.Origin,
            SafeNormalize(
                ray.Direction,
                Vector3<T>::Forward())
        };
    }

    /// Input: function parameters as declared by `GetPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `GetPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> GetPoint(
        const Ray<T>& ray,
        T distance) noexcept
    {
        return ray.GetPoint(
            distance);
    }

    /// Input: function parameters as declared by `ProjectPointParameter`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ProjectPointParameter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T ProjectPointParameter(
        const Ray<T>& ray,
        const Vector3<T>& point) noexcept
    {
        const Vector3<T> originToPoint =
            point - ray.Origin;

        const T directionLengthSquared =
            ray.Direction.LengthSquared();

        if (directionLengthSquared <=
            std::numeric_limits<T>::epsilon())
        {
            return T(0);
        }

        return
            Dot(
                originToPoint,
                ray.Direction)
            / directionLengthSquared;
    }

    /// Input: function parameters as declared by `ClosestParameter`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestParameter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T ClosestParameter(
        const Ray<T>& ray,
        const Vector3<T>& point) noexcept
    {
        return Max(
            ProjectPointParameter(
                ray,
                point),
            T(0));
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPoint(
        const Ray<T>& ray,
        const Vector3<T>& point) noexcept
    {
        return ray.GetPoint(
            ClosestParameter(
                ray,
                point));
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquared(
        const Ray<T>& ray,
        const Vector3<T>& point) noexcept
    {
        return kairo::foundation::math::DistanceSquared(
            point,
            ClosestPoint(
                ray,
                point));
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Ray<T>& ray,
        const Vector3<T>& point) noexcept
    {
        return std::sqrt(
            DistanceSquared(
                ray,
                point));
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const Ray<T>& lhs,
        const Ray<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon()
            * T(10)) noexcept
    {
        return
            NearlyEqual(
                lhs.Origin,
                rhs.Origin,
                epsilon)
            &&
            NearlyEqual(
                lhs.Direction,
                rhs.Direction,
                epsilon);
    }

    //=========================================================
    // Aliases
    //=========================================================

    using Rayf = Ray<float>;
    using Rayd = Ray<double>;

    //=========================================================
    // Constants
    //=========================================================

    namespace constants
    {
        inline constexpr Rayf XAxisRayf =
            Rayf::XAxis();

        inline constexpr Rayf YAxisRayf =
            Rayf::YAxis();

        inline constexpr Rayf ZAxisRayf =
            Rayf::ZAxis();
    }

    //=========================================================
    // Compile-Time Validation
    //=========================================================

    static_assert(
        std::is_trivially_copyable_v<Rayf>);

    static_assert(
        std::is_standard_layout_v<Rayf>);

} // namespace kairo::foundation::geometry