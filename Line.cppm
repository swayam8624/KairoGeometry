module;

#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <type_traits>

export module Kairo.Foundation.Geometry.Line;

import Kairo.Foundation.Math.Vector;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Line
    //=========================================================

    /// An infinite 3D line represented by a point and a direction.
    ///
    /// Convention:
    /// - Direction should be non-zero. A normalized direction is preferred for
    ///   stable parameter units, but closest-point math handles non-unit input.
    /// - Unlike Ray and Segment, the line extends in both parameter directions.
    template<FloatingPoint T>
    struct Line final
    {
        using ValueType = T;

        Vector3<T> Point = Vector3<T>::Zero();
        Vector3<T> Direction = Vector3<T>::UnitX();

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------

        /// Input: point and direction.
        /// Output: line through point along direction.
        /// Task: preserve caller-provided direction without implicit
        /// normalization.
        [[nodiscard]]
        static constexpr Line FromPointDirection(
            const Vector3<T>& point,
            const Vector3<T>& direction) noexcept
        {
            return { point, direction };
        }

        /// Input: two points.
        /// Output: line passing through both points.
        /// Task: construct analytic/debug line from endpoints.
        [[nodiscard]]
        static constexpr Line FromPoints(
            const Vector3<T>& a,
            const Vector3<T>& b) noexcept
        {
            return { a, b - a };
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Line` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Line() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Line` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Line(
            const Vector3<T>& point,
            const Vector3<T>& direction) noexcept
            : Point(point)
            , Direction(direction)
        {
        }

        //-----------------------------------------------------
        // Evaluation
        //-----------------------------------------------------

        /// Input: parameter in direction units.
        /// Output: point on the infinite line.
        /// Task: evaluate analytic line position.
        [[nodiscard]]
        constexpr Vector3<T> GetPoint(T parameter) const noexcept
        {
            return Point + Direction * parameter;
        }
        /// Input: function parameters as declared by `operator()`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator()` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> operator()(T parameter) const noexcept
        {
            return GetPoint(parameter);
        }

        //-----------------------------------------------------
        // Closest / Distance
        //-----------------------------------------------------

        /// Input: point.
        /// Output: projection parameter on the infinite line.
        /// Task: compute closest-line coordinate while safely handling a
        /// degenerate direction.
        [[nodiscard]]
        T ClosestParameter(const Vector3<T>& point) const noexcept
        {
            const T lengthSquared =
                Direction.LengthSquared();

            if (lengthSquared <= std::numeric_limits<T>::epsilon())
            {
                return T(0);
            }

            return Dot(point - Point, Direction) / lengthSquared;
        }

        /// Input: point.
        /// Output: closest point on the infinite line.
        /// Task: support analytic geometry and debug picking queries.
        [[nodiscard]]
        Vector3<T> ClosestPoint(const Vector3<T>& point) const noexcept
        {
            return GetPoint(
                ClosestParameter(point));
        }
        /// Input: function parameters as declared by `DistanceSquaredToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `DistanceSquaredToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T DistanceSquaredToPoint(const Vector3<T>& point) const noexcept
        {
            return kairo::foundation::math::DistanceSquared(
                point,
                ClosestPoint(point));
        }
        /// Input: function parameters as declared by `DistanceToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `DistanceToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T DistanceToPoint(const Vector3<T>& point) const noexcept
        {
            return std::sqrt(
                DistanceSquaredToPoint(point));
        }

        //-----------------------------------------------------
        // Mutation / Validation
        //-----------------------------------------------------

        /// Input: none.
        /// Output: none.
        /// Task: normalize direction, using +X as fallback for degenerate input.
        void NormalizeDirection() noexcept
        {
            Direction =
                SafeNormalize(
                    Direction,
                    Vector3<T>::UnitX());
        }
        /// Input: function parameters as declared by `IsDirectionNormalized`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsDirectionNormalized` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool IsDirectionNormalized(
            T epsilon =
                std::numeric_limits<T>::epsilon() * T(10)) const noexcept
        {
            return NearlyEqual(
                Direction.LengthSquared(),
                T(1),
                epsilon);
        }
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool IsValid(
            T epsilon =
                std::numeric_limits<T>::epsilon() * T(10)) const noexcept
        {
            return Direction.LengthSquared() > epsilon;
        }
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(const Line&) const noexcept = default;
    };

    //=========================================================
    // Free Functions
    //=========================================================

    /// Input: function parameters as declared by `GetPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `GetPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> GetPoint(
        const Line<T>& line,
        T parameter) noexcept
    {
        return line.GetPoint(parameter);
    }

    /// Input: function parameters as declared by `ClosestParameter`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestParameter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T ClosestParameter(
        const Line<T>& line,
        const Vector3<T>& point) noexcept
    {
        return line.ClosestParameter(point);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPoint(
        const Line<T>& line,
        const Vector3<T>& point) noexcept
    {
        return line.ClosestPoint(point);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPoint(
        const Vector3<T>& point,
        const Line<T>& line) noexcept
    {
        return line.ClosestPoint(point);
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquared(
        const Line<T>& line,
        const Vector3<T>& point) noexcept
    {
        return line.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquared(
        const Vector3<T>& point,
        const Line<T>& line) noexcept
    {
        return line.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Line<T>& line,
        const Vector3<T>& point) noexcept
    {
        return line.DistanceToPoint(point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Vector3<T>& point,
        const Line<T>& line) noexcept
    {
        return line.DistanceToPoint(point);
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const Line<T>& lhs,
        const Line<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(10)) noexcept
    {
        return
            NearlyEqual(lhs.Point, rhs.Point, epsilon) &&
            NearlyEqual(lhs.Direction, rhs.Direction, epsilon);
    }

    using Linef = Line<float>;
    using Lined = Line<double>;

    namespace constants
    {
        inline constexpr Linef XAxisLinef =
            Linef::FromPointDirection(
                Vec3f::Zero(),
                Vec3f::UnitX());
    }

    static_assert(std::is_trivially_copyable_v<Linef>);
    static_assert(std::is_standard_layout_v<Linef>);

} // namespace kairo::foundation::geometry
