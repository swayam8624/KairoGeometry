module;

#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <type_traits>

export module Kairo.Foundation.Geometry.Segment;

import Kairo.Foundation.Math.Vector;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Segment Closest Result
    //=========================================================

    /// Result of a closest-points query between two finite 3D segments.
    ///
    /// Parameters are normalized to [0, 1], where 0 selects Start and 1 selects
    /// End on the corresponding segment.
    template<FloatingPoint T>
    struct SegmentClosestResult final
    {
        T ParameterA = T(0);
        T ParameterB = T(0);
        Vector3<T> PointA = Vector3<T>::Zero();
        Vector3<T> PointB = Vector3<T>::Zero();
        T DistanceSquared = T(0);
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(const SegmentClosestResult&) const noexcept = default;
    };

    //=========================================================
    // Segment
    //=========================================================

    /// A finite 3D line segment represented by two endpoints.
    ///
    /// Convention:
    /// - `Start == End` is valid and represents a point segment.
    /// - Parameters are normalized: `GetPoint(0) == Start`,
    ///   `GetPoint(1) == End`.
    /// - Segment is the primitive backing capsule/capsule distance, character
    ///   controller contacts, constraint anchors, and swept shape queries.
    template<FloatingPoint T>
    struct Segment final
    {
        using ValueType = T;

        Vector3<T> Start = Vector3<T>::Zero();
        Vector3<T> End = Vector3<T>::UnitX();

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------

        /// Input: two endpoints.
        /// Output: segment spanning start to end.
        /// Task: construct a finite segment without normalizing or mutating the
        /// supplied endpoints.
        [[nodiscard]]
        static constexpr Segment FromEndpoints(
            const Vector3<T>& start,
            const Vector3<T>& end) noexcept
        {
            return { start, end };
        }

        /// Input: one point.
        /// Output: zero-length segment at that point.
        /// Task: represent degenerate segment cases explicitly.
        [[nodiscard]]
        static constexpr Segment FromPoint(
            const Vector3<T>& point) noexcept
        {
            return { point, point };
        }

        /// Input: segment center, direction, and full length.
        /// Output: finite segment centered on `center`.
        /// Task: construct tool/debug segments from direction-length data.
        [[nodiscard]]
        static Segment FromCenterDirectionLength(
            const Vector3<T>& center,
            const Vector3<T>& direction,
            T length) noexcept
        {
            const Vector3<T> axis =
                SafeNormalize(
                    direction,
                    Vector3<T>::UnitX());

            const Vector3<T> half =
                axis * (Max(length, T(0)) * T(0.5));

            return
            {
                center - half,
                center + half
            };
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Segment` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Segment() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Segment` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Segment(
            const Vector3<T>& start,
            const Vector3<T>& end) noexcept
            : Start(start)
            , End(end)
        {
        }

        //-----------------------------------------------------
        // Basic Properties
        //-----------------------------------------------------

        /// Input: none.
        /// Output: vector from Start to End.
        /// Task: expose the segment axis for projection and distance math.
        [[nodiscard]]
        constexpr Vector3<T> Axis() const noexcept
        {
            return End - Start;
        }

        /// Input: none.
        /// Output: squared segment length.
        /// Task: avoid square roots for comparisons and projection denominators.
        [[nodiscard]]
        constexpr T LengthSquared() const noexcept
        {
            return Axis().LengthSquared();
        }

        /// Input: none.
        /// Output: segment length.
        /// Task: report physical length for tools, contacts, and constraints.
        [[nodiscard]]
        T Length() const noexcept
        {
            return Axis().Length();
        }

        /// Input: none.
        /// Output: normalized axis, or +X for a point segment.
        /// Task: provide stable direction even for degenerate segments.
        [[nodiscard]]
        Vector3<T> Direction() const noexcept
        {
            return SafeNormalize(
                Axis(),
                Vector3<T>::UnitX());
        }

        /// Input: none.
        /// Output: midpoint.
        /// Task: support broad-phase and debug visualization code.
        [[nodiscard]]
        constexpr Vector3<T> Center() const noexcept
        {
            return (Start + End) * T(0.5);
        }

        //-----------------------------------------------------
        // Evaluation
        //-----------------------------------------------------

        /// Input: normalized parameter where 0 is Start and 1 is End.
        /// Output: point on the infinite parameterized segment line.
        /// Task: evaluate segment positions for interpolation and closest
        /// point math. Use ClosestParameter() when clamping is required.
        [[nodiscard]]
        constexpr Vector3<T> GetPoint(T parameter) const noexcept
        {
            return Start + Axis() * parameter;
        }

        /// Input: normalized parameter.
        /// Output: GetPoint(parameter).
        /// Task: function-call convenience for generic curve-style code.
        [[nodiscard]]
        constexpr Vector3<T> operator()(T parameter) const noexcept
        {
            return GetPoint(parameter);
        }

        //-----------------------------------------------------
        // Closest / Distance
        //-----------------------------------------------------

        /// Input: point.
        /// Output: unclamped projection parameter on the segment line.
        /// Task: expose projection for callers that need to know whether a point
        /// lies before Start or after End.
        [[nodiscard]]
        T ProjectPointParameter(const Vector3<T>& point) const noexcept
        {
            const Vector3<T> axis =
                Axis();

            const T lengthSquared =
                axis.LengthSquared();

            if (lengthSquared <= std::numeric_limits<T>::epsilon())
            {
                return T(0);
            }

            return Dot(point - Start, axis) / lengthSquared;
        }

        /// Input: point.
        /// Output: clamped parameter in [0, 1].
        /// Task: compute nearest finite-segment parameter.
        [[nodiscard]]
        T ClosestParameter(const Vector3<T>& point) const noexcept
        {
            return Clamp(
                ProjectPointParameter(point),
                T(0),
                T(1));
        }

        /// Input: point.
        /// Output: closest point on the finite segment.
        /// Task: fundamental point/segment query used by capsule and distance
        /// systems.
        [[nodiscard]]
        Vector3<T> ClosestPoint(const Vector3<T>& point) const noexcept
        {
            return GetPoint(
                ClosestParameter(point));
        }

        /// Input: point.
        /// Output: squared distance from point to finite segment.
        /// Task: avoid square roots for broad-phase and contact tests.
        [[nodiscard]]
        T DistanceSquaredToPoint(const Vector3<T>& point) const noexcept
        {
            return kairo::foundation::math::DistanceSquared(
                point,
                ClosestPoint(point));
        }

        /// Input: point.
        /// Output: distance from point to finite segment.
        /// Task: readable distance helper for gameplay/tooling code.
        [[nodiscard]]
        T DistanceToPoint(const Vector3<T>& point) const noexcept
        {
            return std::sqrt(
                DistanceSquaredToPoint(point));
        }

        //-----------------------------------------------------
        // Validation
        //-----------------------------------------------------

        /// Input: tolerance.
        /// Output: true if Start and End are effectively the same point.
        /// Task: detect point-segment degeneracy without requiring exact
        /// floating-point equality.
        [[nodiscard]]
        bool IsPoint(
            T epsilon =
                std::numeric_limits<T>::epsilon() * T(10)) const noexcept
        {
            return LengthSquared() <= epsilon * epsilon;
        }
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(const Segment&) const noexcept = default;
    };

    //=========================================================
    // Free Functions
    //=========================================================

    /// Input: function parameters as declared by `Axis`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Axis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Axis(const Segment<T>& segment) noexcept
    {
        return segment.Axis();
    }

    /// Input: function parameters as declared by `Direction`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Direction` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> Direction(const Segment<T>& segment) noexcept
    {
        return segment.Direction();
    }

    /// Input: function parameters as declared by `Center`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Center` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Center(const Segment<T>& segment) noexcept
    {
        return segment.Center();
    }

    /// Input: function parameters as declared by `Length`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Length` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Length(const Segment<T>& segment) noexcept
    {
        return segment.Length();
    }

    /// Input: function parameters as declared by `LengthSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `LengthSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T LengthSquared(const Segment<T>& segment) noexcept
    {
        return segment.LengthSquared();
    }

    /// Input: function parameters as declared by `ClosestParameter`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestParameter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T ClosestParameter(
        const Segment<T>& segment,
        const Vector3<T>& point) noexcept
    {
        return segment.ClosestParameter(point);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPoint(
        const Segment<T>& segment,
        const Vector3<T>& point) noexcept
    {
        return segment.ClosestPoint(point);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPoint(
        const Vector3<T>& point,
        const Segment<T>& segment) noexcept
    {
        return segment.ClosestPoint(point);
    }

    /// Input: two finite segments.
    /// Output: closest normalized parameters and points on both segments.
    /// Task: robust segment/segment closest query for capsule-capsule tests,
    /// constraints, and contact generation. Degenerate point-segment cases are
    /// handled explicitly.
    template<FloatingPoint T>
    [[nodiscard]]
    SegmentClosestResult<T> ClosestPoints(
        const Segment<T>& lhs,
        const Segment<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(10)) noexcept
    {
        const Vector3<T> d1 = lhs.End - lhs.Start;
        const Vector3<T> d2 = rhs.End - rhs.Start;
        const Vector3<T> r = lhs.Start - rhs.Start;

        const T a = Dot(d1, d1);
        const T e = Dot(d2, d2);
        const T f = Dot(d2, r);

        T s = T(0);
        T t = T(0);

        if (a <= epsilon && e <= epsilon)
        {
            const T distanceSquared =
                kairo::foundation::math::DistanceSquared(
                    lhs.Start,
                    rhs.Start);

            return
            {
                T(0),
                T(0),
                lhs.Start,
                rhs.Start,
                distanceSquared
            };
        }

        if (a <= epsilon)
        {
            s = T(0);
            t = Clamp(f / e, T(0), T(1));
        }
        else
        {
            const T c = Dot(d1, r);

            if (e <= epsilon)
            {
                t = T(0);
                s = Clamp(-c / a, T(0), T(1));
            }
            else
            {
                const T b = Dot(d1, d2);
                const T denominator = a * e - b * b;

                if (std::abs(denominator) > epsilon)
                {
                    s = Clamp(
                        (b * f - c * e) / denominator,
                        T(0),
                        T(1));
                }
                else
                {
                    s = T(0);
                }

                t = (b * s + f) / e;

                if (t < T(0))
                {
                    t = T(0);
                    s = Clamp(-c / a, T(0), T(1));
                }
                else if (t > T(1))
                {
                    t = T(1);
                    s = Clamp((b - c) / a, T(0), T(1));
                }
            }
        }

        const Vector3<T> pointA =
            lhs.Start + d1 * s;

        const Vector3<T> pointB =
            rhs.Start + d2 * t;

        return
        {
            s,
            t,
            pointA,
            pointB,
            kairo::foundation::math::DistanceSquared(pointA, pointB)
        };
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquared(
        const Segment<T>& lhs,
        const Segment<T>& rhs) noexcept
    {
        return ClosestPoints(lhs, rhs).DistanceSquared;
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Segment<T>& lhs,
        const Segment<T>& rhs) noexcept
    {
        return std::sqrt(
            DistanceSquared(lhs, rhs));
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquared(
        const Segment<T>& segment,
        const Vector3<T>& point) noexcept
    {
        return segment.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquared(
        const Vector3<T>& point,
        const Segment<T>& segment) noexcept
    {
        return segment.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Segment<T>& segment,
        const Vector3<T>& point) noexcept
    {
        return segment.DistanceToPoint(point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Vector3<T>& point,
        const Segment<T>& segment) noexcept
    {
        return segment.DistanceToPoint(point);
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const Segment<T>& lhs,
        const Segment<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon() * T(10)) noexcept
    {
        return
            NearlyEqual(lhs.Start, rhs.Start, epsilon) &&
            NearlyEqual(lhs.End, rhs.End, epsilon);
    }

    using Segmentf = Segment<float>;
    using Segmentd = Segment<double>;
    using SegmentClosestResultf = SegmentClosestResult<float>;
    using SegmentClosestResultd = SegmentClosestResult<double>;

    namespace constants
    {
        inline constexpr Segmentf UnitXSegmentf =
            Segmentf::FromEndpoints(
                Vec3f::Zero(),
                Vec3f::UnitX());
    }

    static_assert(std::is_trivially_copyable_v<Segmentf>);
    static_assert(std::is_standard_layout_v<Segmentf>);
    static_assert(std::is_trivially_copyable_v<SegmentClosestResultf>);

} // namespace kairo::foundation::geometry
