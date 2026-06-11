module;

#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <numbers>
#include <type_traits>

export module Kairo.Foundation.Geometry.Capsule;

import Kairo.Foundation.Math.Vector;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Capsule
    //=========================================================

    /// A 3D capsule represented by a line segment and a radius.
    ///
    /// Shape:
    /// - A capsule is the swept volume of a sphere moving from Start to End.
    ///
    /// Convention:
    /// - Radius should be non-negative.
    /// - Start == End with Radius > 0 behaves like a sphere.
    /// - Start == End with Radius == 0 behaves like a point.
    template<FloatingPoint T>
    struct Capsule final
    {
        using ValueType = T;

        Vector3<T> Start =
            Vector3<T>::Zero();

        Vector3<T> End =
            Vector3<T>::UnitY();

        T Radius =
            T(0.5);

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Unit`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Unit` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Capsule Unit() noexcept
        {
            return
            {
                Vector3<T>
                {
                    T(0),
                    T(-0.5),
                    T(0)
                },

                Vector3<T>
                {
                    T(0),
                    T(0.5),
                    T(0)
                },

                T(0.5)
            };
        }
        /// Input: function parameters as declared by `FromEndpointsRadius`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromEndpointsRadius` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Capsule FromEndpointsRadius(
            const Vector3<T>& start,
            const Vector3<T>& end,
            T radius) noexcept
        {
            return
            {
                start,
                end,
                radius
            };
        }
        /// Input: function parameters as declared by `FromCenterHeightRadius`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromCenterHeightRadius` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Capsule FromCenterHeightRadius(
            const Vector3<T>& center,
            T height,
            T radius) noexcept
        {
            const T halfSegmentLength =
                Max(
                    (height * T(0.5)) - radius,
                    T(0));

            return
            {
                center - Vector3<T>::UnitY() * halfSegmentLength,
                center + Vector3<T>::UnitY() * halfSegmentLength,
                radius
            };
        }
        /// Input: function parameters as declared by `FromPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Capsule FromPoint(
            const Vector3<T>& point) noexcept
        {
            return
            {
                point,
                point,
                T(0)
            };
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Capsule` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Capsule() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Capsule` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Capsule(
            const Vector3<T>& start,
            const Vector3<T>& end,
            T radius) noexcept
            :
            Start(start),
            End(end),
            Radius(radius)
        {
        }

        //-----------------------------------------------------
        // Basic Properties
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Axis`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Axis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> Axis() const noexcept
        {
            return End - Start;
        }
        /// Input: function parameters as declared by `AxisLengthSquared`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `AxisLengthSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T AxisLengthSquared() const noexcept
        {
            return Axis().LengthSquared();
        }
        /// Input: function parameters as declared by `AxisLength`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `AxisLength` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T AxisLength() const noexcept
        {
            return Axis().Length();
        }
        /// Input: function parameters as declared by `Direction`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Direction` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        Vector3<T> Direction() const noexcept
        {
            return SafeNormalize(
                Axis(),
                Vector3<T>::UnitY());
        }
        /// Input: function parameters as declared by `Center`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Center` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> Center() const noexcept
        {
            return
                (Start + End) *
                T(0.5);
        }
        /// Input: function parameters as declared by `Height`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Height` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T Height() const noexcept
        {
            return
                AxisLength() +
                Radius * T(2);
        }
        /// Input: function parameters as declared by `Diameter`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Diameter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T Diameter() const noexcept
        {
            return Radius * T(2);
        }
        /// Input: function parameters as declared by `RadiusSquared`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `RadiusSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T RadiusSquared() const noexcept
        {
            return Radius * Radius;
        }

        //-----------------------------------------------------
        // Surface / Volume
        //-----------------------------------------------------
        /// Input: function parameters as declared by `SurfaceArea`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `SurfaceArea` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T SurfaceArea() const noexcept
        {
            // Capsule surface area:
            // cylinder side area + sphere area
            //
            // A = 2*pi*r*h + 4*pi*r^2
            //
            // where h is the segment length, not total capsule height.
            const T segmentLength =
                AxisLength();

            return
                T(2) *
                std::numbers::pi_v<T> *
                Radius *
                segmentLength
                +
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
            // Capsule volume:
            // cylinder volume + sphere volume
            //
            // V = pi*r^2*h + 4/3*pi*r^3
            //
            // where h is the segment length, not total capsule height.
            const T segmentLength =
                AxisLength();

            return
                std::numbers::pi_v<T> *
                Radius *
                Radius *
                segmentLength
                +
                (T(4) / T(3)) *
                std::numbers::pi_v<T> *
                Radius *
                Radius *
                Radius;
        }

        //-----------------------------------------------------
        // Axis Projection
        //-----------------------------------------------------
        /// Input: function parameters as declared by `ClosestAxisParameter`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ClosestAxisParameter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T ClosestAxisParameter(
            const Vector3<T>& point) const noexcept
        {
            const Vector3<T> axis =
                Axis();

            const T lengthSquared =
                axis.LengthSquared();

            if (lengthSquared <=
                std::numeric_limits<T>::epsilon())
            {
                return T(0);
            }

            return Clamp(
                Dot(point - Start, axis) / lengthSquared,
                T(0),
                T(1));
        }
        /// Input: function parameters as declared by `ClosestPointOnAxis`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ClosestPointOnAxis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        Vector3<T> ClosestPointOnAxis(
            const Vector3<T>& point) const noexcept
        {
            return
                Start +
                Axis() *
                ClosestAxisParameter(point);
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
            const Vector3<T> axisPoint =
                ClosestPointOnAxis(point);

            const Vector3<T> axisToPoint =
                point - axisPoint;

            const T lengthSquared =
                axisToPoint.LengthSquared();

            if (lengthSquared <=
                std::numeric_limits<T>::epsilon())
            {
                return
                    axisPoint +
                    Direction() *
                    Radius;
            }

            return
                axisPoint +
                SafeNormalize(
                    axisToPoint,
                    Direction())
                * Radius;
        }
        /// Input: function parameters as declared by `SignedDistanceToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `SignedDistanceToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T SignedDistanceToPoint(
            const Vector3<T>& point) const noexcept
        {
            return
                kairo::foundation::math::Distance(
                    point,
                    ClosestPointOnAxis(point))
                - Radius;
        }
        /// Input: function parameters as declared by `DistanceToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `DistanceToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T DistanceToPoint(
            const Vector3<T>& point) const noexcept
        {
            return Max(
                SignedDistanceToPoint(point),
                T(0));
        }
        /// Input: function parameters as declared by `DistanceSquaredToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `DistanceSquaredToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T DistanceSquaredToPoint(
            const Vector3<T>& point) const noexcept
        {
            const T signedDistance =
                SignedDistanceToPoint(point);

            if (signedDistance <= T(0))
            {
                return T(0);
            }

            return signedDistance * signedDistance;
        }

        //-----------------------------------------------------
        // Containment
        //-----------------------------------------------------
        /// Input: function parameters as declared by `ContainsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ContainsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool ContainsPoint(
            const Vector3<T>& point) const noexcept
        {
            return
                kairo::foundation::math::DistanceSquared(
                    point,
                    ClosestPointOnAxis(point))
                <= RadiusSquared();
        }
        /// Input: function parameters as declared by `ContainsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ContainsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool ContainsPoint(
            const Vector3<T>& point,
            T epsilon) const noexcept
        {
            const T expandedRadius =
                Radius + epsilon;

            return
                kairo::foundation::math::DistanceSquared(
                    point,
                    ClosestPointOnAxis(point))
                <= expandedRadius * expandedRadius;
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
        /// Input: function parameters as declared by `NormalizeRadius`.
        /// Output: none; mutates the object or output parameters described by the function.
        /// Task: provide the `NormalizeRadius` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        void NormalizeRadius() noexcept
        {
            if (Radius < T(0))
            {
                Radius = T(0);
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
        /// Input: function parameters as declared by `IsSphere`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsSphere` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool IsSphere(
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(10)) const noexcept
        {
            return
                AxisLengthSquared()
                <= epsilon * epsilon;
        }
        /// Input: function parameters as declared by `IsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool IsPoint() const noexcept
        {
            return
                Start == End &&
                Radius == T(0);
        }

        //-----------------------------------------------------
        // Equality
        //-----------------------------------------------------
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(
            const Capsule&) const noexcept = default;
    };

    //=========================================================
    // Free Functions
    //=========================================================

    /// Input: function parameters as declared by `Axis`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Axis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Axis(
        const Capsule<T>& capsule) noexcept
    {
        return capsule.Axis();
    }

    /// Input: function parameters as declared by `Direction`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Direction` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> Direction(
        const Capsule<T>& capsule) noexcept
    {
        return capsule.Direction();
    }

    /// Input: function parameters as declared by `Center`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Center` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Center(
        const Capsule<T>& capsule) noexcept
    {
        return capsule.Center();
    }

    /// Input: function parameters as declared by `Height`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Height` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Height(
        const Capsule<T>& capsule) noexcept
    {
        return capsule.Height();
    }

    /// Input: function parameters as declared by `RadiusSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `RadiusSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T RadiusSquared(
        const Capsule<T>& capsule) noexcept
    {
        return capsule.RadiusSquared();
    }

    /// Input: function parameters as declared by `SurfaceArea`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SurfaceArea` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T SurfaceArea(
        const Capsule<T>& capsule) noexcept
    {
        return capsule.SurfaceArea();
    }

    /// Input: function parameters as declared by `Volume`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Volume` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Volume(
        const Capsule<T>& capsule) noexcept
    {
        return capsule.Volume();
    }

    /// Input: function parameters as declared by `ClosestAxisParameter`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestAxisParameter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T ClosestAxisParameter(
        const Capsule<T>& capsule,
        const Vector3<T>& point) noexcept
    {
        return capsule.ClosestAxisParameter(
            point);
    }

    /// Input: function parameters as declared by `ClosestPointOnAxis`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPointOnAxis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPointOnAxis(
        const Capsule<T>& capsule,
        const Vector3<T>& point) noexcept
    {
        return capsule.ClosestPointOnAxis(
            point);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPoint(
        const Capsule<T>& capsule,
        const Vector3<T>& point) noexcept
    {
        return capsule.ClosestPoint(
            point);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> ClosestPoint(
        const Vector3<T>& point,
        const Capsule<T>& capsule) noexcept
    {
        return capsule.ClosestPoint(
            point);
    }

    /// Input: function parameters as declared by `SignedDistance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SignedDistance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T SignedDistance(
        const Capsule<T>& capsule,
        const Vector3<T>& point) noexcept
    {
        return capsule.SignedDistanceToPoint(
            point);
    }

    /// Input: function parameters as declared by `SignedDistance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SignedDistance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T SignedDistance(
        const Vector3<T>& point,
        const Capsule<T>& capsule) noexcept
    {
        return capsule.SignedDistanceToPoint(
            point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Capsule<T>& capsule,
        const Vector3<T>& point) noexcept
    {
        return capsule.DistanceToPoint(
            point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Vector3<T>& point,
        const Capsule<T>& capsule) noexcept
    {
        return capsule.DistanceToPoint(
            point);
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquared(
        const Capsule<T>& capsule,
        const Vector3<T>& point) noexcept
    {
        return capsule.DistanceSquaredToPoint(
            point);
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T DistanceSquared(
        const Vector3<T>& point,
        const Capsule<T>& capsule) noexcept
    {
        return capsule.DistanceSquaredToPoint(
            point);
    }

    /// Input: function parameters as declared by `Contains`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Contains` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Contains(
        const Capsule<T>& capsule,
        const Vector3<T>& point) noexcept
    {
        return capsule.ContainsPoint(
            point);
    }

    /// Input: function parameters as declared by `Contains`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Contains` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Contains(
        const Capsule<T>& capsule,
        const Vector3<T>& point,
        T epsilon) noexcept
    {
        return capsule.ContainsPoint(
            point,
            epsilon);
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const Capsule<T>& lhs,
        const Capsule<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon()
            * T(10)) noexcept
    {
        return
            NearlyEqual(
                lhs.Start,
                rhs.Start,
                epsilon)
            &&
            NearlyEqual(
                lhs.End,
                rhs.End,
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

    using Capsulef =
        Capsule<float>;

    using Capsuled =
        Capsule<double>;

    //=========================================================
    // Constants
    //=========================================================

    namespace constants
    {
        inline constexpr Capsulef UnitCapsulef =
            Capsulef::Unit();
    }

    //=========================================================
    // Compile-Time Validation
    //=========================================================

    static_assert(
        std::is_trivially_copyable_v<Capsulef>);

    static_assert(
        std::is_standard_layout_v<Capsulef>);

} // namespace kairo::foundation::geometry
