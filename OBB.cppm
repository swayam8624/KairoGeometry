module;

#include <array>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <type_traits>

export module Kairo.Foundation.Geometry.OBB;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.Math.Quaternion;
import Kairo.Foundation.Math.Transform;

import Kairo.Foundation.Geometry.AABB;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // OBB
    //=========================================================

    /// An oriented bounding box.
    ///
    /// Representation:
    /// - Center: box center in world/local space.
    /// - Axes[0], Axes[1], Axes[2]: local X/Y/Z box axes.
    /// - HalfExtents: positive half-size along each local axis.
    ///
    /// Convention:
    /// - Axes should be normalized and orthogonal.
    /// - HalfExtents should be non-negative.
    template<FloatingPoint T>
    struct OBB final
    {
        using ValueType = T;

        Vector3<T> Center =
            Vector3<T>::Zero();

        std::array<Vector3<T>, 3> Axes
        {
            Vector3<T>::UnitX(),
            Vector3<T>::UnitY(),
            Vector3<T>::UnitZ()
        };

        Vector3<T> HalfExtents =
            Vector3<T>
            {
                T(0.5),
                T(0.5),
                T(0.5)
            };

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Unit`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Unit` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr OBB Unit() noexcept
        {
            return {};
        }
        /// Input: function parameters as declared by `FromCenterAxesHalfExtents`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromCenterAxesHalfExtents` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr OBB FromCenterAxesHalfExtents(
            const Vector3<T>& center,
            const Vector3<T>& axisX,
            const Vector3<T>& axisY,
            const Vector3<T>& axisZ,
            const Vector3<T>& halfExtents) noexcept
        {
            return
            {
                center,
                { axisX, axisY, axisZ },
                halfExtents
            };
        }
        /// Input: function parameters as declared by `FromCenterRotationHalfExtents`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromCenterRotationHalfExtents` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static OBB FromCenterRotationHalfExtents(
            const Vector3<T>& center,
            const Quaternion<T>& rotation,
            const Vector3<T>& halfExtents) noexcept
        {
            return
            {
                center,
                {
                    Right(rotation),
                    Up(rotation),
                    Forward(rotation)
                },
                halfExtents
            };
        }
        /// Input: function parameters as declared by `FromAABB`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static OBB FromAABB(
            const AABB<T>& box) noexcept
        {
            return
            {
                box.Center(),
                {
                    Vector3<T>::UnitX(),
                    Vector3<T>::UnitY(),
                    Vector3<T>::UnitZ()
                },
                box.Extent()
            };
        }
        /// Input: function parameters as declared by `FromTransform`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromTransform` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static OBB FromTransform(
            const Transform<T>& transform) noexcept
        {
            return FromCenterRotationHalfExtents(
                transform.Translation,
                transform.Rotation,
                transform.Scale * T(0.5));
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `OBB` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr OBB() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `OBB` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr OBB(
            const Vector3<T>& center,
            const std::array<Vector3<T>, 3>& axes,
            const Vector3<T>& halfExtents) noexcept
            :
            Center(center),
            Axes(axes),
            HalfExtents(halfExtents)
        {
        }

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `OBB` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr OBB(
            const Vector3<T>& center,
            const Vector3<T>& axisX,
            const Vector3<T>& axisY,
            const Vector3<T>& axisZ,
            const Vector3<T>& halfExtents) noexcept
            :
            Center(center),
            Axes{ axisX, axisY, axisZ },
            HalfExtents(halfExtents)
        {
        }

        //-----------------------------------------------------
        // Axis Access
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Axis`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Axis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr const Vector3<T>& Axis(
            std::size_t index) const noexcept
        {
            assert(index < 3);
            return Axes[index];
        }
        /// Input: function parameters as declared by `Axis`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Axis` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T>& Axis(
            std::size_t index) noexcept
        {
            assert(index < 3);
            return Axes[index];
        }
        /// Input: function parameters as declared by `AxisX`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `AxisX` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr const Vector3<T>& AxisX() const noexcept
        {
            return Axes[0];
        }
        /// Input: function parameters as declared by `AxisY`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `AxisY` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr const Vector3<T>& AxisY() const noexcept
        {
            return Axes[1];
        }
        /// Input: function parameters as declared by `AxisZ`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `AxisZ` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr const Vector3<T>& AxisZ() const noexcept
        {
            return Axes[2];
        }

        //-----------------------------------------------------
        // Basic Properties
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Size`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Size` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> Size() const noexcept
        {
            return HalfExtents * T(2);
        }
        /// Input: function parameters as declared by `Volume`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Volume` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T Volume() const noexcept
        {
            const Vector3<T> size =
                Size();

            return
                size.x *
                size.y *
                size.z;
        }
        /// Input: function parameters as declared by `SurfaceArea`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `SurfaceArea` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T SurfaceArea() const noexcept
        {
            const Vector3<T> size =
                Size();

            return
                T(2) *
                (
                    size.x * size.y +
                    size.y * size.z +
                    size.z * size.x
                );
        }

        //-----------------------------------------------------
        // Corner Access
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Corner`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Corner` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> Corner(
            std::size_t index) const noexcept
        {
            assert(index < 8);

            const T sx =
                (index & 1)
                ? T(1)
                : T(-1);

            const T sy =
                (index & 2)
                ? T(1)
                : T(-1);

            const T sz =
                (index & 4)
                ? T(1)
                : T(-1);

            return
                Center +
                Axes[0] * (HalfExtents.x * sx) +
                Axes[1] * (HalfExtents.y * sy) +
                Axes[2] * (HalfExtents.z * sz);
        }

        //-----------------------------------------------------
        // Local / World Conversion
        //-----------------------------------------------------
        /// Input: function parameters as declared by `WorldToLocalPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `WorldToLocalPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> WorldToLocalPoint(
            const Vector3<T>& point) const noexcept
        {
            const Vector3<T> delta =
                point - Center;

            return
            {
                Dot(delta, Axes[0]),
                Dot(delta, Axes[1]),
                Dot(delta, Axes[2])
            };
        }
        /// Input: function parameters as declared by `LocalToWorldPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `LocalToWorldPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> LocalToWorldPoint(
            const Vector3<T>& point) const noexcept
        {
            return
                Center +
                Axes[0] * point.x +
                Axes[1] * point.y +
                Axes[2] * point.z;
        }
        /// Input: function parameters as declared by `WorldToLocalDirection`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `WorldToLocalDirection` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> WorldToLocalDirection(
            const Vector3<T>& direction) const noexcept
        {
            return
            {
                Dot(direction, Axes[0]),
                Dot(direction, Axes[1]),
                Dot(direction, Axes[2])
            };
        }
        /// Input: function parameters as declared by `LocalToWorldDirection`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `LocalToWorldDirection` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> LocalToWorldDirection(
            const Vector3<T>& direction) const noexcept
        {
            return
                Axes[0] * direction.x +
                Axes[1] * direction.y +
                Axes[2] * direction.z;
        }

        //-----------------------------------------------------
        // Closest / Distance
        //-----------------------------------------------------
        /// Input: function parameters as declared by `ClosestPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> ClosestPoint(
            const Vector3<T>& point) const noexcept
        {
            const Vector3<T> local =
                WorldToLocalPoint(point);

            const Vector3<T> clamped
            {
                Clamp(local.x, -HalfExtents.x, HalfExtents.x),
                Clamp(local.y, -HalfExtents.y, HalfExtents.y),
                Clamp(local.z, -HalfExtents.z, HalfExtents.z)
            };

            return LocalToWorldPoint(clamped);
        }
        /// Input: function parameters as declared by `DistanceSquaredToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `DistanceSquaredToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T DistanceSquaredToPoint(
            const Vector3<T>& point) const noexcept
        {
            return kairo::foundation::math::DistanceSquared(
                point,
                ClosestPoint(point));
        }
        /// Input: function parameters as declared by `DistanceToPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `DistanceToPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T DistanceToPoint(
            const Vector3<T>& point) const noexcept
        {
            return std::sqrt(
                DistanceSquaredToPoint(point));
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
            const Vector3<T> local =
                WorldToLocalPoint(point);

            return
                local.x >= -HalfExtents.x &&
                local.x <=  HalfExtents.x &&
                local.y >= -HalfExtents.y &&
                local.y <=  HalfExtents.y &&
                local.z >= -HalfExtents.z &&
                local.z <=  HalfExtents.z;
        }
        /// Input: function parameters as declared by `ContainsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ContainsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool ContainsPoint(
            const Vector3<T>& point,
            T epsilon) const noexcept
        {
            const Vector3<T> local =
                WorldToLocalPoint(point);

            return
                local.x >= -HalfExtents.x - epsilon &&
                local.x <=  HalfExtents.x + epsilon &&
                local.y >= -HalfExtents.y - epsilon &&
                local.y <=  HalfExtents.y + epsilon &&
                local.z >= -HalfExtents.z - epsilon &&
                local.z <=  HalfExtents.z + epsilon;
        }

        //-----------------------------------------------------
        // Conversion
        //-----------------------------------------------------
        /// Input: function parameters as declared by `ToAABB`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ToAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        AABB<T> ToAABB() const noexcept
        {
            AABB<T> result =
                AABB<T>::FromPoint(
                    Corner(0));

            for (std::size_t i = 1; i < 8; ++i)
            {
                result.ExpandToInclude(
                    Corner(i));
            }

            return result;
        }

        //-----------------------------------------------------
        // Mutation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `NormalizeAxes`.
        /// Output: none; mutates the object or output parameters described by the function.
        /// Task: provide the `NormalizeAxes` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        void NormalizeAxes() noexcept
        {
            Axes[0] =
                SafeNormalize(
                    Axes[0],
                    Vector3<T>::UnitX());

            Axes[1] =
                SafeNormalize(
                    Axes[1],
                    Vector3<T>::UnitY());

            Axes[2] =
                SafeNormalize(
                    Axes[2],
                    Vector3<T>::UnitZ());
        }
        /// Input: function parameters as declared by `NormalizeExtents`.
        /// Output: none; mutates the object or output parameters described by the function.
        /// Task: provide the `NormalizeExtents` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        void NormalizeExtents() noexcept
        {
            HalfExtents.x =
                Max(HalfExtents.x, T(0));

            HalfExtents.y =
                Max(HalfExtents.y, T(0));

            HalfExtents.z =
                Max(HalfExtents.z, T(0));
        }

        //-----------------------------------------------------
        // Validation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `HasValidExtents`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `HasValidExtents` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool HasValidExtents() const noexcept
        {
            return
                HalfExtents.x >= T(0) &&
                HalfExtents.y >= T(0) &&
                HalfExtents.z >= T(0);
        }
        /// Input: function parameters as declared by `HasNormalizedAxes`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `HasNormalizedAxes` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool HasNormalizedAxes(
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(100)) const noexcept
        {
            return
                kairo::foundation::math::NearlyEqual(
                    Axes[0].LengthSquared(),
                    T(1),
                    epsilon)
                &&
                kairo::foundation::math::NearlyEqual(
                    Axes[1].LengthSquared(),
                    T(1),
                    epsilon)
                &&
                kairo::foundation::math::NearlyEqual(
                    Axes[2].LengthSquared(),
                    T(1),
                    epsilon);
        }
        /// Input: function parameters as declared by `HasOrthogonalAxes`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `HasOrthogonalAxes` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool HasOrthogonalAxes(
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(100)) const noexcept
        {
            return
                std::abs(Dot(Axes[0], Axes[1])) <= epsilon &&
                std::abs(Dot(Axes[1], Axes[2])) <= epsilon &&
                std::abs(Dot(Axes[2], Axes[0])) <= epsilon;
        }
        /// Input: function parameters as declared by `IsValid`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsValid` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool IsValid(
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(100)) const noexcept
        {
            return
                HasValidExtents() &&
                HasNormalizedAxes(epsilon) &&
                HasOrthogonalAxes(epsilon);
        }

        //-----------------------------------------------------
        // Equality
        //-----------------------------------------------------
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(
            const OBB&) const noexcept = default;
    };

    //=========================================================
    // Free Functions
    //=========================================================

    /// Input: function parameters as declared by `Center`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Center` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Center(
        const OBB<T>& box) noexcept
    {
        return box.Center;
    }

    /// Input: function parameters as declared by `Size`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Size` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Size(
        const OBB<T>& box) noexcept
    {
        return box.Size();
    }

    /// Input: function parameters as declared by `Volume`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Volume` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T Volume(
        const OBB<T>& box) noexcept
    {
        return box.Volume();
    }

    /// Input: function parameters as declared by `SurfaceArea`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SurfaceArea` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T SurfaceArea(
        const OBB<T>& box) noexcept
    {
        return box.SurfaceArea();
    }

    /// Input: function parameters as declared by `Corner`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Corner` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Corner(
        const OBB<T>& box,
        std::size_t index) noexcept
    {
        return box.Corner(index);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> ClosestPoint(
        const OBB<T>& box,
        const Vector3<T>& point) noexcept
    {
        return box.ClosestPoint(point);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> ClosestPoint(
        const Vector3<T>& point,
        const OBB<T>& box) noexcept
    {
        return box.ClosestPoint(point);
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T DistanceSquared(
        const OBB<T>& box,
        const Vector3<T>& point) noexcept
    {
        return box.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T DistanceSquared(
        const Vector3<T>& point,
        const OBB<T>& box) noexcept
    {
        return box.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const OBB<T>& box,
        const Vector3<T>& point) noexcept
    {
        return box.DistanceToPoint(point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const Vector3<T>& point,
        const OBB<T>& box) noexcept
    {
        return box.DistanceToPoint(point);
    }

    /// Input: function parameters as declared by `Contains`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Contains` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Contains(
        const OBB<T>& box,
        const Vector3<T>& point) noexcept
    {
        return box.ContainsPoint(point);
    }

    /// Input: function parameters as declared by `Contains`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Contains` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Contains(
        const OBB<T>& box,
        const Vector3<T>& point,
        T epsilon) noexcept
    {
        return box.ContainsPoint(
            point,
            epsilon);
    }

    /// Input: function parameters as declared by `ToAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ToAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    AABB<T> ToAABB(
        const OBB<T>& box) noexcept
    {
        return box.ToAABB();
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const OBB<T>& lhs,
        const OBB<T>& rhs,
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
                lhs.Axes[0],
                rhs.Axes[0],
                epsilon)
            &&
            NearlyEqual(
                lhs.Axes[1],
                rhs.Axes[1],
                epsilon)
            &&
            NearlyEqual(
                lhs.Axes[2],
                rhs.Axes[2],
                epsilon)
            &&
            NearlyEqual(
                lhs.HalfExtents,
                rhs.HalfExtents,
                epsilon);
    }

    //=========================================================
    // Aliases
    //=========================================================

    using OBBf =
        OBB<float>;

    using OBBd =
        OBB<double>;

    //=========================================================
    // Constants
    //=========================================================

    namespace constants
    {
        inline constexpr OBBf UnitOBBf =
            OBBf::Unit();
    }

    //=========================================================
    // Compile-Time Validation
    //=========================================================

    static_assert(
        std::is_trivially_copyable_v<OBBf>);

    static_assert(
        std::is_standard_layout_v<OBBf>);

} // namespace kairo::foundation::geometry
