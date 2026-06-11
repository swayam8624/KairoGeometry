module;

#include <algorithm>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <type_traits>

export module Kairo.Foundation.Geometry.AABB;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Math.Matrix;
import Kairo.Foundation.Math.Transform;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // AABB
    //=========================================================

    /// Axis-aligned bounding box represented by minimum and maximum corners.
    ///
    /// Convention:
    /// - Min <= Max on every axis means the box is valid.
    /// - Invalid/empty boxes can be represented by Min > Max.
    /// - AABB is always aligned to world/local axes, never rotated.
    template<FloatingPoint T>
    struct AABB final
    {
        using ValueType = T;

        Vector3<T> Min =
        {
            T(0),
            T(0),
            T(0)
        };

        Vector3<T> Max =
        {
            T(0),
            T(0),
            T(0)
        };

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Empty`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Empty` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr AABB Empty() noexcept
        {
            return
            {
                Vector3<T>
                {
                    std::numeric_limits<T>::max(),
                    std::numeric_limits<T>::max(),
                    std::numeric_limits<T>::max()
                },

                Vector3<T>
                {
                    std::numeric_limits<T>::lowest(),
                    std::numeric_limits<T>::lowest(),
                    std::numeric_limits<T>::lowest()
                }
            };
        }
        /// Input: function parameters as declared by `Unit`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Unit` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr AABB Unit() noexcept
        {
            return
            {
                Vector3<T>
                {
                    T(-0.5),
                    T(-0.5),
                    T(-0.5)
                },

                Vector3<T>
                {
                    T(0.5),
                    T(0.5),
                    T(0.5)
                }
            };
        }
        /// Input: function parameters as declared by `FromMinMax`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromMinMax` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr AABB FromMinMax(
            const Vector3<T>& min,
            const Vector3<T>& max) noexcept
        {
            return
            {
                min,
                max
            };
        }
        /// Input: function parameters as declared by `FromCenterExtent`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromCenterExtent` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr AABB FromCenterExtent(
            const Vector3<T>& center,
            const Vector3<T>& extent) noexcept
        {
            return
            {
                center - extent,
                center + extent
            };
        }
        /// Input: function parameters as declared by `FromPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr AABB FromPoint(
            const Vector3<T>& point) noexcept
        {
            return
            {
                point,
                point
            };
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `AABB` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr AABB() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `AABB` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr AABB(
            const Vector3<T>& min,
            const Vector3<T>& max) noexcept
            :
            Min(min),
            Max(max)
        {
        }

        //-----------------------------------------------------
        // Basic Properties
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Center`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Center` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> Center() const noexcept
        {
            return
                (Min + Max) *
                T(0.5);
        }
        /// Input: function parameters as declared by `Size`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Size` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> Size() const noexcept
        {
            return Max - Min;
        }
        /// Input: function parameters as declared by `Extent`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Extent` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> Extent() const noexcept
        {
            return
                Size() *
                T(0.5);
        }
        /// Input: function parameters as declared by `HalfSize`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `HalfSize` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> HalfSize() const noexcept
        {
            return Extent();
        }
        /// Input: function parameters as declared by `Width`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Width` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T Width() const noexcept
        {
            return Max.x - Min.x;
        }
        /// Input: function parameters as declared by `Height`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Height` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T Height() const noexcept
        {
            return Max.y - Min.y;
        }
        /// Input: function parameters as declared by `Depth`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Depth` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T Depth() const noexcept
        {
            return Max.z - Min.z;
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
        // Corners
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Corner`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Corner` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> Corner(
            std::size_t index) const noexcept
        {
            assert(index < 8);

            return
            {
                (index & 1) ? Max.x : Min.x,
                (index & 2) ? Max.y : Min.y,
                (index & 4) ? Max.z : Min.z
            };
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
                point.x >= Min.x &&
                point.x <= Max.x &&
                point.y >= Min.y &&
                point.y <= Max.y &&
                point.z >= Min.z &&
                point.z <= Max.z;
        }
        /// Input: function parameters as declared by `ContainsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ContainsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool ContainsPoint(
            const Vector3<T>& point,
            T epsilon) const noexcept
        {
            return
                point.x >= Min.x - epsilon &&
                point.x <= Max.x + epsilon &&
                point.y >= Min.y - epsilon &&
                point.y <= Max.y + epsilon &&
                point.z >= Min.z - epsilon &&
                point.z <= Max.z + epsilon;
        }
        /// Input: function parameters as declared by `ContainsAABB`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ContainsAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool ContainsAABB(
            const AABB& other) const noexcept
        {
            return
                ContainsPoint(other.Min) &&
                ContainsPoint(other.Max);
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
            return
            {
                Clamp(point.x, Min.x, Max.x),
                Clamp(point.y, Min.y, Max.y),
                Clamp(point.z, Min.z, Max.z)
            };
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
        // Expansion / Mutation
        //-----------------------------------------------------

        /// Input: function parameters as declared by `Expand`.
        /// Output: none; mutates this geometry value in place.
        /// Task: update the value while preserving documented geometry invariants.
        constexpr void Expand(
            T amount) noexcept
        {
            const Vector3<T> expansion
            {
                amount,
                amount,
                amount
            };

            Min -= expansion;
            Max += expansion;
        }

        /// Input: function parameters as declared by `Expand`.
        /// Output: none; mutates this geometry value in place.
        /// Task: update the value while preserving documented geometry invariants.
        constexpr void Expand(
            const Vector3<T>& amount) noexcept
        {
            Min -= amount;
            Max += amount;
        }

        /// Input: function parameters as declared by `ExpandToInclude`.
        /// Output: none; mutates this geometry value in place.
        /// Task: update the value while preserving documented geometry invariants.
        constexpr void ExpandToInclude(
            const Vector3<T>& point) noexcept
        {
            Min =
            {
                kairo::foundation::math::Min(Min.x, point.x),
                kairo::foundation::math::Min(Min.y, point.y),
                kairo::foundation::math::Min(Min.z, point.z)
            };

            Max =
            {
                kairo::foundation::math::Max(Max.x, point.x),
                kairo::foundation::math::Max(Max.y, point.y),
                kairo::foundation::math::Max(Max.z, point.z)
            };
        }

        /// Input: function parameters as declared by `ExpandToInclude`.
        /// Output: none; mutates this geometry value in place.
        /// Task: update the value while preserving documented geometry invariants.
        constexpr void ExpandToInclude(
            const AABB& other) noexcept
        {
            ExpandToInclude(other.Min);
            ExpandToInclude(other.Max);
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
            return
                Min.x <= Max.x &&
                Min.y <= Max.y &&
                Min.z <= Max.z;
        }
        /// Input: function parameters as declared by `IsEmpty`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsEmpty` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool IsEmpty() const noexcept
        {
            return !IsValid();
        }
        /// Input: function parameters as declared by `IsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool IsPoint() const noexcept
        {
            return
                Min.x == Max.x &&
                Min.y == Max.y &&
                Min.z == Max.z;
        }

        //-----------------------------------------------------
        // Equality
        //-----------------------------------------------------
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(
            const AABB&) const noexcept = default;
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
        const AABB<T>& box) noexcept
    {
        return box.Center();
    }

    /// Input: function parameters as declared by `Size`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Size` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Size(
        const AABB<T>& box) noexcept
    {
        return box.Size();
    }

    /// Input: function parameters as declared by `Extent`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Extent` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Extent(
        const AABB<T>& box) noexcept
    {
        return box.Extent();
    }

    /// Input: function parameters as declared by `Volume`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Volume` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T Volume(
        const AABB<T>& box) noexcept
    {
        return box.Volume();
    }

    /// Input: function parameters as declared by `SurfaceArea`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `SurfaceArea` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T SurfaceArea(
        const AABB<T>& box) noexcept
    {
        return box.SurfaceArea();
    }

    /// Input: function parameters as declared by `Corner`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Corner` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Corner(
        const AABB<T>& box,
        std::size_t index) noexcept
    {
        return box.Corner(index);
    }

    /// Input: function parameters as declared by `Contains`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Contains` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Contains(
        const AABB<T>& box,
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
        const AABB<T>& outer,
        const AABB<T>& inner) noexcept
    {
        return outer.ContainsAABB(inner);
    }

    /// Input: function parameters as declared by `ClosestPoint`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ClosestPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> ClosestPoint(
        const AABB<T>& box,
        const Vector3<T>& point) noexcept
    {
        return box.ClosestPoint(point);
    }

    /// Input: function parameters as declared by `DistanceSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `DistanceSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T DistanceSquared(
        const AABB<T>& box,
        const Vector3<T>& point) noexcept
    {
        return box.DistanceSquaredToPoint(point);
    }

    /// Input: function parameters as declared by `Distance`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Distance` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Distance(
        const AABB<T>& box,
        const Vector3<T>& point) noexcept
    {
        return box.DistanceToPoint(point);
    }

    /// Input: function parameters as declared by `Intersects`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersects` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr bool Intersects(
        const AABB<T>& lhs,
        const AABB<T>& rhs) noexcept
    {
        return
            lhs.Min.x <= rhs.Max.x &&
            lhs.Max.x >= rhs.Min.x &&
            lhs.Min.y <= rhs.Max.y &&
            lhs.Max.y >= rhs.Min.y &&
            lhs.Min.z <= rhs.Max.z &&
            lhs.Max.z >= rhs.Min.z;
    }

    /// Input: function parameters as declared by `Intersection`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Intersection` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr AABB<T> Intersection(
        const AABB<T>& lhs,
        const AABB<T>& rhs) noexcept
    {
        if (!Intersects(lhs, rhs))
        {
            return AABB<T>::Empty();
        }

        return
        {
            Vector3<T>
            {
                kairo::foundation::math::Max(lhs.Min.x, rhs.Min.x),
                kairo::foundation::math::Max(lhs.Min.y, rhs.Min.y),
                kairo::foundation::math::Max(lhs.Min.z, rhs.Min.z)
            },

            Vector3<T>
            {
                kairo::foundation::math::Min(lhs.Max.x, rhs.Max.x),
                kairo::foundation::math::Min(lhs.Max.y, rhs.Max.y),
                kairo::foundation::math::Min(lhs.Max.z, rhs.Max.z)
            }
        };
    }

    /// Input: function parameters as declared by `Merge`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Merge` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr AABB<T> Merge(
        const AABB<T>& lhs,
        const AABB<T>& rhs) noexcept
    {
        if (!lhs.IsValid())
        {
            return rhs;
        }

        if (!rhs.IsValid())
        {
            return lhs;
        }

        return
        {
            Vector3<T>
            {
                kairo::foundation::math::Min(lhs.Min.x, rhs.Min.x),
                kairo::foundation::math::Min(lhs.Min.y, rhs.Min.y),
                kairo::foundation::math::Min(lhs.Min.z, rhs.Min.z)
            },

            Vector3<T>
            {
                kairo::foundation::math::Max(lhs.Max.x, rhs.Max.x),
                kairo::foundation::math::Max(lhs.Max.y, rhs.Max.y),
                kairo::foundation::math::Max(lhs.Max.z, rhs.Max.z)
            }
        };
    }

    /// Input: function parameters as declared by `Expanded`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Expanded` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr AABB<T> Expanded(
        AABB<T> box,
        T amount) noexcept
    {
        box.Expand(amount);
        return box;
    }

    /// Input: function parameters as declared by `Expanded`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Expanded` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr AABB<T> Expanded(
        AABB<T> box,
        const Vector3<T>& amount) noexcept
    {
        box.Expand(amount);
        return box;
    }

    /// Input: function parameters as declared by `TransformAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `TransformAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    AABB<T> TransformAABB(
        const AABB<T>& box,
        const Matrix4<T>& matrix) noexcept
    {
        if (!box.IsValid())
        {
            return AABB<T>::Empty();
        }

        AABB<T> result =
            AABB<T>::FromPoint(
                TransformPoint(
                    matrix,
                    box.Corner(0)));

        for (std::size_t i = 1; i < 8; ++i)
        {
            result.ExpandToInclude(
                TransformPoint(
                    matrix,
                    box.Corner(i)));
        }

        return result;
    }

    /// Input: function parameters as declared by `TransformAABB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `TransformAABB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    AABB<T> TransformAABB(
        const AABB<T>& box,
        const Transform<T>& transform) noexcept
    {
        return TransformAABB(
            box,
            ToMatrix4(transform));
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const AABB<T>& lhs,
        const AABB<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon()
            * T(10)) noexcept
    {
        return
            NearlyEqual(
                lhs.Min,
                rhs.Min,
                epsilon)
            &&
            NearlyEqual(
                lhs.Max,
                rhs.Max,
                epsilon);
    }

    //=========================================================
    // Aliases
    //=========================================================

    using AABBf =
        AABB<float>;

    using AABBd =
        AABB<double>;

    //=========================================================
    // Constants
    //=========================================================

    namespace constants
    {
        inline constexpr AABBf UnitAABBf =
            AABBf::Unit();
    }

    //=========================================================
    // Compile-Time Validation
    //=========================================================

    static_assert(
        std::is_trivially_copyable_v<AABBf>);

    static_assert(
        std::is_standard_layout_v<AABBf>);

} // namespace kairo::foundation::geometry
