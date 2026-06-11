module;

#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <type_traits>

export module Kairo.Foundation.Geometry.Triangle;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.Plane;

export namespace kairo::foundation::geometry
{
    using namespace kairo::foundation::math;

    //=========================================================
    // Triangle
    //=========================================================

    /// A 3D triangle represented by three vertices:
    ///
    ///     A, B, C
    ///
    /// Winding convention:
    /// - The front-facing normal is computed as:
    ///       Normalize(Cross(B - A, C - A))
    ///
    /// Notes:
    /// - Degenerate triangles have nearly zero area.
    /// - For ray tracing and collision, triangle winding matters.
    template<FloatingPoint T>
    struct Triangle final
    {
        using ValueType = T;

        Vector3<T> A =
            Vector3<T>::Zero();

        Vector3<T> B =
            Vector3<T>::UnitX();

        Vector3<T> C =
            Vector3<T>::UnitY();

        //-----------------------------------------------------
        // Static Constructors
        //-----------------------------------------------------
        /// Input: function parameters as declared by `FromPoints`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `FromPoints` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Triangle FromPoints(
            const Vector3<T>& a,
            const Vector3<T>& b,
            const Vector3<T>& c) noexcept
        {
            return
            {
                a,
                b,
                c
            };
        }
        /// Input: function parameters as declared by `UnitRightTriangleXY`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `UnitRightTriangleXY` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        static constexpr Triangle UnitRightTriangleXY() noexcept
        {
            return
            {
                Vector3<T>::Zero(),
                Vector3<T>::UnitX(),
                Vector3<T>::UnitY()
            };
        }

        //-----------------------------------------------------
        // Constructors
        //-----------------------------------------------------

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Triangle` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Triangle() noexcept = default;

        /// Input: constructor parameters, or none for the default constructor.
        /// Output: initialized `Triangle` value.
        /// Task: construct the transparent geometry value without hidden allocation or ownership.
        constexpr Triangle(
            const Vector3<T>& a,
            const Vector3<T>& b,
            const Vector3<T>& c) noexcept
            :
            A(a),
            B(b),
            C(c)
        {
        }

        //-----------------------------------------------------
        // Edges
        //-----------------------------------------------------
        /// Input: function parameters as declared by `EdgeAB`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `EdgeAB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> EdgeAB() const noexcept
        {
            return B - A;
        }
        /// Input: function parameters as declared by `EdgeAC`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `EdgeAC` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> EdgeAC() const noexcept
        {
            return C - A;
        }
        /// Input: function parameters as declared by `EdgeBC`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `EdgeBC` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> EdgeBC() const noexcept
        {
            return C - B;
        }

        //-----------------------------------------------------
        // Basic Properties
        //-----------------------------------------------------
        /// Input: function parameters as declared by `Centroid`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Centroid` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> Centroid() const noexcept
        {
            return
                (A + B + C) /
                T(3);
        }
        /// Input: function parameters as declared by `UnnormalizedNormal`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `UnnormalizedNormal` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr Vector3<T> UnnormalizedNormal() const noexcept
        {
            return Cross(
                EdgeAB(),
                EdgeAC());
        }
        /// Input: function parameters as declared by `Normal`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Normal` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        Vector3<T> Normal() const noexcept
        {
            return SafeNormalize(
                UnnormalizedNormal(),
                Vector3<T>::Up());
        }
        /// Input: function parameters as declared by `AreaSquared`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `AreaSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr T AreaSquared() const noexcept
        {
            return
                UnnormalizedNormal()
                    .LengthSquared()
                * T(0.25);
        }
        /// Input: function parameters as declared by `Area`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Area` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T Area() const noexcept
        {
            return
                UnnormalizedNormal()
                    .Length()
                * T(0.5);
        }
        /// Input: function parameters as declared by `Perimeter`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Perimeter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        T Perimeter() const noexcept
        {
            return
                Distance(A, B) +
                Distance(B, C) +
                Distance(C, A);
        }
        /// Input: function parameters as declared by `ToPlane`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ToPlane` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        Plane<T> ToPlane() const noexcept
        {
            return Plane<T>::FromPoints(
                A,
                B,
                C);
        }

        //-----------------------------------------------------
        // Barycentric Coordinates
        //-----------------------------------------------------

        /// Input: point.
        /// Output: barycentric coordinates relative to this triangle.
        ///
        /// Result:
        ///     x = weight for A
        ///     y = weight for B
        ///     z = weight for C
        ///
        /// If the triangle is degenerate, returns (-1, -1, -1).
        /// Input: function parameters as declared by `Barycentric`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `Barycentric` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
        [[nodiscard]]
        Vector3<T> Barycentric(
            const Vector3<T>& point,
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(10)) const noexcept
        {
            const Vector3<T> v0 =
                B - A;

            const Vector3<T> v1 =
                C - A;

            const Vector3<T> v2 =
                point - A;

            const T d00 =
                Dot(v0, v0);

            const T d01 =
                Dot(v0, v1);

            const T d11 =
                Dot(v1, v1);

            const T d20 =
                Dot(v2, v0);

            const T d21 =
                Dot(v2, v1);

            const T denominator =
                d00 * d11 -
                d01 * d01;

            if (std::abs(denominator) <= epsilon)
            {
                return
                {
                    T(-1),
                    T(-1),
                    T(-1)
                };
            }

            const T inverseDenominator =
                T(1) / denominator;

            const T v =
                (d11 * d20 - d01 * d21)
                * inverseDenominator;

            const T w =
                (d00 * d21 - d01 * d20)
                * inverseDenominator;

            const T u =
                T(1) - v - w;

            return
            {
                u,
                v,
                w
            };
        }

        //-----------------------------------------------------
        // Containment
        //-----------------------------------------------------
        /// Input: function parameters as declared by `ContainsPoint`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `ContainsPoint` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool ContainsPoint(
            const Vector3<T>& point,
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(100)) const noexcept
        {
            const Vector3<T> normal =
                Normal();

            const T planeDistance =
                Dot(
                    normal,
                    point - A);

            if (std::abs(planeDistance) > epsilon)
            {
                return false;
            }

            const Vector3<T> barycentric =
                Barycentric(
                    point,
                    epsilon);

            return
                barycentric.x >= -epsilon &&
                barycentric.y >= -epsilon &&
                barycentric.z >= -epsilon &&
                barycentric.x <= T(1) + epsilon &&
                barycentric.y <= T(1) + epsilon &&
                barycentric.z <= T(1) + epsilon;
        }

        //-----------------------------------------------------
        // Validation
        //-----------------------------------------------------
        /// Input: function parameters as declared by `IsDegenerate`.
        /// Output: the computed value described by the function name and return type.
        /// Task: provide the `IsDegenerate` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        bool IsDegenerate(
            T epsilon =
                std::numeric_limits<T>::epsilon()
                * T(100)) const noexcept
        {
            return
                UnnormalizedNormal()
                    .LengthSquared()
                <= epsilon * epsilon;
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
            return !IsDegenerate(
                epsilon);
        }

        //-----------------------------------------------------
        // Equality
        //-----------------------------------------------------
        /// Input: function parameters as declared by `operator==`.
        /// Output: the operator result for the supplied operands.
        /// Task: provide the `operator==` operation as part of the geometry API while preserving the module's value-type, allocation-free design.

        [[nodiscard]]
        constexpr bool operator==(
            const Triangle&) const noexcept = default;
    };

    //=========================================================
    // Free Functions
    //=========================================================

    /// Input: function parameters as declared by `EdgeAB`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `EdgeAB` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> EdgeAB(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.EdgeAB();
    }

    /// Input: function parameters as declared by `EdgeAC`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `EdgeAC` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> EdgeAC(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.EdgeAC();
    }

    /// Input: function parameters as declared by `EdgeBC`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `EdgeBC` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> EdgeBC(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.EdgeBC();
    }

    /// Input: function parameters as declared by `Centroid`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Centroid` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> Centroid(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.Centroid();
    }

    /// Input: function parameters as declared by `UnnormalizedNormal`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `UnnormalizedNormal` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr Vector3<T> UnnormalizedNormal(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.UnnormalizedNormal();
    }

    /// Input: function parameters as declared by `Normal`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Normal` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> Normal(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.Normal();
    }

    /// Input: function parameters as declared by `AreaSquared`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `AreaSquared` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    constexpr T AreaSquared(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.AreaSquared();
    }

    /// Input: function parameters as declared by `Area`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Area` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Area(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.Area();
    }

    /// Input: function parameters as declared by `Perimeter`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Perimeter` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    T Perimeter(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.Perimeter();
    }

    /// Input: function parameters as declared by `ToPlane`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `ToPlane` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Plane<T> ToPlane(
        const Triangle<T>& triangle) noexcept
    {
        return triangle.ToPlane();
    }

    /// Input: function parameters as declared by `Barycentric`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Barycentric` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    Vector3<T> Barycentric(
        const Triangle<T>& triangle,
        const Vector3<T>& point,
        T epsilon =
            std::numeric_limits<T>::epsilon()
            * T(10)) noexcept
    {
        return triangle.Barycentric(
            point,
            epsilon);
    }

    /// Input: function parameters as declared by `Contains`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `Contains` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool Contains(
        const Triangle<T>& triangle,
        const Vector3<T>& point,
        T epsilon =
            std::numeric_limits<T>::epsilon()
            * T(100)) noexcept
    {
        return triangle.ContainsPoint(
            point,
            epsilon);
    }

    /// Input: function parameters as declared by `NearlyEqual`.
    /// Output: the computed value described by the function name and return type.
    /// Task: provide the `NearlyEqual` operation as part of the geometry API while preserving the module's value-type, allocation-free design.
    template<FloatingPoint T>
    [[nodiscard]]
    bool NearlyEqual(
        const Triangle<T>& lhs,
        const Triangle<T>& rhs,
        T epsilon =
            std::numeric_limits<T>::epsilon()
            * T(10)) noexcept
    {
        return
            NearlyEqual(
                lhs.A,
                rhs.A,
                epsilon)
            &&
            NearlyEqual(
                lhs.B,
                rhs.B,
                epsilon)
            &&
            NearlyEqual(
                lhs.C,
                rhs.C,
                epsilon);
    }

    //=========================================================
    // Aliases
    //=========================================================

    using Trianglef =
        Triangle<float>;

    using Triangled =
        Triangle<double>;

    //=========================================================
    // Constants
    //=========================================================

    namespace constants
    {
        inline constexpr Trianglef UnitRightTriangleXYf =
            Trianglef::UnitRightTriangleXY();
    }

    //=========================================================
    // Compile-Time Validation
    //=========================================================

    static_assert(
        std::is_trivially_copyable_v<Trianglef>);

    static_assert(
        std::is_standard_layout_v<Trianglef>);

} // namespace kairo::foundation::geometry