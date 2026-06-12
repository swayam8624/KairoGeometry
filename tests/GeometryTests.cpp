#include <cmath>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string_view>

import Kairo.Foundation.Geometry;
import Kairo.Foundation.Math;

using namespace kairo::foundation::geometry;
using namespace kairo::foundation::math;

namespace
{
    constexpr float Epsilon = 1.0e-4f;

    int gFailures = 0;
    int gChecks = 0;

    void Require(bool condition, std::string_view message)
    {
        ++gChecks;
        if (!condition)
        {
            ++gFailures;
            std::cerr << "[FAIL] " << message << '\n';
        }
    }

    void RequireNear(float actual, float expected, std::string_view message)
    {
        Require(
            NearlyEqual(actual, expected, Epsilon),
            message);
    }

    void RunSuite(std::string_view name, void (*suite)())
    {
        const int checksBefore =
            gChecks;

        const auto start =
            std::chrono::steady_clock::now();

        std::cout << "[ RUN      ] " << name << '\n';
        suite();

        const auto end =
            std::chrono::steady_clock::now();

        const auto microseconds =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        std::cout << "[     DONE ] " << name
                  << " checks=" << (gChecks - checksBefore)
                  << " time=" << microseconds << "us\n";
    }

    void RequireNear(const Vec3f& actual, const Vec3f& expected, std::string_view message)
    {
        Require(
            NearlyEqual(actual, expected, Epsilon),
            message);
    }

    void TestRay()
    {
        const Rayf ray =
            Rayf::FromOriginDirection(
                Vec3f(1.0f, 2.0f, 3.0f),
                Vec3f::UnitX());

        RequireNear(ray.GetPoint(4.0f), Vec3f(5.0f, 2.0f, 3.0f), "ray point evaluation");
        RequireNear(ClosestParameter(ray, Vec3f(3.0f, 5.0f, 3.0f)), 2.0f, "ray closest parameter clamps forward");
        RequireNear(ClosestParameter(ray, Vec3f(-4.0f, 5.0f, 3.0f)), 0.0f, "ray closest parameter clamps behind origin");
        RequireNear(Distance(ray, Vec3f(3.0f, 5.0f, 3.0f)), 3.0f, "ray point distance");
        Require(NormalizedRay(Rayf(Vec3f::Zero(), Vec3f(10.0f, 0.0f, 0.0f))).IsDirectionNormalized(), "normalized ray direction");
    }

    void TestLineAndSegment()
    {
        const Linef line =
            Linef::FromPointDirection(
                Vec3f::Zero(),
                Vec3f::UnitY());

        RequireNear(line.ClosestPoint(Vec3f(3.0f, 2.0f, 0.0f)), Vec3f(0.0f, 2.0f, 0.0f), "line closest point");
        RequireNear(Distance(line, Vec3f(3.0f, 2.0f, 0.0f)), 3.0f, "line point distance");

        const Segmentf segment =
            Segmentf::FromEndpoints(
                Vec3f::Zero(),
                Vec3f(2.0f, 0.0f, 0.0f));

        RequireNear(segment.GetPoint(0.5f), Vec3f(1.0f, 0.0f, 0.0f), "segment point evaluation");
        RequireNear(segment.ClosestPoint(Vec3f(3.0f, 1.0f, 0.0f)), Vec3f(2.0f, 0.0f, 0.0f), "segment closest point clamps endpoint");
        RequireNear(Distance(segment, Vec3f(1.0f, 2.0f, 0.0f)), 2.0f, "segment point distance");

        const Segmentf a =
            Segmentf::FromEndpoints(
                Vec3f(0.0f, 0.0f, 0.0f),
                Vec3f(2.0f, 0.0f, 0.0f));

        const Segmentf b =
            Segmentf::FromEndpoints(
                Vec3f(1.0f, 1.0f, 0.0f),
                Vec3f(1.0f, 3.0f, 0.0f));

        const SegmentClosestResultf result =
            ClosestPoints(a, b);

        RequireNear(result.PointA, Vec3f(1.0f, 0.0f, 0.0f), "segment-segment closest point A");
        RequireNear(result.PointB, Vec3f(1.0f, 1.0f, 0.0f), "segment-segment closest point B");
        RequireNear(result.DistanceSquared, 1.0f, "segment-segment distance squared");
        RequireNear(DistanceBetweenSegments(a, b), 1.0f, "segment-segment distance helper");
    }

    void TestPlaneSphereTriangle()
    {
        const Planef ground =
            Planef::XZ();

        RequireNear(ground.SignedDistanceToPoint(Vec3f(0.0f, 3.0f, 0.0f)), 3.0f, "plane signed distance");
        RequireNear(ground.ProjectPoint(Vec3f(1.0f, 3.0f, 2.0f)), Vec3f(1.0f, 0.0f, 2.0f), "plane projection");
        RequireNear(ground.ReflectPoint(Vec3f(1.0f, 3.0f, 2.0f)), Vec3f(1.0f, -3.0f, 2.0f), "plane point reflection");

        const Spheref sphere =
            Spheref::FromCenterRadius(
                Vec3f::Zero(),
                2.0f);

        Require(sphere.ContainsPoint(Vec3f(1.0f, 1.0f, 0.0f)), "sphere contains point");
        Require(!sphere.ContainsPoint(Vec3f(3.0f, 0.0f, 0.0f)), "sphere rejects outside point");
        RequireNear(sphere.ClosestPoint(Vec3f(5.0f, 0.0f, 0.0f)), Vec3f(2.0f, 0.0f, 0.0f), "sphere closest point");
        RequireNear(sphere.SignedDistanceToPoint(Vec3f(5.0f, 0.0f, 0.0f)), 3.0f, "sphere signed distance");

        const Trianglef triangle =
            Trianglef::UnitRightTriangleXY();

        RequireNear(triangle.Area(), 0.5f, "triangle area");
        RequireNear(triangle.Centroid(), Vec3f(1.0f / 3.0f, 1.0f / 3.0f, 0.0f), "triangle centroid");
        RequireNear(triangle.Barycentric(Vec3f(0.25f, 0.25f, 0.0f)), Vec3f(0.5f, 0.25f, 0.25f), "triangle barycentric coordinates");
        Require(triangle.ContainsPoint(Vec3f(0.25f, 0.25f, 0.0f)), "triangle contains point");
        RequireNear(ClosestPointOnTriangle(triangle, Vec3f(2.0f, 2.0f, 0.0f)), Vec3f(0.5f, 0.5f, 0.0f), "triangle closest point");
    }

    void TestAABB()
    {
        AABBf box =
            AABBf::FromCenterExtent(
                Vec3f::Zero(),
                Vec3f(1.0f, 2.0f, 3.0f));

        Require(box.IsValid(), "AABB valid from center/extent");
        RequireNear(box.Center(), Vec3f::Zero(), "AABB center");
        RequireNear(box.Size(), Vec3f(2.0f, 4.0f, 6.0f), "AABB size");
        RequireNear(box.Volume(), 48.0f, "AABB volume");
        Require(box.ContainsPoint(Vec3f(0.5f, 1.5f, 2.5f)), "AABB contains point");
        Require(!box.ContainsPoint(Vec3f(2.0f, 0.0f, 0.0f)), "AABB rejects outside point");
        RequireNear(box.ClosestPoint(Vec3f(4.0f, 0.0f, 0.0f)), Vec3f(1.0f, 0.0f, 0.0f), "AABB closest point");

        const AABBf other =
            AABBf::FromMinMax(
                Vec3f(0.5f, -1.0f, -1.0f),
                Vec3f(2.0f, 1.0f, 1.0f));

        Require(Intersects(box, other), "AABB intersects AABB");
        const AABBf overlap =
            Intersection(box, other);
        RequireNear(overlap.Min, Vec3f(0.5f, -1.0f, -1.0f), "AABB intersection min");
        RequireNear(overlap.Max, Vec3f(1.0f, 1.0f, 1.0f), "AABB intersection max");

        box.ExpandToInclude(Vec3f(4.0f, 0.0f, 0.0f));
        RequireNear(box.Max.x, 4.0f, "AABB expand to include point");
    }

    void TestCapsuleOBB()
    {
        const Capsulef capsule =
            Capsulef::FromEndpointsRadius(
                Vec3f(0.0f, -1.0f, 0.0f),
                Vec3f(0.0f, 1.0f, 0.0f),
                0.5f);

        RequireNear(capsule.AxisLength(), 2.0f, "capsule axis length");
        Require(capsule.ContainsPoint(Vec3f(0.25f, 0.0f, 0.0f)), "capsule contains point");
        RequireNear(capsule.SignedDistanceToPoint(Vec3f(1.5f, 0.0f, 0.0f)), 1.0f, "capsule signed distance");
        RequireNear(capsule.ClosestPointOnAxis(Vec3f(2.0f, 0.25f, 0.0f)), Vec3f(0.0f, 0.25f, 0.0f), "capsule closest axis point");

        const Capsulef other =
            Capsulef::FromEndpointsRadius(
                Vec3f(0.75f, -1.0f, 0.0f),
                Vec3f(0.75f, 1.0f, 0.0f),
                0.5f);

        Require(CapsuleCapsule(capsule, other), "capsule intersects capsule");

        const OBBf box =
            OBBf::FromCenterRotationHalfExtents(
                Vec3f::Zero(),
                RotationAroundZ(0.25f),
                Vec3f(1.0f, 2.0f, 0.5f));

        Require(box.IsValid(), "OBB valid");
        Require(box.ContainsPoint(Vec3f::Zero()), "OBB contains center");
        RequireNear(box.LocalToWorldPoint(Vec3f::Zero()), box.Center, "OBB local origin to center");
        RequireNear(box.WorldToLocalPoint(box.Center), Vec3f::Zero(), "OBB center to local origin");
        RequireNear(box.ClosestPoint(Vec3f(5.0f, 0.0f, 0.0f)).z, 0.0f, "OBB closest point keeps z plane");
        Require(Intersects(box, OBBf::FromAABB(AABBf::Unit())), "OBB intersects unit AABB as OBB");
    }

    void TestIntersections()
    {
        const Rayf ray =
            Rayf::FromOriginDirection(
                Vec3f(-4.0f, 0.0f, 0.0f),
                Vec3f::UnitX());

        const Planef plane =
            Planef::YZ();

        const std::optional<RayHitf> planeHit =
            RayPlane(ray, plane);

        Require(planeHit.has_value(), "ray hits plane");
        RequireNear(planeHit->Distance, 4.0f, "ray-plane hit distance");

        const Spheref sphere =
            Spheref::FromCenterRadius(
                Vec3f::Zero(),
                1.0f);

        const std::optional<RayHitf> sphereHit =
            RaySphere(ray, sphere);

        Require(sphereHit.has_value(), "ray hits sphere");
        RequireNear(sphereHit->Distance, 3.0f, "ray-sphere hit distance");
        Require(Intersects(sphere, AABBf::Unit()), "sphere intersects AABB");

        const Trianglef triangle =
            Trianglef::UnitRightTriangleXY();

        const std::optional<RayHitf> triangleHit =
            RayTriangle(
                Rayf::FromOriginDirection(
                    Vec3f(0.25f, 0.25f, -1.0f),
                    Vec3f::UnitZ()),
                triangle);

        Require(triangleHit.has_value(), "ray hits triangle");
        RequireNear(triangleHit->Point, Vec3f(0.25f, 0.25f, 0.0f), "ray-triangle hit point");

        const AABBf box =
            AABBf::Unit();

        const std::optional<RayHitf> boxHit =
            RayAABB(ray, box);

        Require(boxHit.has_value(), "ray hits AABB");
        RequireNear(boxHit->Distance, 3.5f, "ray-AABB hit distance");

        const OBBf obb =
            OBBf::FromAABB(box);

        const std::optional<RayHitf> obbHit =
            RayOBB(ray, obb);

        Require(obbHit.has_value(), "ray hits OBB");
        Require(Intersects(obb, box), "OBB intersects AABB");
        Require(Intersects(obb, sphere), "OBB intersects sphere");

        const Capsulef capsule =
            Capsulef::FromEndpointsRadius(
                Vec3f(0.0f, -1.0f, 0.0f),
                Vec3f(0.0f, 1.0f, 0.0f),
                0.5f);

        const std::optional<RayHitf> capsuleHit =
            RayCapsule(ray, capsule);

        Require(capsuleHit.has_value(), "ray hits capsule");
        RequireNear(capsuleHit->Distance, 3.5f, "ray-capsule hit distance");

        Require(PlaneOBB(plane, obb), "plane intersects OBB");
        Require(!PlaneOBB(Planef::FromPointNormal(Vec3f(3.0f, 0.0f, 0.0f), Vec3f::UnitX()), obb), "separated plane misses OBB");
        Require(PlaneCapsule(Planef::XZ(), capsule), "plane intersects capsule");

        Require(PlaneTriangle(Planef::XY(), triangle), "plane intersects triangle");
        Require(SphereTriangle(Spheref::FromCenterRadius(Vec3f(0.25f, 0.25f, 0.2f), 0.25f), triangle), "sphere intersects triangle");
        Require(!SphereTriangle(Spheref::FromCenterRadius(Vec3f(4.0f, 4.0f, 4.0f), 0.25f), triangle), "sphere misses triangle");

        const Capsulef nearBoxCapsule =
            Capsulef::FromEndpointsRadius(
                Vec3f(-0.25f, 0.75f, 0.0f),
                Vec3f(0.25f, 0.75f, 0.0f),
                0.3f);

        Require(AABBCapsule(box, nearBoxCapsule), "AABB intersects capsule");
        Require(OBBCapsule(obb, nearBoxCapsule), "OBB intersects capsule");
        Require(!AABBCapsule(box, Capsulef::FromEndpointsRadius(Vec3f(3.0f, 3.0f, 3.0f), Vec3f(4.0f, 3.0f, 3.0f), 0.1f)), "AABB misses capsule");

        const AABBf thinBox =
            AABBf::FromMinMax(
                Vec3f(0.2f, 0.2f, -0.1f),
                Vec3f(0.4f, 0.4f, 0.1f));

        Require(TriangleAABB(triangle, thinBox), "triangle intersects AABB");
        Require(TriangleOBB(triangle, OBBf::FromAABB(thinBox)), "triangle intersects OBB");
        Require(TriangleCapsule(triangle, Capsulef::FromEndpointsRadius(Vec3f(0.25f, 0.25f, -1.0f), Vec3f(0.25f, 0.25f, 1.0f), 0.05f)), "triangle intersects capsule");

        const Trianglef coplanarTriangle =
            Trianglef::FromPoints(
                Vec3f(0.25f, 0.25f, 0.0f),
                Vec3f(1.0f, 0.25f, 0.0f),
                Vec3f(0.25f, 1.0f, 0.0f));

        Require(TriangleTriangle(triangle, coplanarTriangle), "coplanar triangles intersect");
        Require(Intersects(capsule, triangle), "generic capsule triangle intersection");
        Require(Intersects(triangle, box), "generic triangle AABB intersection");
    }

    void TestFrustum()
    {
        const Frustumf frustum =
            Frustumf::FromViewProjection(
                Mat4f::Identity());

        Require(frustum.IsValid(), "frustum from identity matrix is valid");
        Require(frustum.ContainsPoint(Vec3f(0.0f, 0.0f, 0.5f)), "frustum contains clip-space center");
        Require(!frustum.ContainsPoint(Vec3f(2.0f, 0.0f, 0.5f)), "frustum rejects outside point");
        Require(frustum.IntersectsSphere(Spheref::FromCenterRadius(Vec3f::Zero(), 0.25f)), "frustum intersects sphere");
        Require(frustum.IntersectsAABB(AABBf::Unit()), "frustum intersects AABB");
        Require(frustum.IntersectsOBB(OBBf::FromAABB(AABBf::Unit())), "frustum intersects OBB");

        std::array<Vec3f, 8> corners {};
        Require(frustum.GetCorners(corners), "frustum corners can be reconstructed");
        RequireNear(corners[0], Vec3f(-1.0f, -1.0f, 0.0f), "frustum left-bottom-near corner");
        RequireNear(corners[7], Vec3f(1.0f, 1.0f, 1.0f), "frustum right-top-far corner");
    }

    void TestDeterministicStress()
    {
        constexpr int Iterations = 50000;

        float accumulator =
            0.0f;

        for (int i = 0; i < Iterations; ++i)
        {
            const float t =
                static_cast<float>(i % 997) / 997.0f;

            const Vec3f point
            {
                std::sin(t * 19.0f) * 2.0f,
                std::cos(t * 23.0f) * 1.5f,
                std::sin(t * 7.0f) * 0.75f
            };

            const AABBf box =
                AABBf::FromCenterExtent(
                    Vec3f(0.1f, -0.2f, 0.0f),
                    Vec3f(1.0f, 1.25f, 0.5f));

            const Spheref sphere =
                Spheref::FromCenterRadius(
                    Vec3f(0.25f, 0.0f, 0.0f),
                    0.9f);

            const Segmentf segment =
                Segmentf::FromEndpoints(
                    Vec3f(-1.0f, -0.75f, 0.0f),
                    Vec3f(1.25f, 0.8f, 0.0f));

            accumulator +=
                DistanceToAABB(point, box) +
                DistanceToSphere(point, sphere) +
                DistanceToSegment(point, segment);
        }

        Require(accumulator > 0.0f, "deterministic geometry stress accumulator is positive");
        std::cout << "[   STRESS ] iterations=" << Iterations
                  << " accumulator=" << accumulator << '\n';
    }
}

int main()
{
    const auto start =
        std::chrono::steady_clock::now();

    RunSuite("Ray", TestRay);
    RunSuite("LineAndSegment", TestLineAndSegment);
    RunSuite("PlaneSphereTriangle", TestPlaneSphereTriangle);
    RunSuite("AABB", TestAABB);
    RunSuite("CapsuleOBB", TestCapsuleOBB);
    RunSuite("Intersections", TestIntersections);
    RunSuite("Frustum", TestFrustum);
    RunSuite("DeterministicStress", TestDeterministicStress);

    const auto end =
        std::chrono::steady_clock::now();

    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    if (gFailures != 0)
    {
        std::cerr << gFailures << " geometry test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "All KairoGeometry tests passed. checks=" << gChecks
              << " elapsed=" << milliseconds << "ms\n";
    return EXIT_SUCCESS;
}
