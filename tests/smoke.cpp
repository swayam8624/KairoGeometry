import Kairo.Foundation.Geometry;
import Kairo.Foundation.Math;

#define KAIRO_REQUIRE(expression) do { if (!(expression)) return __LINE__; } while (false)

int main()
{
    using namespace kairo::foundation::geometry;
    using namespace kairo::foundation::math;

    const Rayf ray =
        Rayf::FromOriginDirection(
            Vec3f::Zero(),
            Vec3f::UnitX());

    KAIRO_REQUIRE(NearlyEqual(ray.GetPoint(3.0f), Vec3f(3.0f, 0.0f, 0.0f)));
    KAIRO_REQUIRE(NearlyEqual(ClosestPoint(ray, Vec3f(2.0f, 5.0f, 0.0f)), Vec3f(2.0f, 0.0f, 0.0f)));

    const Planef ground =
        Planef::XZ();

    KAIRO_REQUIRE(NearlyEqual(ground.SignedDistanceToPoint(Vec3f(0.0f, 2.0f, 0.0f)), 2.0f));
    KAIRO_REQUIRE(NearlyEqual(ground.ProjectPoint(Vec3f(1.0f, 2.0f, 3.0f)), Vec3f(1.0f, 0.0f, 3.0f)));

    const Spheref unitSphere =
        Spheref::Unit();

    KAIRO_REQUIRE(unitSphere.ContainsPoint(Vec3f(0.25f, 0.25f, 0.25f)));
    KAIRO_REQUIRE(Intersects(unitSphere, Spheref::FromCenterRadius(Vec3f(1.5f, 0.0f, 0.0f), 0.75f)));

    const Trianglef triangle =
        Trianglef::UnitRightTriangleXY();

    KAIRO_REQUIRE(NearlyEqual(triangle.Area(), 0.5f));
    KAIRO_REQUIRE(triangle.ContainsPoint(Vec3f(0.25f, 0.25f, 0.0f)));
    KAIRO_REQUIRE(!triangle.ContainsPoint(Vec3f(1.25f, 0.25f, 0.0f)));

    const AABBf unitBox =
        AABBf::Unit();

    KAIRO_REQUIRE(unitBox.ContainsPoint(Vec3f(0.25f, 0.25f, 0.25f)));
    KAIRO_REQUIRE(NearlyEqual(unitBox.ClosestPoint(Vec3f(2.0f, 0.0f, 0.0f)), Vec3f(0.5f, 0.0f, 0.0f)));

    const Capsulef capsule =
        Capsulef::FromEndpointsRadius(
            Vec3f(0.0f, -1.0f, 0.0f),
            Vec3f(0.0f, 1.0f, 0.0f),
            0.5f);

    KAIRO_REQUIRE(capsule.ContainsPoint(Vec3f(0.25f, 0.0f, 0.0f)));
    KAIRO_REQUIRE(NearlyEqual(DistanceToCapsule(Vec3f(1.5f, 0.0f, 0.0f), capsule), 1.0f));
    KAIRO_REQUIRE(Intersects(Spheref::FromCenterRadius(Vec3f(0.75f, 0.0f, 0.0f), 0.25f), capsule));

    const OBBf obb =
        OBBf::FromAABB(unitBox);

    KAIRO_REQUIRE(obb.ContainsPoint(Vec3f(0.25f, 0.25f, 0.25f)));
    KAIRO_REQUIRE(NearlyEqual(ClosestPointOnOBB(obb, Vec3f(2.0f, 0.0f, 0.0f)), Vec3f(0.5f, 0.0f, 0.0f)));
    KAIRO_REQUIRE(Intersects(unitBox, obb));

    const Rayf obbRay =
        Rayf::FromOriginDirection(
            Vec3f(-2.0f, 0.0f, 0.0f),
            Vec3f::UnitX());

    const auto obbHit =
        RayOBB(obbRay, obb);

    KAIRO_REQUIRE(obbHit.has_value());
    KAIRO_REQUIRE(NearlyEqual(obbHit->Distance, 1.5f));
    KAIRO_REQUIRE(NearlyEqual(obbHit->Point, Vec3f(-0.5f, 0.0f, 0.0f)));

    const Segmentf segmentA =
        Segmentf::FromEndpoints(
            Vec3f(0.0f, 0.0f, 0.0f),
            Vec3f(1.0f, 0.0f, 0.0f));

    const Segmentf segmentB =
        Segmentf::FromEndpoints(
            Vec3f(0.0f, 1.0f, 0.0f),
            Vec3f(1.0f, 1.0f, 0.0f));

    const SegmentClosestResultf segmentResult =
        ClosestPoints(segmentA, segmentB);

    KAIRO_REQUIRE(NearlyEqual(segmentResult.DistanceSquared, 1.0f));
    KAIRO_REQUIRE(NearlyEqual(DistanceBetweenSegments(segmentA, segmentB), 1.0f));

    const Linef line =
        Linef::FromPointDirection(
            Vec3f::Zero(),
            Vec3f::UnitY());

    KAIRO_REQUIRE(NearlyEqual(ClosestPointOnLine(line, Vec3f(2.0f, 3.0f, 0.0f)), Vec3f(0.0f, 3.0f, 0.0f)));

    const Capsulef nearbyCapsule =
        Capsulef::FromEndpointsRadius(
            Vec3f(0.75f, -1.0f, 0.0f),
            Vec3f(0.75f, 1.0f, 0.0f),
            0.5f);

    KAIRO_REQUIRE(CapsuleCapsule(capsule, nearbyCapsule));

    const Frustumf clipFrustum =
        Frustumf::FromViewProjection(
            Mat4f::Identity());

    KAIRO_REQUIRE(clipFrustum.ContainsPoint(Vec3f(0.0f, 0.0f, 0.5f)));
    KAIRO_REQUIRE(!clipFrustum.ContainsPoint(Vec3f(2.0f, 0.0f, 0.5f)));
    KAIRO_REQUIRE(clipFrustum.IntersectsSphere(Spheref::FromCenterRadius(Vec3f::Zero(), 0.25f)));
    KAIRO_REQUIRE(clipFrustum.IntersectsAABB(unitBox));
    KAIRO_REQUIRE(clipFrustum.IntersectsOBB(obb));

    return 0;
}
