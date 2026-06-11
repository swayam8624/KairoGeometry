import Kairo.Foundation.Geometry;
import Kairo.Foundation.Math;

#include <cassert>

int main()
{
    using namespace kairo::foundation::geometry;
    using namespace kairo::foundation::math;

    const Rayf ray =
        Rayf::FromOriginDirection(
            Vec3f::Zero(),
            Vec3f::UnitX());

    assert(NearlyEqual(ray.GetPoint(3.0f), Vec3f(3.0f, 0.0f, 0.0f)));
    assert(NearlyEqual(ClosestPoint(ray, Vec3f(2.0f, 5.0f, 0.0f)), Vec3f(2.0f, 0.0f, 0.0f)));

    const Planef ground =
        Planef::XZ();

    assert(NearlyEqual(ground.SignedDistanceToPoint(Vec3f(0.0f, 2.0f, 0.0f)), 2.0f));
    assert(NearlyEqual(ground.ProjectPoint(Vec3f(1.0f, 2.0f, 3.0f)), Vec3f(1.0f, 0.0f, 3.0f)));

    const Spheref unitSphere =
        Spheref::Unit();

    assert(unitSphere.ContainsPoint(Vec3f(0.25f, 0.25f, 0.25f)));
    assert(Intersects(unitSphere, Spheref::FromCenterRadius(Vec3f(1.5f, 0.0f, 0.0f), 0.75f)));

    const Trianglef triangle =
        Trianglef::UnitRightTriangleXY();

    assert(NearlyEqual(triangle.Area(), 0.5f));
    assert(triangle.ContainsPoint(Vec3f(0.25f, 0.25f, 0.0f)));
    assert(!triangle.ContainsPoint(Vec3f(1.25f, 0.25f, 0.0f)));

    const AABBf unitBox =
        AABBf::Unit();

    assert(unitBox.ContainsPoint(Vec3f(0.25f, 0.25f, 0.25f)));
    assert(NearlyEqual(unitBox.ClosestPoint(Vec3f(2.0f, 0.0f, 0.0f)), Vec3f(0.5f, 0.0f, 0.0f)));

    const Capsulef capsule =
        Capsulef::FromEndpointsRadius(
            Vec3f(0.0f, -1.0f, 0.0f),
            Vec3f(0.0f, 1.0f, 0.0f),
            0.5f);

    assert(capsule.ContainsPoint(Vec3f(0.25f, 0.0f, 0.0f)));
    assert(NearlyEqual(DistanceToCapsule(Vec3f(1.5f, 0.0f, 0.0f), capsule), 1.0f));
    assert(Intersects(Spheref::FromCenterRadius(Vec3f(0.75f, 0.0f, 0.0f), 0.25f), capsule));

    const OBBf obb =
        OBBf::FromAABB(unitBox);

    assert(obb.ContainsPoint(Vec3f(0.25f, 0.25f, 0.25f)));
    assert(NearlyEqual(ClosestPointOnOBB(obb, Vec3f(2.0f, 0.0f, 0.0f)), Vec3f(0.5f, 0.0f, 0.0f)));
    assert(Intersects(unitBox, obb));

    const Rayf obbRay =
        Rayf::FromOriginDirection(
            Vec3f(-2.0f, 0.0f, 0.0f),
            Vec3f::UnitX());

    const auto obbHit =
        RayOBB(obbRay, obb);

    assert(obbHit.has_value());
    assert(NearlyEqual(obbHit->Distance, 1.5f));
    assert(NearlyEqual(obbHit->Point, Vec3f(-0.5f, 0.0f, 0.0f)));

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

    assert(NearlyEqual(segmentResult.DistanceSquared, 1.0f));
    assert(NearlyEqual(DistanceBetweenSegments(segmentA, segmentB), 1.0f));

    const Linef line =
        Linef::FromPointDirection(
            Vec3f::Zero(),
            Vec3f::UnitY());

    assert(NearlyEqual(ClosestPointOnLine(line, Vec3f(2.0f, 3.0f, 0.0f)), Vec3f(0.0f, 3.0f, 0.0f)));

    const Capsulef nearbyCapsule =
        Capsulef::FromEndpointsRadius(
            Vec3f(0.75f, -1.0f, 0.0f),
            Vec3f(0.75f, 1.0f, 0.0f),
            0.5f);

    assert(CapsuleCapsule(capsule, nearbyCapsule));

    const Frustumf clipFrustum =
        Frustumf::FromViewProjection(
            Mat4f::Identity());

    assert(clipFrustum.ContainsPoint(Vec3f(0.0f, 0.0f, 0.5f)));
    assert(!clipFrustum.ContainsPoint(Vec3f(2.0f, 0.0f, 0.5f)));
    assert(clipFrustum.IntersectsSphere(Spheref::FromCenterRadius(Vec3f::Zero(), 0.25f)));
    assert(clipFrustum.IntersectsAABB(unitBox));
    assert(clipFrustum.IntersectsOBB(obb));

    return 0;
}
