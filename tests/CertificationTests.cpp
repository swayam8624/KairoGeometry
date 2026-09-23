#include <cstdint>
#include <cstdlib>
#include <iostream>

import Kairo.Foundation.Geometry;
import Kairo.Foundation.Math;

using namespace kairo::foundation::geometry;
using namespace kairo::foundation::math;

namespace
{
    std::uint32_t g_state = 0x91E10DA5u;

    float NextFloat(float minimum, float maximum)
    {
        g_state = g_state * 1664525u + 1013904223u;
        const float unit = static_cast<float>(g_state & 0x00FFFFFFu) /
            static_cast<float>(0x01000000u);
        return minimum + (maximum - minimum) * unit;
    }

    [[noreturn]] void Fail(const char* message)
    {
        std::cerr << "[KairoGeometry certification] " << message << '\n';
        std::exit(2);
    }
}

int main()
{
    constexpr int iterations = 25000;
    for (int index = 0; index < iterations; ++index)
    {
        const Vec3f sphereCenter(
            NextFloat(-20.0f, 20.0f),
            NextFloat(-20.0f, 20.0f),
            NextFloat(-20.0f, 20.0f));
        const Spheref sphere = Spheref::FromCenterRadius(
            sphereCenter, NextFloat(0.01f, 5.0f));

        const Vec3f boxCenter(
            NextFloat(-20.0f, 20.0f),
            NextFloat(-20.0f, 20.0f),
            NextFloat(-20.0f, 20.0f));
        const AABBf box = AABBf::FromCenterExtent(
            boxCenter,
            Vec3f(
                NextFloat(0.01f, 4.0f),
                NextFloat(0.01f, 4.0f),
                NextFloat(0.01f, 4.0f)));

        if (Intersects(sphere, box) != Intersects(box, sphere))
            Fail("sphere/AABB symmetry violated");

        const Capsulef capsule = Capsulef::FromEndpointsRadius(
            Vec3f(
                NextFloat(-10.0f, 10.0f),
                NextFloat(-10.0f, 10.0f),
                NextFloat(-10.0f, 10.0f)),
            Vec3f(
                NextFloat(-10.0f, 10.0f),
                NextFloat(-10.0f, 10.0f),
                NextFloat(-10.0f, 10.0f)),
            NextFloat(0.01f, 2.0f));

        if (Intersects(sphere, capsule) != Intersects(capsule, sphere))
            Fail("sphere/capsule symmetry violated");

        const OBBf oriented = OBBf::FromAABB(box);
        if (Intersects(oriented, sphere) != Intersects(sphere, oriented))
            Fail("sphere/OBB symmetry violated");

        const Vec3f point(
            NextFloat(-25.0f, 25.0f),
            NextFloat(-25.0f, 25.0f),
            NextFloat(-25.0f, 25.0f));
        const Vec3f closest = box.ClosestPoint(point);
        if (!box.ContainsPoint(closest))
            Fail("AABB closest point escaped its source box");
    }

    std::cout
        << "{\"schema\":\"kairo.geometry.certification.v1\","
        << "\"iterations\":" << iterations << ","
        << "\"seed\":2447449509,"
        << "\"status\":\"pass\"}\n";
    return 0;
}
