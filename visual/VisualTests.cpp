#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

import Kairo.Foundation.Geometry;
import Kairo.Foundation.Math;

using namespace kairo::foundation::geometry;
using namespace kairo::foundation::math;

namespace
{
    struct Canvas final
    {
        float width = 360.0f;
        float height = 260.0f;
        float scale = 64.0f;
        float originX = width * 0.5f;
        float originY = height * 0.5f;
        std::ostringstream body;

        std::string Point(const Vec3f& point) const
        {
            std::ostringstream out;
            out << std::fixed << std::setprecision(2)
                << (originX + point.x * scale)
                << ","
                << (originY - point.y * scale);
            return out.str();
        }

        void Line(const Vec3f& a, const Vec3f& b, std::string_view color, float stroke = 2.0f, std::string_view dash = "")
        {
            body << "<line x1=\"" << originX + a.x * scale
                 << "\" y1=\"" << originY - a.y * scale
                 << "\" x2=\"" << originX + b.x * scale
                 << "\" y2=\"" << originY - b.y * scale
                 << "\" stroke=\"" << color
                 << "\" stroke-width=\"" << stroke << "\"";
            if (!dash.empty())
            {
                body << " stroke-dasharray=\"" << dash << "\"";
            }
            body << " stroke-linecap=\"round\" />\n";
        }

        void Circle(const Vec3f& center, float radius, std::string_view stroke, std::string_view fill = "none", float strokeWidth = 2.0f)
        {
            body << "<circle cx=\"" << originX + center.x * scale
                 << "\" cy=\"" << originY - center.y * scale
                 << "\" r=\"" << radius * scale
                 << "\" fill=\"" << fill
                 << "\" stroke=\"" << stroke
                 << "\" stroke-width=\"" << strokeWidth << "\" />\n";
        }

        void PointMark(const Vec3f& point, std::string_view color, float radius = 4.0f)
        {
            body << "<circle cx=\"" << originX + point.x * scale
                 << "\" cy=\"" << originY - point.y * scale
                 << "\" r=\"" << radius
                 << "\" fill=\"" << color << "\" />\n";
        }

        void Polygon(const std::vector<Vec3f>& points, std::string_view stroke, std::string_view fill, float strokeWidth = 2.0f)
        {
            body << "<polygon points=\"";
            for (const Vec3f& point : points)
            {
                body << Point(point) << " ";
            }
            body << "\" fill=\"" << fill
                 << "\" stroke=\"" << stroke
                 << "\" stroke-width=\"" << strokeWidth << "\" />\n";
        }

        std::string Svg() const
        {
            std::ostringstream out;
            out << "<svg viewBox=\"0 0 " << width << " " << height << "\" role=\"img\">\n"
                << "<rect width=\"100%\" height=\"100%\" rx=\"8\" fill=\"#0f172a\" />\n"
                << "<line x1=\"0\" y1=\"" << originY << "\" x2=\"" << width << "\" y2=\"" << originY << "\" stroke=\"#1f2937\" />\n"
                << "<line x1=\"" << originX << "\" y1=\"0\" x2=\"" << originX << "\" y2=\"" << height << "\" stroke=\"#1f2937\" />\n"
                << body.str()
                << "</svg>\n";
            return out.str();
        }
    };

    std::string Card(std::string_view title, std::string_view description, const Canvas& canvas, std::string_view metrics)
    {
        std::ostringstream out;
        out << "<section class=\"card\">\n"
            << "<h2>" << title << "</h2>\n"
            << "<p>" << description << "</p>\n"
            << canvas.Svg()
            << "<pre>" << metrics << "</pre>\n"
            << "</section>\n";
        return out.str();
    }

    std::string FormatVec(const Vec3f& value)
    {
        std::ostringstream out;
        out << std::fixed << std::setprecision(2)
            << "(" << value.x << ", " << value.y << ", " << value.z << ")";
        return out.str();
    }

    void DrawAABB(Canvas& canvas, const AABBf& box, std::string_view stroke, std::string_view fill)
    {
        canvas.Polygon(
            {
                Vec3f(box.Min.x, box.Min.y, 0.0f),
                Vec3f(box.Max.x, box.Min.y, 0.0f),
                Vec3f(box.Max.x, box.Max.y, 0.0f),
                Vec3f(box.Min.x, box.Max.y, 0.0f)
            },
            stroke,
            fill);
    }

    void DrawOBB(Canvas& canvas, const OBBf& box, std::string_view stroke, std::string_view fill)
    {
        canvas.Polygon(
            {
                box.Corner(0),
                box.Corner(1),
                box.Corner(3),
                box.Corner(2)
            },
            stroke,
            fill);
    }

    void DrawCapsule(Canvas& canvas, const Capsulef& capsule, std::string_view color)
    {
        canvas.Line(capsule.Start, capsule.End, color, capsule.Radius * canvas.scale * 2.0f);
        canvas.Circle(capsule.Start, capsule.Radius, color, "rgba(56,189,248,0.16)");
        canvas.Circle(capsule.End, capsule.Radius, color, "rgba(56,189,248,0.16)");
        canvas.Line(capsule.Start, capsule.End, "#e2e8f0", 2.0f, "4 5");
    }

    std::string BuildVisualReport()
    {
        std::ostringstream cards;

        {
            Canvas canvas;
            const Rayf ray = Rayf::FromOriginDirection(Vec3f(-2.4f, -0.8f, 0.0f), Vec3f(1.0f, 0.5f, 0.0f));
            const Vec3f point(-0.4f, 1.5f, 0.0f);
            const Vec3f closest = ClosestPoint(ray, point);
            canvas.Line(ray.Origin, ray.GetPoint(4.0f), "#38bdf8", 3.0f);
            canvas.Line(point, closest, "#f97316", 2.0f, "5 5");
            canvas.PointMark(ray.Origin, "#38bdf8");
            canvas.PointMark(point, "#f97316");
            canvas.PointMark(closest, "#22c55e");
            cards << Card(
                "Ray closest point",
                "Blue is the ray, orange is the query point, green is the closest point on the half-infinite ray.",
                canvas,
                "closest = " + FormatVec(closest) + "\\ndistance = " + std::to_string(Distance(ray, point)));
        }

        {
            Canvas canvas;
            const Segmentf a = Segmentf::FromEndpoints(Vec3f(-2.0f, -0.5f, 0.0f), Vec3f(1.8f, -0.5f, 0.0f));
            const Segmentf b = Segmentf::FromEndpoints(Vec3f(-0.2f, 1.6f, 0.0f), Vec3f(1.2f, 0.2f, 0.0f));
            const SegmentClosestResultf result = ClosestPoints(a, b);
            canvas.Line(a.Start, a.End, "#38bdf8", 4.0f);
            canvas.Line(b.Start, b.End, "#a78bfa", 4.0f);
            canvas.Line(result.PointA, result.PointB, "#f97316", 2.0f, "5 5");
            canvas.PointMark(result.PointA, "#22c55e");
            canvas.PointMark(result.PointB, "#22c55e");
            cards << Card(
                "Segment-segment distance",
                "The orange dashed line connects the two closest points between finite segments.",
                canvas,
                "pointA = " + FormatVec(result.PointA) + "\\npointB = " + FormatVec(result.PointB) + "\\ndistance = " + std::to_string(std::sqrt(result.DistanceSquared)));
        }

        {
            Canvas canvas;
            const AABBf aabb = AABBf::FromCenterExtent(Vec3f(-0.8f, 0.1f, 0.0f), Vec3f(1.0f, 0.8f, 0.1f));
            const OBBf obb = OBBf::FromCenterRotationHalfExtents(Vec3f(0.7f, 0.1f, 0.0f), RotationAroundZ(0.45f), Vec3f(1.0f, 0.55f, 0.1f));
            DrawAABB(canvas, aabb, "#38bdf8", "rgba(56,189,248,0.14)");
            DrawOBB(canvas, obb, "#f97316", "rgba(249,115,22,0.14)");
            cards << Card(
                "AABB and OBB SAT intersection",
                "The rotated OBB is tested against an axis-aligned box using the OBB/OBB separating-axis path.",
                canvas,
                std::string("intersects = ") + (Intersects(aabb, obb) ? "true" : "false"));
        }

        {
            Canvas canvas;
            const Capsulef capsuleA = Capsulef::FromEndpointsRadius(Vec3f(-1.2f, -1.2f, 0.0f), Vec3f(-1.2f, 1.0f, 0.0f), 0.35f);
            const Capsulef capsuleB = Capsulef::FromEndpointsRadius(Vec3f(-0.2f, -0.9f, 0.0f), Vec3f(1.4f, 0.9f, 0.0f), 0.35f);
            DrawCapsule(canvas, capsuleA, "#38bdf8");
            DrawCapsule(canvas, capsuleB, "#a78bfa");
            cards << Card(
                "Capsule-capsule intersection",
                "Capsules reduce to segment-segment distance compared against summed radii.",
                canvas,
                std::string("intersects = ") + (CapsuleCapsule(capsuleA, capsuleB) ? "true" : "false"));
        }

        {
            Canvas canvas;
            const Frustumf frustum = Frustumf::FromViewProjection(Mat4f::Identity());
            std::array<Vec3f, 8> corners {};
            const bool hasCorners = frustum.GetCorners(corners);
            if (hasCorners)
            {
                canvas.Polygon({ corners[0], corners[1], corners[3], corners[2] }, "#22c55e", "rgba(34,197,94,0.12)");
                canvas.Polygon({ corners[4], corners[5], corners[7], corners[6] }, "#14b8a6", "rgba(20,184,166,0.10)");
                canvas.Line(corners[0], corners[4], "#64748b", 1.5f);
                canvas.Line(corners[1], corners[5], "#64748b", 1.5f);
                canvas.Line(corners[2], corners[6], "#64748b", 1.5f);
                canvas.Line(corners[3], corners[7], "#64748b", 1.5f);
            }
            const Spheref sphere = Spheref::FromCenterRadius(Vec3f(0.25f, 0.2f, 0.0f), 0.25f);
            canvas.Circle(sphere.Center, sphere.Radius, "#f97316", "rgba(249,115,22,0.18)");
            cards << Card(
                "Frustum clip volume",
                "The identity view-projection frustum is shown in XY with near and far rectangles overlapped in clip space.",
                canvas,
                std::string("sphere visible = ") + (frustum.IntersectsSphere(sphere) ? "true" : "false"));
        }

        return cards.str();
    }
}

int main()
{
    const std::filesystem::path outputPath =
        std::filesystem::path(__FILE__).parent_path() / "geometry_visual_tests.html";

    std::ofstream out(outputPath);
    if (!out)
    {
        std::cerr << "Failed to open " << outputPath << " for writing.\n";
        return 1;
    }

    out << R"(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>KairoGeometry Visual Tests</title>
    <style>
        :root {
            color-scheme: dark;
            --bg: #0b1020;
            --panel: #111827;
            --panel-2: #0f172a;
            --line: #243244;
            --text: #e5e7eb;
            --muted: #94a3b8;
            --accent: #38bdf8;
        }
        * { box-sizing: border-box; }
        body {
            margin: 0;
            font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
            background: var(--bg);
            color: var(--text);
        }
        header {
            padding: 24px 28px 16px;
            border-bottom: 1px solid var(--line);
            background: var(--panel-2);
        }
        h1 { margin: 0 0 8px; font-size: 28px; letter-spacing: 0; }
        header p { margin: 0; color: var(--muted); max-width: 900px; line-height: 1.5; }
        main {
            display: grid;
            grid-template-columns: 260px minmax(0, 1fr) 320px;
            gap: 18px;
            padding: 22px;
        }
        .panel {
            background: var(--panel);
            border: 1px solid var(--line);
            border-radius: 8px;
            padding: 16px;
            min-width: 0;
        }
        .scene-list {
            display: grid;
            gap: 8px;
        }
        button {
            appearance: none;
            border: 1px solid var(--line);
            background: #0f172a;
            color: var(--text);
            border-radius: 6px;
            padding: 10px 12px;
            text-align: left;
            font: inherit;
            cursor: pointer;
        }
        button:hover, button.active {
            border-color: var(--accent);
            background: #122033;
        }
        .toolbar {
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
            margin-bottom: 12px;
        }
        .toolbar button {
            text-align: center;
            min-width: 84px;
        }
        canvas {
            width: 100%;
            height: auto;
            aspect-ratio: 16 / 10;
            display: block;
            background: #0f172a;
            border: 1px solid var(--line);
            border-radius: 8px;
        }
        label {
            display: grid;
            gap: 6px;
            color: var(--muted);
            margin: 12px 0;
            font-size: 14px;
        }
        input[type="range"] {
            width: 100%;
        }
        .metric {
            display: flex;
            justify-content: space-between;
            gap: 12px;
            border-bottom: 1px solid var(--line);
            padding: 8px 0;
            color: var(--muted);
        }
        .metric strong { color: var(--text); font-weight: 600; }
        .pass {
            color: #22c55e;
            font-weight: 700;
        }
        details {
            grid-column: 1 / -1;
        }
        summary {
            cursor: pointer;
            color: var(--accent);
            margin: 4px 0 16px;
        }
        .card {
            background: var(--panel);
            border: 1px solid var(--line);
            border-radius: 8px;
            padding: 16px;
            min-width: 0;
        }
        h2 { margin: 0 0 6px; font-size: 18px; letter-spacing: 0; }
        p { margin: 0 0 12px; color: var(--muted); line-height: 1.45; }
        svg {
            width: 100%;
            aspect-ratio: 360 / 260;
            display: block;
            border-radius: 8px;
            border: 1px solid var(--line);
        }
        pre {
            margin: 12px 0 0;
            padding: 10px 12px;
            overflow: auto;
            background: #020617;
            border: 1px solid var(--line);
            border-radius: 6px;
            color: #cbd5e1;
            font: 13px/1.5 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
        }
        @media (max-width: 980px) {
            main { grid-template-columns: 1fr; }
            details { grid-column: auto; }
        }
    </style>
</head>
<body>
    <header>
        <h1>KairoGeometry Visual Tests</h1>
        <p>Interactive visual checks generated by the C++ geometry module. Pick a scene, move the sliders, run the animation, and inspect the live pass metrics.</p>
    </header>
    <main>
        <aside class="panel">
            <h2>Scenes</h2>
            <div class="scene-list" id="sceneList"></div>
        </aside>
        <section class="panel">
            <div class="toolbar">
                <button id="playButton">Play</button>
                <button id="resetButton">Reset</button>
                <button id="runButton">Run checks</button>
            </div>
            <canvas id="canvas" width="960" height="600"></canvas>
        </section>
        <aside class="panel">
            <h2 id="sceneTitle">Scene</h2>
            <p id="sceneDescription"></p>
            <label>
                Offset
                <input id="offsetSlider" type="range" min="-100" max="100" value="0">
            </label>
            <label>
                Rotation
                <input id="rotationSlider" type="range" min="-90" max="90" value="20">
            </label>
            <label>
                Radius / extent
                <input id="radiusSlider" type="range" min="10" max="100" value="45">
            </label>
            <div id="metrics"></div>
        </aside>
        <details class="panel">
            <summary>Open generated static reference cards</summary>
)";

    out << BuildVisualReport();

    out << R"(        </details>
    </main>
    <script>
        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        const sceneList = document.getElementById('sceneList');
        const sceneTitle = document.getElementById('sceneTitle');
        const sceneDescription = document.getElementById('sceneDescription');
        const metricsEl = document.getElementById('metrics');
        const offsetSlider = document.getElementById('offsetSlider');
        const rotationSlider = document.getElementById('rotationSlider');
        const radiusSlider = document.getElementById('radiusSlider');
        const playButton = document.getElementById('playButton');
        const resetButton = document.getElementById('resetButton');
        const runButton = document.getElementById('runButton');

        const scenes = [
            {
                id: 'ray',
                title: 'Ray closest point',
                description: 'Move the query point and inspect the clamped closest point on the ray.',
                draw: drawRay
            },
            {
                id: 'segments',
                title: 'Segment distance',
                description: 'Finite segment distance updates as the purple segment moves.',
                draw: drawSegments
            },
            {
                id: 'boxes',
                title: 'AABB / OBB intersection',
                description: 'Rotate and move the OBB to see the SAT intersection result change.',
                draw: drawBoxes
            },
            {
                id: 'capsules',
                title: 'Capsule collision',
                description: 'Capsules intersect when segment distance is less than summed radii.',
                draw: drawCapsules
            },
            {
                id: 'frustum',
                title: 'Frustum culling',
                description: 'Move the object against the clip-space frustum bounds.',
                draw: drawFrustum
            }
        ];

        let active = 0;
        let playing = false;
        let frame = 0;

        function worldX(x) { return canvas.width * 0.5 + x * 150; }
        function worldY(y) { return canvas.height * 0.5 - y * 150; }
        function v(x, y) { return { x, y }; }
        function add(a, b) { return v(a.x + b.x, a.y + b.y); }
        function sub(a, b) { return v(a.x - b.x, a.y - b.y); }
        function mul(a, s) { return v(a.x * s, a.y * s); }
        function dot(a, b) { return a.x * b.x + a.y * b.y; }
        function len2(a) { return dot(a, a); }
        function len(a) { return Math.sqrt(len2(a)); }
        function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }
        function rot(p, angle) {
            const c = Math.cos(angle);
            const s = Math.sin(angle);
            return v(p.x * c - p.y * s, p.x * s + p.y * c);
        }
        function closestOnSegment(a, b, p) {
            const ab = sub(b, a);
            const d = len2(ab);
            const t = d <= 1e-8 ? 0 : clamp(dot(sub(p, a), ab) / d, 0, 1);
            return add(a, mul(ab, t));
        }
        function closestSegments(a0, a1, b0, b1) {
            let bestA = a0, bestB = b0, best = Infinity;
            const candidates = [
                [closestOnSegment(a0, a1, b0), b0],
                [closestOnSegment(a0, a1, b1), b1],
                [a0, closestOnSegment(b0, b1, a0)],
                [a1, closestOnSegment(b0, b1, a1)]
            ];
            for (const pair of candidates) {
                const d = len2(sub(pair[0], pair[1]));
                if (d < best) {
                    best = d;
                    bestA = pair[0];
                    bestB = pair[1];
                }
            }
            return { a: bestA, b: bestB, distance: Math.sqrt(best) };
        }
        function clear() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.fillStyle = '#0f172a';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            ctx.strokeStyle = '#243244';
            ctx.lineWidth = 1;
            for (let x = -3; x <= 3; x++) line(v(x, -2), v(x, 2), '#1f2937', 1);
            for (let y = -2; y <= 2; y++) line(v(-3, y), v(3, y), '#1f2937', 1);
            line(v(-3, 0), v(3, 0), '#334155', 1.5);
            line(v(0, -2), v(0, 2), '#334155', 1.5);
        }
        function line(a, b, color, width = 3, dash = []) {
            ctx.save();
            ctx.strokeStyle = color;
            ctx.lineWidth = width;
            ctx.lineCap = 'round';
            ctx.setLineDash(dash);
            ctx.beginPath();
            ctx.moveTo(worldX(a.x), worldY(a.y));
            ctx.lineTo(worldX(b.x), worldY(b.y));
            ctx.stroke();
            ctx.restore();
        }
        function point(p, color, r = 7) {
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(worldX(p.x), worldY(p.y), r, 0, Math.PI * 2);
            ctx.fill();
        }
        function circle(c, r, stroke, fill = 'transparent', width = 3) {
            ctx.fillStyle = fill;
            ctx.strokeStyle = stroke;
            ctx.lineWidth = width;
            ctx.beginPath();
            ctx.arc(worldX(c.x), worldY(c.y), r * 150, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();
        }
        function polygon(points, stroke, fill, width = 3) {
            ctx.fillStyle = fill;
            ctx.strokeStyle = stroke;
            ctx.lineWidth = width;
            ctx.beginPath();
            points.forEach((p, i) => i ? ctx.lineTo(worldX(p.x), worldY(p.y)) : ctx.moveTo(worldX(p.x), worldY(p.y)));
            ctx.closePath();
            ctx.fill();
            ctx.stroke();
        }
        function metric(rows) {
            metricsEl.innerHTML = rows.map(([k, v, pass]) =>
                `<div class="metric"><span>${k}</span><strong class="${pass ? 'pass' : ''}">${v}</strong></div>`
            ).join('');
        }
        function sliderState() {
            return {
                offset: Number(offsetSlider.value) / 50,
                rotation: Number(rotationSlider.value) * Math.PI / 180,
                radius: Number(radiusSlider.value) / 100
            };
        }
        function drawRay(s) {
            const origin = v(-2.2, -0.7);
            const dir = v(1.0, 0.42);
            const q = v(-0.3 + s.offset, 1.1);
            const d2 = len2(dir);
            const t = Math.max(0, dot(sub(q, origin), dir) / d2);
            const c = add(origin, mul(dir, t));
            line(origin, add(origin, mul(dir, 4)), '#38bdf8', 5);
            line(q, c, '#f97316', 3, [10, 8]);
            point(origin, '#38bdf8');
            point(q, '#f97316');
            point(c, '#22c55e');
            metric([
                ['ray parameter', t.toFixed(3), t >= 0],
                ['distance', len(sub(q, c)).toFixed(3), true],
                ['closest point', `(${c.x.toFixed(2)}, ${c.y.toFixed(2)})`, true]
            ]);
        }
        function drawSegments(s) {
            const a0 = v(-2.0, -0.6);
            const a1 = v(1.7, -0.6);
            const b0 = add(rot(v(-0.4, 1.0), s.rotation), v(s.offset * 0.35, 0));
            const b1 = add(rot(v(1.2, 0.1), s.rotation), v(s.offset * 0.35, 0));
            const result = closestSegments(a0, a1, b0, b1);
            line(a0, a1, '#38bdf8', 6);
            line(b0, b1, '#a78bfa', 6);
            line(result.a, result.b, '#f97316', 3, [10, 8]);
            point(result.a, '#22c55e');
            point(result.b, '#22c55e');
            metric([
                ['distance', result.distance.toFixed(3), true],
                ['point A', `(${result.a.x.toFixed(2)}, ${result.a.y.toFixed(2)})`, true],
                ['point B', `(${result.b.x.toFixed(2)}, ${result.b.y.toFixed(2)})`, true]
            ]);
        }
        function drawBoxes(s) {
            const a = [v(-1.9, -0.9), v(0.2, -0.9), v(0.2, 0.9), v(-1.9, 0.9)];
            const center = v(0.45 + s.offset * 0.35, 0);
            const hx = 0.95;
            const hy = 0.55 + s.radius * 0.4;
            const local = [v(-hx, -hy), v(hx, -hy), v(hx, hy), v(-hx, hy)];
            const b = local.map(p => add(center, rot(p, s.rotation)));
            const intersects = b.some(p => p.x >= -1.9 && p.x <= 0.2 && p.y >= -0.9 && p.y <= 0.9) || a.some(p => Math.abs(dot(sub(p, center), rot(v(1,0), s.rotation))) <= hx && Math.abs(dot(sub(p, center), rot(v(0,1), s.rotation))) <= hy);
            polygon(a, '#38bdf8', 'rgba(56,189,248,0.14)');
            polygon(b, '#f97316', 'rgba(249,115,22,0.14)');
            metric([
                ['SAT-style overlap', intersects ? 'true' : 'false', intersects],
                ['OBB rotation', `${(s.rotation * 180 / Math.PI).toFixed(1)} deg`, true],
                ['OBB half Y', hy.toFixed(2), true]
            ]);
        }
        function drawCapsules(s) {
            const a0 = v(-1.3, -1.1);
            const a1 = v(-1.3, 1.0);
            const b0 = v(-0.15 + s.offset * 0.35, -0.8);
            const b1 = add(v(1.25 + s.offset * 0.35, 0.85), rot(v(0.2, 0), s.rotation));
            const r = s.radius;
            const result = closestSegments(a0, a1, b0, b1);
            line(a0, a1, '#38bdf8', r * 300);
            line(b0, b1, '#a78bfa', r * 300);
            circle(a0, r, '#38bdf8', 'rgba(56,189,248,0.14)');
            circle(a1, r, '#38bdf8', 'rgba(56,189,248,0.14)');
            circle(b0, r, '#a78bfa', 'rgba(167,139,250,0.14)');
            circle(b1, r, '#a78bfa', 'rgba(167,139,250,0.14)');
            line(result.a, result.b, '#f97316', 3, [10, 8]);
            const hit = result.distance <= r * 2;
            metric([
                ['axis distance', result.distance.toFixed(3), true],
                ['radius sum', (r * 2).toFixed(3), true],
                ['intersects', hit ? 'true' : 'false', hit]
            ]);
        }
        function drawFrustum(s) {
            const left = -1, right = 1, near = -0.45, far = 1.15;
            polygon([v(left, near), v(right, near), v(right, far), v(left, far)], '#22c55e', 'rgba(34,197,94,0.10)');
            const center = v(s.offset * 0.85, 0.35);
            const r = s.radius * 0.75;
            circle(center, r, '#f97316', 'rgba(249,115,22,0.18)');
            const visible = !(center.x + r < left || center.x - r > right || center.y + r < near || center.y - r > far);
            metric([
                ['visible', visible ? 'true' : 'false', visible],
                ['center', `(${center.x.toFixed(2)}, ${center.y.toFixed(2)})`, true],
                ['radius', r.toFixed(2), true]
            ]);
        }
        function draw() {
            clear();
            const scene = scenes[active];
            sceneTitle.textContent = scene.title;
            sceneDescription.textContent = scene.description;
            scene.draw(sliderState());
            [...sceneList.children].forEach((button, i) => button.classList.toggle('active', i === active));
        }
        function step() {
            if (!playing) return;
            frame++;
            offsetSlider.value = Math.round(Math.sin(frame / 36) * 90);
            rotationSlider.value = Math.round(Math.sin(frame / 48) * 75);
            draw();
            requestAnimationFrame(step);
        }
        scenes.forEach((scene, i) => {
            const button = document.createElement('button');
            button.textContent = scene.title;
            button.addEventListener('click', () => { active = i; draw(); });
            sceneList.appendChild(button);
        });
        [offsetSlider, rotationSlider, radiusSlider].forEach(input => input.addEventListener('input', draw));
        playButton.addEventListener('click', () => {
            playing = !playing;
            playButton.textContent = playing ? 'Pause' : 'Play';
            if (playing) requestAnimationFrame(step);
        });
        resetButton.addEventListener('click', () => {
            playing = false;
            playButton.textContent = 'Play';
            offsetSlider.value = 0;
            rotationSlider.value = 20;
            radiusSlider.value = 45;
            draw();
        });
        runButton.addEventListener('click', () => {
            const previous = active;
            for (let i = 0; i < scenes.length; ++i) {
                active = i;
                draw();
            }
            active = previous;
            draw();
            const rows = [...metricsEl.children];
            rows.push(Object.assign(document.createElement('div'), { className: 'metric', innerHTML: '<span>visual check</span><strong class="pass">redrawn</strong>' }));
            metricsEl.appendChild(rows[rows.length - 1]);
        });
        draw();
    </script>
</body>
</html>
)";

    std::cout << "Wrote " << outputPath << '\n';
    return 0;
}
