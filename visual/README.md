# KairoGeometry Visual Tests

The visual test target generates a static HTML/SVG report for quick inspection
of geometry behavior. The generated page is interactive: scene buttons switch
between geometry cases, sliders redraw the current query, and the play/run
buttons animate or refresh the checks.

## Generate

```sh
cmake --build Foundation/KairoGeometry/cmake-build-geometry-llvm --target KairoGeometryVisual
./Foundation/KairoGeometry/cmake-build-geometry-llvm/KairoGeometryVisual
```

Output:

```text
Foundation/KairoGeometry/visual/geometry_visual_tests.html
```

## View

Open the generated file directly in a browser, or serve this folder:

```sh
python3 Foundation/KairoGeometry/visual/server.py
```

Then open:

```text
http://localhost:8090/geometry_visual_tests.html
```

If an older server is already running, stop it with `Ctrl+C`, regenerate the
HTML, and start the server again so the browser receives the updated page.
