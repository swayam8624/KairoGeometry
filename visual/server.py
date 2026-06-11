import http.server
import os
import socketserver

PORT = int(os.environ.get("KAIRO_GEOMETRY_VISUAL_PORT", "8090"))
ROOT = os.path.dirname(__file__)


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    def do_GET(self):
        if self.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return

        super().do_GET()


if __name__ == "__main__":
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"KairoGeometry visual tests: http://localhost:{PORT}/geometry_visual_tests.html")
        httpd.serve_forever()
