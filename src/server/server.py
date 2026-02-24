import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
from src.ai_engine.play import ChessGame


game = ChessGame()

# UI files live two levels up from this script
UI_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "UI_chess")

MIME_TYPES = {
    ".html": "text/html",
    ".css": "text/css",
    ".js": "application/javascript",
}


class ChessHandler(BaseHTTPRequestHandler):

    def _json_response(self, data, status=200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        # CORS preflight
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/state":
            return self._json_response(game.get_state())

        if path == "/ai":
            return self._json_response({"ok": True, "ai": game.play_ai_move()})

        if path == "/reset":
            game.reset()
            return self._json_response({"ok": True})

        if path == "/move":
            params = parse_qs(urlparse(self.path).query)
            uci = params.get("uci", [""])[0]
            result = game.apply_move(uci)
            return self._json_response(result, 200 if result.get("ok") else 400)

        # serve UI files
        if path == "/":
            return self._serve_file("index.html")

        # try to serve whatever file was requested
        rel = path.lstrip("/")
        if rel:
            return self._serve_file(rel)

        return self._json_response({"error": "not_found"}, 404)

    def log_message(self, fmt, *args):
        pass  # suppress default request logging, it's noisy

    def _serve_file(self, rel_path):
        fpath = os.path.join(UI_DIR, rel_path)
        if not os.path.isfile(fpath):
            return self._json_response({"error": "not_found"}, 404)

        with open(fpath, "rb") as f:
            data = f.read()

        ext = os.path.splitext(fpath)[1]
        content_type = MIME_TYPES.get(ext, "application/octet-stream")

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main():
    port = 8000
    server = HTTPServer(("0.0.0.0", port), ChessHandler)
    print(f"Chess server running on http://localhost:{port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
