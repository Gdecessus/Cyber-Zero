import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
from src.ai_engine.play import ChessGame

game = ChessGame()
UI_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "UI_chess")

MIME_TYPES = {".html": "text/html", ".css": "text/css", ".js": "application/javascript"}


class ChessHandler(BaseHTTPRequestHandler):

    def _json(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/state":
            return self._json(game.get_state())
        if path == "/ai":
            return self._json({"ok": True, "ai": game.play_ai_move()})
        if path == "/reset":
            game.reset()
            return self._json({"ok": True})
        if path == "/move":
            uci = parse_qs(urlparse(self.path).query).get("uci", [""])[0]
            res = game.apply_move(uci)
            return self._json(res, 200 if res.get("ok") else 400)

        # serve UI
        if path == "/":
            return self._serve_file("index.html")

        rel = path.lstrip("/")
        if rel:
            return self._serve_file(rel)

        return self._json({"error": "not_found"}, 404)

    def _serve_file(self, rel_path):
        fpath = os.path.join(UI_DIR, rel_path)
        if not os.path.isfile(fpath):
            return self._json({"error": "not_found"}, 404)

        with open(fpath, "rb") as f:
            data = f.read()

        ext = os.path.splitext(fpath)[1]
        ctype = MIME_TYPES.get(ext, "application/octet-stream")

        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        pass


if __name__ == "__main__":
    server = HTTPServer(("0.0.0.0", 8000), ChessHandler)
    print("Server running on http://localhost:8000")
    server.serve_forever()
