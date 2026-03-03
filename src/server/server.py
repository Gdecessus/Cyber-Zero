import os
import numpy as np
import chess
from flask import Flask, jsonify, request, send_from_directory
from src.ai_engine.model import ChessModel
from src.ai_engine.mcts import MCTS


class ChessGame:
    def __init__(self, model_path="best_model.keras", n_sims=200):
        if model_path and os.path.exists(model_path):
            self.model = ChessModel.load(model_path)
        else:
            self.model = ChessModel()

        self.mcts = MCTS(self.model, n_sims)
        self.board = chess.Board()
    
    def reset(self):
        self.board.reset()

    def get_state(self):
        """Return everything the frontend needs to draw the board."""
        game_over = self.board.is_game_over()

        # convert move objects to simple strings like "e2e4"
        moves = []
        for m in self.board.legal_moves:
            moves.append(m.uci())

        # only look up the result if the game is actually over
        result = None
        if game_over:
            result = self.board.result()

        return {
            "fen": self.board.fen(),
            # board.turn is True for white, False for black
            "turn": "w" if self.board.turn == chess.WHITE else "b",
            "legal_moves": moves,
            "game_over": game_over,
            "result": result,
        }

    def apply_move(self, move_uci):
        try:
            move = chess.Move.from_uci(move_uci)
        except ValueError:
            return {"ok": False, "error": "invalid_format"}

        if move not in self.board.legal_moves:
            return {"ok": False, "error": "illegal_move"}

        self.board.push(move)
        return {"ok": True, "move": move.uci()}

    def play_ai_move(self):
        moves, probs = self.mcts.get_move_probs(self.board, temperature=0.1)
        if not moves:
            return None
        move = moves[np.argmax(probs)]
        self.board.push(move)
        return move.uci()


# FLASK SERVER BELOW

UI_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "UI_chess")
app = Flask(__name__, static_folder=UI_DIR)
game = ChessGame()


@app.route("/")
def index():
    return send_from_directory(UI_DIR, "index.html")


@app.route("/<path:filename>")
def static_files(filename):
    return send_from_directory(UI_DIR, filename)


@app.route("/state")
def state():
    return jsonify(game.get_state())


@app.route("/move")
def move():
    uci = request.args.get("uci", "")
    return jsonify(game.apply_move(uci))


@app.route("/ai")
def ai():
    return jsonify({"move": game.play_ai_move()})


@app.route("/reset")
def reset():
    game.reset()
    return jsonify({"ok": True})

if __name__ == "__main__":
    print("Server ON")
    app.run(host="0.0.0.0", port=8000)
