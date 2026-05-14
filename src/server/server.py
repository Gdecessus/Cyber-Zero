import os
import numpy as np
import chess
import requests
from flask import Flask, jsonify, request, send_from_directory
from src.ai_engine.model import ChessModel
from src.ai_engine.mcts import MCTS


ARM_URL = "http://localhost:5000"
ARM_TIMEOUT = 60


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
        # return everything the frontend needs to draw the board
        game_over = self.board.is_game_over()

        # convert move objects to simple strings like "e2e4"
        moves = []
        for m in self.board.legal_moves:
            moves.append(m.uci())

        # only look up the result if the game is actually over
        result = None
        if game_over:
            result = self.board.result()

        # board.turn is True for white, False for black
        if self.board.turn == chess.WHITE:
            turn = "w"
        else:
            turn = "b"

        return {
            "fen": self.board.fen(),
            "turn": turn,
            "legal_moves": moves,
            "game_over": game_over,
            "result": result,
        }

    # validates a player move, updates the board, then tells the arm to execute it
    def apply_move(self, move_uci):
        try:
            move = chess.Move.from_uci(move_uci)
        except ValueError:
            return {"ok": False, "error": "invalid_format"}

        # only legal moves get through — this is our defence against bad UI input
        if move not in self.board.legal_moves:
            return {"ok": False, "error": "illegal_move"}

        # if this is a capture, figure out which graveyard the dead piece goes to
        grave = None
        if self.board.is_capture(move):
            # captured piece is whoever isn't moving
            grave = "GRAVEB" if self.board.turn == chess.WHITE else "GRAVEW"
        self.board.push(move)
        self._arm_play(move, grave)
        return {"ok": True, "move": move.uci()}

    # asks MCTS for the best move from the current position, then plays it physically
    def play_ai_move(self):
        moves, probs = self.mcts.get_move_probs(self.board, temperature=0.1)
        if not moves:
            return None
        # pick the move with the highest probability from MCTS
        move = moves[np.argmax(probs)]
        grave = None
        if self.board.is_capture(move):
            grave = "GRAVEB" if self.board.turn == chess.WHITE else "GRAVEW"
        self.board.push(move)
        self._arm_play(move, grave)
        return move.uci()

    # turn a chess move into one or two physical arm actions
    def _arm_play(self, move, grave):
        from_sq = chess.square_name(move.from_square)
        to_sq = chess.square_name(move.to_square)

        # take the captured piece off first, then do the move
        if grave:
            self._arm_send(to_sq, grave)
        self._arm_send(from_sq, to_sq)

    # one HTTP call to the arm server — blocks until the physical motion is done
    def _arm_send(self, from_sq, to_sq):
        try:
            requests.post(
                f"{ARM_URL}/pick_and_place",
                json={"from": from_sq, "to": to_sq},
                timeout=ARM_TIMEOUT,
            )
        except requests.exceptions.ConnectionError:
            pass


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


@app.route("/history")
def history():
    moves = []
    for m in game.board.move_stack:
        moves.append(m.uci())

    if len(moves) == 0:
        return '<div class="empty">No moves yet</div>'

    # group as pairs (white move, black move) per turn
    html = ""
    move_num = 1
    for i in range(0, len(moves), 2):
        white_move = moves[i]
        if i + 1 < len(moves):
            black_move = moves[i + 1]
        else:
            black_move = ""

        html = html + '<div class="hist-row">'
        html = html +   '<span class="hist-num">' + str(move_num) + '.</span>'
        html = html +   '<span class="hist-w">' + white_move + '</span>'
        html = html +   '<span class="hist-b">' + black_move + '</span>'
        html = html + '</div>'
        move_num = move_num + 1

    return html


@app.route("/thinking")
def thinking():
    if game.mcts.root is None:
        return "..."

    # collect (visits, uci) pairs
    pairs = []
    for move in game.mcts.root.children:
        visits = game.mcts.root.children[move].visits
        pairs.append((visits, move.uci()))

    # sort biggest first
    pairs.sort()
    pairs.reverse()

    # build HTML for top 10
    html = ""
    for i in range(10):
        if i >= len(pairs):
            break
        visits = pairs[i][0]
        uci = pairs[i][1]
        bar = "█" * visits

        if i == 0:
            html = html + '<div class="top-move">' + uci + " " + bar + " " + str(visits) + '</div>'
        else:
            html = html + '<div>' + uci + " " + bar + " " + str(visits) + '</div>'

    return html


if __name__ == "__main__":
    print("Server ON")
    app.run(host="0.0.0.0", port=8000, threaded=True)
