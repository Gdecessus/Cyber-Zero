import os
import numpy as np
import chess
from src.ai_engine.model import ChessModel
from src.ai_engine.mcts import MCTS


class ChessGame:
    """Interface between the HTTP server and the AI engine."""

    def __init__(self, model_path="best_model.keras", n_sims=200):
        if model_path and os.path.exists(model_path):
            self.model = ChessModel.load(model_path)
            print(f"Loaded model from {model_path}")
        else:
            self.model = ChessModel()
            print("No saved model found, using fresh network")
        self.mcts = MCTS(self.model, n_sims)
        self.board = chess.Board()

    def reset(self):
        self.board.reset()

    def get_state(self):
        game_over = self.board.is_game_over()
        result = self.board.result() if game_over else None

        # figure out why the game ended
        end_reason = None
        if self.board.is_checkmate():
            end_reason = "checkmate"
        elif self.board.is_stalemate():
            end_reason = "stalemate"
        elif self.board.is_insufficient_material():
            end_reason = "insufficient_material"
        elif game_over:
            end_reason = "draw"  # covers 75-move, fivefold rep, etc

        return {
            "fen": self.board.fen(),
            "turn": "w" if self.board.turn == chess.WHITE else "b",
            "legal_moves": [m.uci() for m in self.board.legal_moves],
            "game_over": game_over,
            "result": result,
            "end_reason": end_reason,
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

    def play_ai_move(self, temperature=0.1):
        moves, probs = self.mcts.get_move_probs(self.board, temperature=temperature)
        if not moves:
            return None

        best = np.argmax(probs)
        move = moves[best]
        self.board.push(move)

        return {"move": move.uci()}
