import numpy as np
import chess
from src.ai_engine.mcts import MCTS
from src.utils.utils import board_to_tensor, create_policy_vector


class SelfPlay:

    def __init__(self, model, n_sims=100, max_moves=120):
        self.model = model
        self.mcts = MCTS(model, n_sims)
        self.max_moves = max_moves

    def play_game(self, temperature=1.0, temp_threshold=10):
        board = chess.Board()
        history = []

        for move_num in range(self.max_moves):
            state = board_to_tensor(board)[0]

            # explore early, exploit later
            temp = temperature if move_num < temp_threshold else 0.1
            moves, probs = self.mcts.get_move_probs(board, temp)
            if not moves:
                break

            policy = create_policy_vector(moves, probs)
            history.append((state, policy))

            if temp > 0.5:
                idx = np.random.choice(len(moves), p=probs)
            else:
                idx = np.argmax(probs)

            board.push(moves[idx])
            if board.is_game_over():
                break

        result = board.result()
        return self._label_outcomes(history, result) + (result,)

    def _label_outcomes(self, history, result):
        if result == "1-0":
            outcome = 1.0
        elif result == "0-1":
            outcome = -1.0
        else:
            outcome = 0.0

        states, policies, values = [], [], []

        for state, policy in history:
            was_white = state[0, 0, 12] == 1.0
            states.append(state)
            policies.append(policy)
            values.append(outcome if was_white else -outcome)

        return np.array(states), np.array(policies), np.array(values)

    def generate_games(self, n_games, temperature=1.0):
        game_states, game_policies, game_values = [], [], []
        results = []

        for i in range(n_games):
            states, policies, values, result = self.play_game(temperature)
            game_states.append(states)
            game_policies.append(policies)
            game_values.append(values)
            results.append(result)

        X = np.concatenate(game_states) if game_states else np.array([])
        y_pol = np.concatenate(game_policies) if game_policies else np.array([])
        y_val = np.concatenate(game_values) if game_values else np.array([])
        return X, y_pol, y_val, results


class Arena:

    def __init__(self, model1, model2, n_sims=100):
        self.mcts1 = MCTS(model1, n_sims)
        self.mcts2 = MCTS(model2, n_sims)

    def play_game(self, model1_white=True):
        board = chess.Board()

        for _ in range(120):
            if board.turn == chess.WHITE:
                mcts = self.mcts1 if model1_white else self.mcts2
            else:
                mcts = self.mcts2 if model1_white else self.mcts1

            moves, probs = mcts.get_move_probs(board, temperature=0.1)
            if not moves:
                break

            board.push(moves[np.argmax(probs)])
            if board.is_game_over():
                break

        result = board.result()
        if result == "1-0":
            return 1.0 if model1_white else 0.0
        elif result == "0-1":
            return 0.0 if model1_white else 1.0
        return 0.5

    def evaluate(self, n_games=10):
        wins = 0
        for i in range(n_games):
            wins += self.play_game(i % 2 == 0)
        return wins / n_games
