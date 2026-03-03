import os
import random
import numpy as np
import chess
from src.ai_engine.model import ChessModel
from src.ai_engine.mcts import MCTS
from src.utils.utils import board_to_tensor, create_policy_vector


def play_game(mcts, max_moves=120, temp_threshold=10):
    # play one full game of self-play, return training data
    board = chess.Board()
    history = []

    for move_num in range(max_moves):
        state = board_to_tensor(board)[0]

        # explore early, exploit later
        if move_num < temp_threshold:
            temp = 1.0
        else:
            temp = 0.1

        moves, probs = mcts.get_move_probs(board, temp)
        if not moves:
            break

        history.append((state, create_policy_vector(moves, probs)))

        # pick a move — random early on, best move later
        if temp > 0.5:
            idx = random.choices(range(len(moves)), weights=probs)[0]
        else:
            idx = 0
            for i in range(len(probs)):
                if probs[i] > probs[idx]:
                    idx = i

        board.push(moves[idx])
        if board.is_game_over():
            break

    return _label_outcomes(history, board.result())


def _label_outcomes(history, result):
    # turn result string into a number
    if result == '1-0':
        outcome = 1.0
    elif result == '0-1':
        outcome = -1.0
    else:
        outcome = 0.0

    states = []
    policies = []
    values = []

    for state, policy in history:
        states.append(state)
        policies.append(policy)

        # channel 12 = whose turn (1.0 = white)
        was_white = state[0, 0, 12] == 1.0
        if was_white:
            values.append(outcome)
        else:
            values.append(-outcome)

    return np.array(states), np.array(policies), np.array(values), result


def generate_data(model, n_games, n_sims=50):
    # play n_games of self-play, combine all training data
    mcts = MCTS(model, n_sims)

    all_states = []
    all_policies = []
    all_values = []
    all_results = []

    for _ in range(n_games):
        states, policies, values, result = play_game(mcts)
        all_states.append(states)
        all_policies.append(policies)
        all_values.append(values)
        all_results.append(result)

    if len(all_states) == 0 or len(all_states[0]) == 0:
        return np.array([]), np.array([]), np.array([]), all_results

    return np.concatenate(all_states), np.concatenate(all_policies), np.concatenate(all_values), all_results


def evaluate_models(current_model, best_model, n_games=10, n_sims=100):
    # play n_games between two models, return win rate for current_model
    current_mcts = MCTS(current_model, n_sims)
    best_mcts = MCTS(best_model, n_sims)

    score = 0.0
    for i in range(n_games):
        # alternate who plays white
        current_is_white = (i % 2 == 0)
        board = chess.Board()

        for _ in range(120):
            # pick which mcts to use for this turn
            if board.turn == chess.WHITE:
                if current_is_white:
                    mcts = current_mcts
                else:
                    mcts = best_mcts
            else:
                if current_is_white:
                    mcts = best_mcts
                else:
                    mcts = current_mcts

            moves, probs = mcts.get_move_probs(board, temperature=0.1)
            if not moves:
                break

            # pick the best move
            best_idx = 0
            for j in range(len(probs)):
                if probs[j] > probs[best_idx]:
                    best_idx = j
            board.push(moves[best_idx])

            if board.is_game_over():
                break

        # 1 point for win, 0.5 for draw, 0 for loss
        result = board.result()
        if result == "1-0":
            if current_is_white:
                score += 1.0
        elif result == "0-1":
            if not current_is_white:
                score += 1.0
        else:
            score += 0.5

    return score / n_games


def train(model, episodes=5, games_per_ep=8, n_sims=50):
    for ep in range(episodes):
        print(f"\nEpisode {ep+1}/{episodes}")

        states, policies, values, results = generate_data(model, games_per_ep, n_sims)
        if len(states) == 0:
            continue

        w = results.count('1-0')
        b = results.count('0-1')
        d = results.count('1/2-1/2')
        print(f"  {len(states)} positions | {w}W-{b}B-{d}D")

        history = model.train(states, {'policy': policies, 'value': values})
        print(f"  loss: {history.history['loss'][0]:.4f}")

        model.save("current_model.keras")


if __name__ == "__main__":
    print("\n=== Cyber-Zero Training ===")

    if os.path.exists("current_model.keras"):
        print("Loading checkpoint...")
        model = ChessModel.load("current_model.keras")
    else:
        model = ChessModel()

    try:
        train(model, episodes=5, games_per_ep=8, n_sims=50)

        print("\nEvaluating against best model...")
        if os.path.exists("best_model.keras"):
            best = ChessModel.load("best_model.keras")
            win_rate = evaluate_models(model, best)
            print(f"  win rate: {win_rate*100:.1f}%")

            if win_rate > 0.55:
                print("  New best model!")
                model.save("best_model.keras")
            else:
                print("  Keeping previous best.")
        else:
            print("No best model yet, saving current as best.")
            model.save("best_model.keras")

        print("\nDone.")

    except KeyboardInterrupt:
        print("\nInterrupted, saving...")
        model.save("interrupted_model.keras")
