import os
import numpy as np
import chess
from src.ai_engine.model import ChessModel
from src.ai_engine.mcts import MCTS
from src.utils.utils import board_to_tensor, create_policy_vector


def play_game(mcts, max_moves=120, temp_threshold=10):
    board = chess.Board()
    history = []

    for move_num in range(max_moves):
        state = board_to_tensor(board)[0]

        # explore early moves, exploit later ones
        if move_num < temp_threshold:
            temp = 1.0
        else:
            temp = 0.1
        moves, probs = mcts.get_move_probs(board, temp)
        if not moves:
            break

        history.append((state, create_policy_vector(moves, probs)))

        if temp > 0.5:
            idx = np.random.choice(len(moves), p=probs)
        else:
            idx = np.argmax(probs)

        board.push(moves[idx])
        if board.is_game_over():
            break

    return _label_outcomes(history, board.result())


def _label_outcomes(history, result):
    # convert the game result string to a number
    if result == '1-0':
        outcome = 1.0      # white won
    elif result == '0-1':
        outcome = -1.0     # black won
    else:
        outcome = 0.0      # draw

    states = []
    policies = []
    values = []
    for state, policy in history:
        # channel 12 tells us whose turn it was (1.0 = white)
        was_white = state[0, 0, 12] == 1.0

        states.append(state)
        policies.append(policy)

        # white gets +outcome, black gets -outcome
        if was_white:
            values.append(outcome)
        else:
            values.append(-outcome)

    return np.array(states), np.array(policies), np.array(values), result


def generate_data(model, n_games, n_sims=50):
    """Play n_games of self-play and combine all the training data."""
    mcts = MCTS(model, n_sims)

    # play each game and collect the results
    game_data = []
    for _ in range(n_games):
        game_data.append(play_game(mcts))

    # separate into lists by type
    states = []
    policies = []
    values = []
    results = []
    for g in game_data:
        states.append(g[0])
        policies.append(g[1])
        values.append(g[2])
        results.append(g[3])

    # if no data was generated, return empty arrays
    if len(states) == 0 or len(states[0]) == 0:
        return np.array([]), np.array([]), np.array([]), results

    # stack all the games into single arrays
    return np.concatenate(states), np.concatenate(policies), np.concatenate(values), results


def evaluate_models(current, best, n_games=10, n_sims=100):
    mcts1 = MCTS(current, n_sims)
    mcts2 = MCTS(best, n_sims)

    score = 0.0
    for i in range(n_games):
        model1_white = (i % 2 == 0)
        board = chess.Board()

        for _ in range(120):
            # pick which MCTS to use based on whose turn it is
            if board.turn == chess.WHITE:
                if model1_white:
                    mcts = mcts1
                else:
                    mcts = mcts2
            else:
                if model1_white:
                    mcts = mcts2
                else:
                    mcts = mcts1

            moves, probs = mcts.get_move_probs(board, temperature=0.1)
            if not moves:
                break

            board.push(moves[np.argmax(probs)])
            if board.is_game_over():
                break

        # score: 1 point for a win, 0.5 for draw, 0 for loss
        result = board.result()
        if result == "1-0":
            if model1_white:
                score += 1.0
            else:
                score += 0.0
        elif result == "0-1":
            if model1_white:
                score += 0.0
            else:
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
