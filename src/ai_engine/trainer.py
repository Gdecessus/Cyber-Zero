import os
from src.ai_engine.model import ChessModel
from src.ai_engine.selfplay import SelfPlay, Arena


class ChessTrainer:

    def __init__(self, model=None):
        self.model = model if model else ChessModel()

    def reinforcement_train(self, episodes=10, games_per_ep=8, n_sims=50):
        selfplay = SelfPlay(self.model, n_sims=n_sims)

        for ep in range(episodes):
            print(f"\nEpisode {ep+1}/{episodes}")

            states, policies, values, results = selfplay.generate_games(games_per_ep)
            if len(states) == 0:
                continue

            w = results.count('1-0')
            b = results.count('0-1')
            d = results.count('1/2-1/2')
            print(f"  {len(states)} positions | {w}W-{b}B-{d}D")

            history = self.model.train(
                states,
                {'policy': policies, 'value': values},
                epochs=1, batch_size=32
            )
            print(f"  loss: {history.history['loss'][0]:.4f}")

            self.model.save("current_model.keras")

        self.model.save("reinforced_model.keras")

    def arena_evaluate(self, current_path, best_path, n_games=10):
        if not os.path.exists(best_path):
            print("No best model yet, saving current as best.")
            self.model.save(best_path)
            return True

        best = ChessModel.load(best_path)
        arena = Arena(self.model, best, n_sims=100)
        win_rate = arena.evaluate(n_games)

        print(f"  win rate: {win_rate*100:.1f}%")

        if win_rate > 0.55:
            print("  new best model!")
            self.model.save(best_path)
            return True

        return False

    def full_training_pipeline(self):
        # initial round with more episodes
        self.reinforcement_train(episodes=5, games_per_ep=8, n_sims=50)
        self.arena_evaluate("current_model.keras", "best_model.keras")

        for i in range(3):
            print(f"\n=== Iteration {i+1}/3 ===")

            self.reinforcement_train(episodes=3, games_per_ep=6, n_sims=50)
            self.arena_evaluate("current_model.keras", "best_model.keras")

            # reload best model for next round
            self.model = ChessModel.load("best_model.keras")
