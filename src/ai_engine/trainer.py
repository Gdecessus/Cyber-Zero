import os
import logging
from src.ai_engine.model import ChessModel
from src.ai_engine.selfplay import SelfPlay, Arena


class ChessTrainer:
    def __init__(self, model=None):
        self.model = model if model else ChessModel()

    def reinforcement_train(self, episodes=10, games_per_ep=8, n_sims=50):
        print(f"\n--- Starting reinforcement learning ({episodes} episodes) ---")
        logging.info("Reinforcement learning start")

        selfplay = SelfPlay(self.model, n_sims=n_sims)

        for ep in range(episodes):
            print(f"\nEpisode {ep+1}/{episodes}")
            logging.info(f"Episode {ep+1}/{episodes}")

            states, policies, values, results = selfplay.generate_games(games_per_ep)

            if len(states) == 0:
                print("  no data generated, skipping...")
                continue

            w = results.count('1-0')
            b = results.count('0-1')
            d = results.count('1/2-1/2')
            print(f"  {len(states)} positions | results: {w}W-{b}B-{d}D")
            logging.info(f"Generated {len(states)} positions, results: {w}W-{b}B-{d}D")

            history = self.model.train(
                states,
                {'policy': policies, 'value': values},
                epochs=1, batch_size=32, verbose=1
            )

            loss = history.history['loss'][0]
            print(f"  loss: {loss:.4f}")
            logging.info(f"Loss: {loss:.4f}")

            self.model.save("current_model.keras")

        self.model.save("reinforced_model.keras")
        print("Reinforcement learning done.")
        logging.info("Reinforcement learning complete")

    def arena_evaluate(self, current_path, best_path, n_games=10):
        """Play current model vs best model to see if we improved."""
        print(f"\n--- Arena evaluation ({n_games} games) ---")
        logging.info("Arena evaluation start")

        if not os.path.exists(best_path):
            print("No best model yet, saving current as best.")
            self.model.save(best_path)
            return True

        best = ChessModel.load(best_path)
        arena = Arena(self.model, best, n_sims=100)
        win_rate = arena.evaluate(n_games)

        print(f"Win rate vs best: {win_rate*100:.1f}%")
        logging.info(f"Current model win rate: {win_rate*100:.1f}%")

        if win_rate > 0.55:
            print("New best model!")
            logging.info("New champion! Saving as best model.")
            self.model.save(best_path)
            return True
        else:
            print("Best model unchanged.")
            logging.info("Best model remains unchanged.")
            return False

    def full_training_pipeline(self):
        """AlphaZero-style loop: self-play -> train -> evaluate -> repeat."""
        print("=== Full training pipeline ===")
        logging.info("Full AlphaZero-style training pipeline start")

        # initial round with more episodes
        self.reinforcement_train(episodes=5, games_per_ep=8, n_sims=50)
        self.arena_evaluate("current_model.keras", "best_model.keras", n_games=10)

        # iterate a few more rounds
        for i in range(3):
            print(f"\n=== Iteration {i+1}/3 ===")
            logging.info(f"Training iteration {i+1}/3")

            self.reinforcement_train(episodes=3, games_per_ep=6, n_sims=50)
            improved = self.arena_evaluate("current_model.keras", "best_model.keras")

            if improved:
                print(f"  iteration {i+1}: model improved!")

            # always reload best model for next round
            self.model = ChessModel.load("best_model.keras")

        print("\nTraining complete. Best model: best_model.keras")
        logging.info("Training complete. Final model saved as: best_model.keras")
