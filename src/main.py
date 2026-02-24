import os
import logging
from datetime import datetime
from src.ai_engine.model import ChessModel
from src.ai_engine.trainer import ChessTrainer


def setup_logging():
    log_file = f"training_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()  # also print to console
        ]
    )


def main():
    setup_logging()

    print("\n=== Cyber-Zero Training ===")
    print("1) Full training pipeline")
    print("2) Reinforcement training only")
    print("3) Continue from checkpoint")
    choice = input("\nSelect option: ").strip()

    if choice == '3' and os.path.exists("current_model.keras"):
        print("Loading checkpoint...")
        model = ChessModel.load("current_model.keras")
        trainer = ChessTrainer(model)
    else:
        trainer = ChessTrainer()

    try:
        if choice == '1':
            trainer.full_training_pipeline()
        elif choice == '2':
            trainer.reinforcement_train(episodes=10, games_per_ep=8)
            trainer.model.save("best_model.keras")
        elif choice == '3':
            trainer.reinforcement_train(episodes=5, games_per_ep=6)
            trainer.arena_evaluate("current_model.keras", "best_model.keras")
        else:
            print("Invalid option.")
            return

        print("\nAll done!")

    except KeyboardInterrupt:
        print("\nInterrupted, saving model...")
        trainer.model.save("interrupted_model.keras")
        print("Saved to interrupted_model.keras. Resume with option 3.")


if __name__ == "__main__":
    main()
