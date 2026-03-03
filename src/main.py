import os
from src.ai_engine.model import ChessModel
from src.ai_engine.trainer import ChessTrainer


def main():
    print("\n=== Cyber-Zero Training ===")
    print("1) Full training pipeline")
    print("2) Reinforcement only")
    print("3) Continue from checkpoint")

    choice = input("\nSelect: ").strip()

    if choice == '3' and os.path.exists("current_model.keras"):
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

        print("\nDone.")

    except KeyboardInterrupt:
        print("\nInterrupted, saving...")
        trainer.model.save("interrupted_model.keras")
        print("Saved. Resume with option 3.")


if __name__ == "__main__":
    main()
