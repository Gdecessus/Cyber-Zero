class Config:
    # network
    NUM_RES_BLOCKS = 4
    NUM_FILTERS = 64
    INPUT_SHAPE = (8, 8, 13)  # 12 piece planes + 1 turn plane
    POLICY_SIZE = 4672

    LR = 0.002
    BATCH_SIZE = 32
    GAMES_PER_EPISODE = 16
    EPISODES = 50

    # mcts
    N_SIMULATIONS = 100
    C_PUCT = 1.4  # tried 1.0 and 2.0, 1.4 seems best
    TEMP_THRESHOLD = 10
    MAX_MOVES = 120

    # arena evaluation
    ARENA_GAMES = 10
    WIN_THRESHOLD = 0.55
    EVAL_INTERVAL = 5

    # file paths
    BEST_MODEL = "best_model.keras"
    CURRENT_MODEL = "current_model.keras"
    TRAINING_STATE = "training_state.json"
