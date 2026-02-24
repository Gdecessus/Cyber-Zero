import numpy as np
import chess


def move_to_index(move):
    """
    Encode a chess move as an integer index into the policy vector.
    Normal moves: from_sq * 64 + to_sq  (0..4095)
    Promotions:   4096 + from_sq * 4 + promo_type  (4096..4351)
    """
    from_sq = move.from_square
    to_sq = move.to_square

    if move.promotion:
        promo_map = {chess.KNIGHT: 0, chess.BISHOP: 1, chess.ROOK: 2, chess.QUEEN: 3}
        return 4096 + from_sq * 4 + promo_map[move.promotion]

    return from_sq * 64 + to_sq


def board_to_tensor(board):
    """
    Convert board to 8x8x13 tensor.
    Channels 0-5:  black pieces (p, n, b, r, q, k)
    Channels 6-11: white pieces (P, N, B, R, Q, K)
    Channel 12:    whose turn (1.0 = white, 0.0 = black)
    """
    tensor = np.zeros((8, 8, 13), dtype=np.float32)

    piece_map = {
        'p': 0, 'n': 1, 'b': 2, 'r': 3, 'q': 4, 'k': 5,
        'P': 6, 'N': 7, 'B': 8, 'R': 9, 'Q': 10, 'K': 11
    }

    for sq in chess.SQUARES:
        piece = board.piece_at(sq)
        if piece:
            row = chess.square_rank(sq)
            col = chess.square_file(sq)
            tensor[row, col, piece_map[piece.symbol()]] = 1.0

    # turn channel
    tensor[:, :, 12] = float(board.turn)  # chess.WHITE = True = 1.0

    return np.expand_dims(tensor, axis=0)


def create_policy_vector(moves, probs):
    """Map move probabilities into a full 4672-length policy vector."""
    policy = np.zeros(4672, dtype=np.float32)

    for move, prob in zip(moves, probs):
        policy[move_to_index(move)] = prob

    # renormalize just in case
    total = np.sum(policy)
    if total > 0:
        policy /= total

    return policy
