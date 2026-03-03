import numpy as np
import chess

# promotion piece -> index offset
PROMO_INDEX = {chess.KNIGHT: 0, chess.BISHOP: 1, chess.ROOK: 2, chess.QUEEN: 3}

# piece symbol -> channel index in the board tensor
PIECE_CHANNEL = {
    'p': 0, 'n': 1, 'b': 2, 'r': 3, 'q': 4, 'k': 5,
    'P': 6, 'N': 7, 'B': 8, 'R': 9, 'Q': 10, 'K': 11
}


def move_to_index(move):
    # encode a chess move as an index into the 4672-length policy vector
    # normal moves: from_sq * 64 + to_sq  (0..4095)
    # promotions:   4096 + from_sq * 4 + promo_type  (4096..4351)
    if move.promotion:
        return 4096 + move.from_square * 4 + PROMO_INDEX[move.promotion]
    return move.from_square * 64 + move.to_square


def board_to_tensor(board):
    # convert board to 8x8x13 tensor
    # channels 0-5: black pieces, 6-11: white pieces, 12: whose turn
    tensor = np.zeros((8, 8, 13), dtype=np.float32)

    for sq in chess.SQUARES:
        piece = board.piece_at(sq)
        if piece:
            r, c = chess.square_rank(sq), chess.square_file(sq)
            tensor[r, c, PIECE_CHANNEL[piece.symbol()]] = 1.0

    tensor[:, :, 12] = float(board.turn)
    return np.expand_dims(tensor, axis=0)


def create_policy_vector(moves, probs):
    # map move probabilities into a full 4672-length policy vector
    policy = np.zeros(4672, dtype=np.float32)
    for move, prob in zip(moves, probs):
        policy[move_to_index(move)] = prob
    return policy
