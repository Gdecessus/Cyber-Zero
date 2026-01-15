import numpy as np
import chess


def move_to_index(move):
    from_sq = move.from_square
    to_sq = move.to_square
    
    if move.promotion:
        promo_map = {
            chess.KNIGHT: 0,
            chess.BISHOP: 1,
            chess.ROOK: 2,
            chess.QUEEN: 3
        }
        promo_idx = promo_map[move.promotion]
        return 4096 + from_sq * 4 + promo_idx
    
    return from_sq * 64 + to_sq


def board_to_tensor(board):
    tensor = np.zeros((8, 8, 13), dtype=np.float32)
    
    piece_map = {
        'p': 0, 'n': 1, 'b': 2, 'r': 3, 'q': 4, 'k': 5,
        'P': 6, 'N': 7, 'B': 8, 'R': 9, 'Q': 10, 'K': 11
    }
    
    for square in chess.SQUARES:
        piece = board.piece_at(square)
        if piece:
            row = chess.square_rank(square)
            col = chess.square_file(square)
            channel = piece_map[piece.symbol()]
            tensor[row, col, channel] = 1.0
    
    tensor[:, :, 12] = 1.0 if board.turn == chess.WHITE else 0.0
    
    return np.expand_dims(tensor, axis=0)


def create_policy_vector(legal_moves, move_probs):
    policy = np.zeros(4672, dtype=np.float32)
    
    for move, prob in zip(legal_moves, move_probs):
        idx = move_to_index(move)
        policy[idx] = prob
    
    if np.sum(policy) > 0:
        policy /= np.sum(policy)
    
    return policy
