import numpy as np
import chess


def board_to_tensor(board):
    # 8x8 board, 12 piece types (6 white + 6 black)
    tensor = np.zeros((8, 8, 12), dtype=np.float32)
    
    # map piece symbols to plane indices
    piece_map = {
        'p': 0,  # black pawn
        'n': 1,  # black knight
        'b': 2,  # black bishop
        'r': 3,  # black rook
        'q': 4,  # black queen
        'k': 5,  # black king
        'P': 6,  # white pawn
        'N': 7,  # white knight
        'B': 8,  # white bishop
        'R': 9,  # white rook
        'Q': 10, # white queen
        'K': 11  # white king
    }
    return tensor