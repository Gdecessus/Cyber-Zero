# ===== Board Encoding =====
# All the inspiration comes from leela zero/alpha zero and a random git I found 
# https://github.com/bhonzik/GeneticChess/blob/main/GeneticChess.py
# https://github.com/LeelaChessZero/lc0/wiki/Technical-Explanation-of-Leela-Chess-Zero
# https://storage.googleapis.com/deepmind-media/DeepMind.com/Blog/alphazero-shedding-new-light-on-chess-shogi-and-go/alphazero_preprint.pdf

def board_to_tensor(board):
    # convert board position to neural network input
    # using planes like AlphaZero (they use a lot of planes, I'll simmer it down)
    # maybe 8x8x13? 12 for pieces and 1 for empty spaces
    
    # loop through squares  
    # create plane for each piece type
    # add plane for whoever turn it is
    
    return tensor  # shape (8,8,13)


def tensor_to_board(tensor):
    # reverse - might need for debugging
    pass


# ===== Move Encoding =====

def move_to_index(move):
    # map chess move to index in policy vector
    # 8x8 = 64 possible moves without promotion (from_square and to_square)
    # simple version: from_square * 64  to_square = 4096 total
    # TODO: Check wether we need all those possible moves since most of them are very likely illegal moves.
    
    return index


def index_to_move(index, board):
    # convert policy index back to move
    # from_sq = index // 64
    # to_sq = index % 64
    
    return move


def get_move_mask(board):
    # create mask for legal moves only (chess library)
    # zero out illegal moves in policy output
    
    # get all legal moves
    
    return mask


# ===== Move Format Conversion =====

def uci_to_tuple(uci_str):
    # "e2e4" -> ((4,1), (4,3)) or something
    # parse UCI format to coordinate pairs and help with deterministic approach for my robotic arm
    
    return (from_pos, to_pos)


def tuple_to_uci(from_pos, to_pos):
    # reverse of above
    return "e2e4"


# ===== Data Augmentation =====

def flip_board_horizontal(board, policy):
    # mirror board left-right
    # AlphaZero does this to double training data
    # need to mirror policy vector too
    
    return flipped_board, flipped_policy


# ===== Training Data =====

def create_sample(board, move_probs, game_result):
    # package training example
    # (position, policy target, value target)
    
    return {
        'position': board_to_tensor(board),
        'policy': move_probs,
        'value': game_result  # 1/-1/0 (1 win, -1 loss and 0 draw)
    }


def batch_data(samples, batch_size):
    # group samples for training
    
    return batches


# ===== Helpers =====

def get_turn(board):
    # return whose turn it is
    return 1 or -1  # white or black


def print_board(board):
    pass
