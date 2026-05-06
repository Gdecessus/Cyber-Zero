import numpy as np
import chess
from src.utils.utils import move_to_index, board_to_tensor, create_policy_vector


def test_normal_move_index():
    move = chess.Move.from_uci("e2e4")
    idx = move_to_index(move)
    assert idx == move.from_square * 64 + move.to_square


def test_promotion_index_in_range():
    move = chess.Move.from_uci("a7a8q")
    idx = move_to_index(move)
    assert 4096 <= idx < 4672


def test_different_promotions_give_different_indices():
    indices = set()
    for promo in ["q", "r", "b", "n"]:
        indices.add(move_to_index(chess.Move.from_uci(f"a7a8{promo}")))
    assert len(indices) == 4


def test_board_tensor_shape():
    board = chess.Board()
    tensor = board_to_tensor(board)
    assert tensor.shape == (1, 8, 8, 13)


def test_board_tensor_piece_count():
    board = chess.Board()
    tensor = board_to_tensor(board)
    assert np.sum(tensor[0, :, :, :12] > 0) == 32


def test_board_tensor_turn_white():
    board = chess.Board()
    tensor = board_to_tensor(board)
    assert tensor[0, 0, 0, 12] == 1.0


def test_board_tensor_turn_black():
    board = chess.Board()
    board.push_uci("e2e4")
    tensor = board_to_tensor(board)
    assert tensor[0, 0, 0, 12] == 0.0


def test_policy_vector_shape():
    move = chess.Move.from_uci("e2e4")
    policy = create_policy_vector([move], [1.0])
    assert policy.shape == (4672,)


def test_policy_vector_sums_correctly():
    moves = [chess.Move.from_uci("e2e4"), chess.Move.from_uci("d2d4")]
    policy = create_policy_vector(moves, [0.6, 0.4])
    assert abs(np.sum(policy) - 1.0) < 1e-6
