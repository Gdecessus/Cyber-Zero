import numpy as np
import chess
from src.ai_engine.mcts import MCTSNode


def test_new_node_is_leaf():
    node = MCTSNode(chess.Board())
    assert node.is_leaf()


def test_node_copies_board():
    board = chess.Board()
    node = MCTSNode(board)
    board.push_uci("e2e4")
    assert node.board.fen() != board.fen()


def test_expand_creates_children():
    board = chess.Board()
    node = MCTSNode(board)
    policy = np.ones(4672) / 4672
    node.expand(policy)
    assert len(node.children) == len(list(board.legal_moves))
    assert not node.is_leaf()


def test_propagate_value_flips():
    board = chess.Board()
    parent = MCTSNode(board)
    policy = np.ones(4672) / 4672
    parent.expand(policy)

    child = list(parent.children.values())[0]
    child.propagate_value(1.0)

    assert child.total_value == 1.0
    assert parent.total_value == -1.0


def test_game_over_detection():
    # fool's mate
    board = chess.Board()
    for m in ["f2f3", "e7e5", "g2g4", "d8h4"]:
        board.push_uci(m)
    node = MCTSNode(board)
    assert node.is_game_over()
