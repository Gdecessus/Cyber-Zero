import numpy as np
import chess
from src.utils.utils import move_to_index, board_to_tensor


class MCTSNode:
    def __init__(self, board, parent=None, move=None, prior=1.0):
        self.board = board.copy()
        self.parent = parent
        self.move = move
        self.prior = prior
        self.visits = 0
        self.value_sum = 0.0
        self.children = {}

    def is_leaf(self):
        return len(self.children) == 0

    def is_terminal(self):
        return self.board.is_game_over()

    def q(self):
        if self.visits == 0:
            return 0  # avoid div by zero
        return self.value_sum / self.visits

    def ucb(self, c=1.4):
        # unvisited nodes always get explored first
        if self.visits == 0:
            return float('inf')
        exploit = self.q()
        explore = c * self.prior * np.sqrt(self.parent.visits) / (1 + self.visits)
        return exploit + explore

    def best_child(self):
        return max(self.children.values(), key=lambda n: n.ucb())

    def expand(self, policy):
        """Create child nodes for all legal moves, weighted by policy network."""
        moves = list(self.board.legal_moves)

        # grab prior prob for each legal move from the network output
        priors = [policy[move_to_index(m)] for m in moves]
        total = sum(priors)

        if total > 0:
            priors = [p / total for p in priors]
        else:
            # network gave us nothing, just use uniform
            priors = [1.0 / len(moves)] * len(moves)

        for m, p in zip(moves, priors):
            child_board = self.board.copy()
            child_board.push(m)
            self.children[m] = MCTSNode(child_board, parent=self, move=m, prior=p)

    def backprop(self, value):
        self.visits += 1
        self.value_sum += value
        # flip sign going up the tree (opponent's perspective)
        if self.parent:
            self.parent.backprop(-value)


class MCTS:
    def __init__(self, model, n_sims=100, c=1.4):
        self.model = model
        self.n_sims = n_sims
        self.c = c

    def search(self, board):
        root = MCTSNode(board)

        for _ in range(self.n_sims):
            node = root

            # 1) selection - walk down tree picking best UCB child
            while not node.is_leaf() and not node.is_terminal():
                node = node.best_child()

            # 2) evaluation
            if node.is_terminal():
                result = node.board.result()
                if result == '1-0':
                    val = 1.0 if node.board.turn == chess.BLACK else -1.0
                elif result == '0-1':
                    val = -1.0 if node.board.turn == chess.BLACK else 1.0
                else:
                    val = 0
            else:
                # ask the neural network
                state = board_to_tensor(node.board)
                policy, val = self.model.predict(state, verbose=0)
                policy = policy[0]  # remove batch dim
                val = val[0][0]

                if node.is_leaf():
                    node.expand(policy)

            # 3) backpropagate result up the tree
            node.backprop(val)

        # collect visit counts for each legal move at the root
        moves = list(board.legal_moves)
        visits = [root.children[m].visits if m in root.children else 0 for m in moves]
        return moves, visits

    def get_move_probs(self, board, temperature=1.0):
        moves, visits = self.search(board)
        if not moves:
            return [], []

        if temperature == 0:
            # greedy - just pick the most visited move
            best = np.argmax(visits)
            probs = [0.0] * len(moves)
            probs[best] = 1.0
        else:
            counts = np.array(visits, dtype=np.float32)
            counts = counts ** (1.0 / temperature)
            total = np.sum(counts)
            probs = counts / total if total > 0 else np.ones(len(moves)) / len(moves)

        return moves, probs
