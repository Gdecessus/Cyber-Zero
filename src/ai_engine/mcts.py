import numpy as np
import chess
from src.utils.utils import move_to_index, board_to_tensor


# one node in the search tree = one board position
class MCTSNode:

    def __init__(self, board, parent=None, move=None, prior=1.0):
        self.board = board.copy()
        self.parent = parent
        self.move = move
        self.prior = prior
        self.visits = 0
        self.total_value = 0.0
        self.children = {}

    def is_leaf(self):
        return len(self.children) == 0

    def is_game_over(self):
        return self.board.is_game_over()

    def average_value(self):
        if self.visits == 0:
            return 0
        return self.total_value / self.visits

    def exploration_score(self, explore_weight=1.4):
        if self.visits == 0:
            return float('inf')

        value_score = self.average_value()
        explore_bonus = explore_weight * self.prior * np.sqrt(self.parent.visits) / (1 + self.visits)
        return value_score + explore_bonus

    def best_child(self):
        best_node = None
        best_score = -1
        for child in self.children.values():
            score = child.exploration_score()
            if score > best_score:
                best_score = score
                best_node = child
        return best_node

    def expand(self, policy):
        moves = list(self.board.legal_moves)

        # get neural net's prior for each move, then normalize
        priors = []
        for m in moves:
            priors.append(policy[move_to_index(m)])

        total = sum(priors)
        if total > 0:
            for i in range(len(priors)):
                priors[i] = priors[i] / total
        else:
            even = 1.0 / len(moves)
            priors = [even] * len(moves)

        for i in range(len(moves)):
            new_board = self.board.copy()
            new_board.push(moves[i])
            self.children[moves[i]] = MCTSNode(
                new_board,
                parent=self,
                move=moves[i],
                prior=priors[i]
            )

    def propagate_value(self, value):
        # walk back up to root, flipping value each level
        node = self
        while node is not None:
            node.visits += 1
            node.total_value += value
            value = -value
            node = node.parent


# runs many simulations to figure out the best move
class MCTS:

    def __init__(self, model, n_sims=100):
        self.model = model
        self.n_sims = n_sims

    def search(self, board):
        root = MCTSNode(board)

        for _ in range(self.n_sims):

            # SELECT
            node = root
            while not node.is_leaf() and not node.is_game_over():
                node = node.best_child()

            # EVALUATE
            if node.is_game_over():
                val = self._get_terminal_value(node)
            else:
                state = board_to_tensor(node.board)
                policy, val = self.model.predict(state)
                policy = policy[0]
                val = val[0][0]

                # EXPAND
                node.expand(policy)

            # BACKPROPAGATE
            node.propagate_value(val)

        # collect visit counts
        moves = list(board.legal_moves)
        visits = []
        for m in moves:
            if m in root.children:
                visits.append(root.children[m].visits)
            else:
                visits.append(0)
        return moves, visits

    def _get_terminal_value(self, node):
        # game is over — return +1, -1, or 0
        # board.turn is the side that WOULD move next (but can't)
        result = node.board.result()

        if result == '1-0':
            if node.board.turn == chess.BLACK:
                return 1.0
            else:
                return -1.0

        elif result == '0-1':
            if node.board.turn == chess.BLACK:
                return -1.0
            else:
                return 1.0

        return 0.0

    def get_move_probs(self, board, temperature=1.0):
        # temperature: 0 = always best move, 1.0 = more random
        moves, visits = self.search(board)
        if not moves:
            return [], []

        if temperature == 0:
            best_index = 0
            for i in range(len(visits)):
                if visits[i] > visits[best_index]:
                    best_index = i

            probs = [0.0] * len(moves)
            probs[best_index] = 1.0
        else:
            # scale visits by temperature, then normalize
            probs = []
            for v in visits:
                probs.append(v ** (1.0 / temperature))

            total = sum(probs)
            if total > 0:
                for i in range(len(probs)):
                    probs[i] = probs[i] / total
            else:
                even = 1.0 / len(moves)
                probs = [even] * len(moves)

        return moves, probs
