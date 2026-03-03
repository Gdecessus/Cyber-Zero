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
            return 0
        return self.value_sum / self.visits

    def ucb(self, c=1.4):
        if self.visits == 0:
            return float('inf')
        return self.q() + c * self.prior * np.sqrt(self.parent.visits) / (1 + self.visits)

    def best_child(self):
        # pick the child with the highest UCB score
        best = None
        best_score = -1
        for child in self.children.values():
            score = child.ucb()
            if score > best_score:
                best_score = score
                best = child
        return best

    def expand(self, policy):
        # get what the neural network thinks about each legal move
        moves = list(self.board.legal_moves)

        priors = []
        for m in moves:
            priors.append(policy[move_to_index(m)])

        # normalize so they add up to 1 (or equal if all zero)
        total = sum(priors)
        if total > 0:
            for i in range(len(priors)):
                priors[i] = priors[i] / total
        else:
            even = 1.0 / len(moves)
            priors = [even] * len(moves)

        # create a child node for each legal move
        for i in range(len(moves)):
            new_board = self.board.copy()
            new_board.push(moves[i])
            self.children[moves[i]] = MCTSNode(new_board, parent=self, move=moves[i], prior=priors[i])

    def backprop(self, value):
        node = self
        while node is not None:
            node.visits += 1
            node.value_sum += value
            value = -value  # flip for opponent
            node = node.parent


class MCTS:

    def __init__(self, model, n_sims=100):
        self.model = model
        self.n_sims = n_sims

    def search(self, board):
        root = MCTSNode(board)

        for _ in range(self.n_sims):
            node = root

            # selection
            while not node.is_leaf() and not node.is_terminal():
                node = node.best_child()

            # evaluation
            if node.is_terminal():
                # figure out who won (turn belongs to the side that CAN'T move)
                result = node.board.result()
                if result == '1-0':
                    # white won
                    if node.board.turn == chess.BLACK:
                        val = 1.0
                    else:
                        val = -1.0
                elif result == '0-1':
                    # black won
                    if node.board.turn == chess.BLACK:
                        val = -1.0
                    else:
                        val = 1.0
                else:
                    val = 0
            else:
                state = board_to_tensor(node.board)
                policy, val = self.model.predict(state)
                policy = policy[0]
                val = val[0][0]
                node.expand(policy)

            node.backprop(val)

        moves = list(board.legal_moves)
        visits = []
        for m in moves:
            if m in root.children:
                visits.append(root.children[m].visits)
            else:
                visits.append(0)
        return moves, visits

    def get_move_probs(self, board, temperature=1.0):
        moves, visits = self.search(board)
        if not moves:
            return [], []

        if temperature == 0:
            best = np.argmax(visits)
            probs = [0.0] * len(moves)
            probs[best] = 1.0
        else:
            counts = np.array(visits, dtype=np.float32)
            counts = counts ** (1.0 / temperature)
            total = np.sum(counts)
            if total > 0:
                probs = counts / total
            else:
                probs = np.ones(len(moves)) / len(moves)

        return moves, probs
