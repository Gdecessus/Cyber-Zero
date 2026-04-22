from tensorflow import keras
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Dense, Conv2D, Flatten, BatchNormalization, Activation, Input, Add
from tensorflow.keras.optimizers import Adam


class ChessModel:

    def __init__(self, lr=0.002, num_res_blocks=4, filters=64):
        self.lr = lr
        self.num_res_blocks = num_res_blocks
        self.filters = filters
        self.model = self._build()

    def _res_block(self, x):
        # save the input so we can add it back at the end (the "skip connection" —
        # this is what makes the network residual and lets deeper nets train)
        shortcut = x

        # first conv-batchnorm-relu: extracts features at this depth
        x = Conv2D(self.filters, 3, padding='same', use_bias=False)(x)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)

        # second conv-batchnorm (no relu yet, it goes after the skip add)
        x = Conv2D(self.filters, 3, padding='same', use_bias=False)(x)
        x = BatchNormalization()(x)

        # add the original input back, then relu — this lets gradients flow
        # through the network even when it's deep
        x = Add()([shortcut, x])
        x = Activation('relu')(x)
        return x

    def _build(self):
        # input is the board: 8x8 with 13 channels (one per piece type for both
        # colours, plus one channel that says whose turn it is)
        inp = Input(shape=(8, 8, 13), name='board_input')

        # initial conv to lift the input from 13 channels to `filters` channels,
        # giving the residual blocks more working space
        x = Conv2D(self.filters, 3, padding='same', use_bias=False)(inp)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)

        # stack of residual blocks — the "trunk" of the network where most
        # of the position understanding actually happens
        for _ in range(self.num_res_blocks):
            x = self._res_block(x)

        # policy head: outputs a probability for each of 4672 possible moves
        # (covers all normal moves and promotions). Softmax so the values sum to 1.
        p = Conv2D(2, 1, padding='same', use_bias=False)(x)
        p = BatchNormalization()(p)
        p = Activation('relu')(p)
        p = Flatten()(p)
        p = Dense(4672, activation='softmax', name='policy')(p)

        # value head: outputs a single score in [-1, +1] (tanh) representing
        # how good the position looks for the side to move
        v = Conv2D(1, 1, padding='same', use_bias=False)(x)
        v = BatchNormalization()(v)
        v = Activation('relu')(v)
        v = Flatten()(v)
        v = Dense(128, activation='relu')(v)
        v = Dense(1, activation='tanh', name='value')(v)

        # build the full model with two outputs and compile with two losses:
        # cross-entropy for the move probabilities, MSE for the position score
        model = Model(inputs=inp, outputs=[p, v])
        model.compile(
            optimizer=Adam(learning_rate=self.lr),
            loss={'policy': 'categorical_crossentropy', 'value': 'mse'},
            metrics={'policy': 'accuracy'}
        )
        return model

    def predict(self, state):
        # run the model on a board state, returns (policy_probs, value_score)
        return self.model.predict(state, verbose=0)

    def train(self, X, y, epochs=1, batch_size=32):
        return self.model.fit(X, y, epochs=epochs, batch_size=batch_size)

    def save(self, path):
        self.model.save(path)

    @staticmethod
    def load(path):
        # bypass __init__ (which would build a fresh untrained model) and load the
        # trained Keras model straight from the saved file
        loaded = ChessModel.__new__(ChessModel)
        loaded.model = keras.models.load_model(path)
        return loaded
