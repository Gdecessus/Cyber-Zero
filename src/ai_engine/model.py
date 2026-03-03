import tensorflow as tf
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
        shortcut = x
        x = Conv2D(self.filters, 3, padding='same', use_bias=False)(x)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)
        x = Conv2D(self.filters, 3, padding='same', use_bias=False)(x)
        x = BatchNormalization()(x)
        x = Add()([shortcut, x])
        x = Activation('relu')(x)
        return x

    def _build(self):
        inp = Input(shape=(8, 8, 13), name='board_input')

        x = Conv2D(self.filters, 3, padding='same', use_bias=False)(inp)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)

        for _ in range(self.num_res_blocks):
            x = self._res_block(x)

        # policy head
        p = Conv2D(2, 1, padding='same', use_bias=False)(x)
        p = BatchNormalization()(p)
        p = Activation('relu')(p)
        p = Flatten()(p)
        p = Dense(4672, activation='softmax', name='policy')(p)

        # value head
        v = Conv2D(1, 1, padding='same', use_bias=False)(x)
        v = BatchNormalization()(v)
        v = Activation('relu')(v)
        v = Flatten()(v)
        v = Dense(128, activation='relu')(v)
        v = Dense(1, activation='tanh', name='value')(v)

        model = Model(inputs=inp, outputs=[p, v])
        model.compile(
            optimizer=Adam(learning_rate=self.lr),
            loss={'policy': 'categorical_crossentropy', 'value': 'mse'},
            metrics={'policy': 'accuracy'}
        )
        return model

    def predict(self, state):
        return self.model.predict(state, verbose=0)

    def train(self, X, y, epochs=1, batch_size=32):
        return self.model.fit(X, y, epochs=epochs, batch_size=batch_size)

    def save(self, path):
        self.model.save(path)

    @staticmethod
    def load(path):
        """Load a saved model from disk without rebuilding it."""
        # create an empty ChessModel shell (skip __init__ since we don't need to build)
        loaded = ChessModel.__new__(ChessModel)
        loaded.model = keras.models.load_model(path)
        return loaded
