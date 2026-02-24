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
        # skip connection - keeps gradients flowing through deep networks
        shortcut = x

        x = Conv2D(self.filters, 3, padding='same', use_bias=False)(x)  # BN handles bias
        x = BatchNormalization()(x)
        x = Activation('relu')(x)

        x = Conv2D(self.filters, 3, padding='same', use_bias=False)(x)
        x = BatchNormalization()(x)

        x = Add()([shortcut, x])
        x = Activation('relu')(x)
        return x

    def _build(self):
        # 8x8 board, 12 piece channels + 1 turn channel
        inp = Input(shape=(8, 8, 13), name='board_input')

        x = Conv2D(self.filters, 3, padding='same', use_bias=False)(inp)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)

        for _ in range(self.num_res_blocks):
            x = self._res_block(x)

        # policy head - prob distribution over all possible moves
        policy = Conv2D(2, 1, padding='same', use_bias=False)(x)
        policy = BatchNormalization()(policy)
        policy = Activation('relu')(policy)
        policy = Flatten()(policy)
        policy = Dense(4672, activation='softmax', name='policy')(policy)

        # value head - how good is this position, squeezed to [-1, 1]
        value = Conv2D(1, 1, padding='same', use_bias=False)(x)
        value = BatchNormalization()(value)
        value = Activation('relu')(value)
        value = Flatten()(value)
        value = Dense(128, activation='relu')(value)
        value = Dense(1, activation='tanh', name='value')(value)

        model = Model(inputs=inp, outputs=[policy, value])
        model.compile(
            optimizer=Adam(learning_rate=self.lr),
            loss={'policy': 'categorical_crossentropy', 'value': 'mse'},
            metrics={'policy': 'accuracy'}
        )
        return model

    def predict(self, state, verbose=0):
        return self.model.predict(state, verbose=verbose)

    def train(self, X, y, epochs=1, batch_size=32, verbose=1):
        return self.model.fit(X, y, epochs=epochs, batch_size=batch_size, verbose=verbose)

    def save(self, path):
        self.model.save(path)

    @staticmethod
    def load(path):
        """Load a previously saved model."""
        m = ChessModel()
        m.model = keras.models.load_model(path)
        return m
