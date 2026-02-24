import tensorflow as tf
from tensorflow import keras
from keras.models import Model
from keras.layers import Dense, Conv2D, Flatten, BatchNormalization, Activation, Input, Add
from keras.optimizers import Adam


class ChessModel:
    def __init__(self, learning_rate=0.002, num_residual_blocks=4, num_filters=64):
        self.learning_rate = learning_rate
        self.num_residual_blocks = num_residual_blocks
        self.num_filters = num_filters
        self.model = self._build_model()
    
    def _residual_block(self, x):
        shortcut = x
        
        x = Conv2D(self.num_filters, 3, padding='same', use_bias=False)(x)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)
        
        x = Conv2D(self.num_filters, 3, padding='same', use_bias=False)(x)
        x = BatchNormalization()(x)
        
        x = Add()([shortcut, x])
        x = Activation('relu')(x)
        
        return x 
    def _build_model(self):
        board_input = Input(shape=(8, 8, 13), name='board_input')
        
        x = Conv2D(self.num_filters, 3, padding='same', use_bias=False)(board_input)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)
        
        for _ in range(self.num_residual_blocks):
            x = self._residual_block(x)
        
        policy = Conv2D(2, 1, padding='same', use_bias=False)(x)
        policy = BatchNormalization()(policy)
        policy = Activation('relu')(policy)
        policy = Flatten()(policy)
        policy = Dense(4672, activation='softmax', name='policy')(policy)
        
        value = Conv2D(1, 1, padding='same', use_bias=False)(x)
        value = BatchNormalization()(value)
        value = Activation('relu')(value)
        value = Flatten()(value)
        value = Dense(128, activation='relu')(value)
        value = Dense(1, activation='tanh', name='value')(value)
        
        model = Model(inputs=board_input, outputs=[policy, value])
        
        model.compile(
            optimizer=Adam(learning_rate=self.learning_rate),
            loss={'policy': 'categorical_crossentropy', 'value': 'mse'},
            metrics={'policy': 'accuracy'}
        )
        
        return model
    def predict():
    
    def fit():
  
    
    def save():
        
    
    def load():
       
    def load_from_file():
        