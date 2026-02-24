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

    def predict():
    
    def fit():
  
    
    def save():
        
    
    def load():
       
    def load_from_file():
        