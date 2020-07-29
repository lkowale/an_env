# https://machinelearningmastery.com/regression-tutorial-keras-deep-learning-library-python/
import numpy
import pandas as pd
import time
import os
import sys
from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.externals import joblib
from keras.models import load_model


# todo models manager, saving and loading models in database
class KerasModel:
    def __init__(self, model_name):
        self.pipeline = None
        self.model_name = model_name
        # self.model_path = sys.argv[0]
        self.model_path = '/home/aa/an_env/arm_1/model'

    # define base model
    def model_base(self):
        # create model
        model = Sequential()
        model.add(Dense(64, input_dim=4, kernel_initializer='normal', activation='softplus'))
        model.add(Dense(64, kernel_initializer='normal', activation='softplus'))
        model.add(Dense(3, kernel_initializer='normal', activation='linear'))
        # Compile model
        model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
        return model

    def build_pipeline(self):
        # fix random seed for reproducibility
        seed = 2020

        numpy.random.seed(seed)
        estimators = []
        estimators.append(('standardize', StandardScaler()))
        estimators.append(('regresor', KerasRegressor(build_fn=self.model_base, epochs=20, batch_size=4, verbose=1)))
        self.pipeline = Pipeline(estimators)

    def train(self):
        df = pd.read_csv("../marker_recorder_data.csv")
        # get rid of any rows that doesnt have one contour for each mask
        # red_ball_side_red_cx,red_ball_side_red_cy,red_ball_side_red_quantity,red_ball_upper_red_cx,red_ball_upper_red_cy,red_ball_upper_red_quantity,x,y,z
        dataset = df.dropna()
        # df = df.head(20)

        # X - input has current object position and current arm position
        X = dataset[['upper_red_c_x', 'upper_red_c_y', 'side_red_c_x', 'side_red_c_y']]
        # Y - output has arm joints paramaters in both positions left - desired, right - current
        Y = dataset[['x', 'y', 'z']]
        start_time = time.time()
        self.pipeline.fit(X, Y)

    def save(self):

        # model_name = 'model_32x32_b4_e2200'

        model_file_name = self.model_name + '.h5'
        pipeline_file_name = 'pipeline_' + self.model_name + '.pkl'

        # Save the Keras model first:
        self.pipeline.named_steps['regresor'].model.save(model_file_name)
        # This hack allows us to save the sklearn pipeline:
        self.pipeline.named_steps['regresor'].model = None
        # Finally, save the pipeline:
        # joblib.dump(pipeline, pipeline_file_name)
        # used protocol=2
        joblib.dump(self.pipeline, pipeline_file_name, protocol=2)

    def load(self):
        path = self.model_path + '/'
        self.pipeline = joblib.load(path + 'pipeline_' + self.model_name + '.pkl')
        self.pipeline.named_steps['regresor'].model = load_model(path + self.model_name + '.h5')


if __name__ == '__main__':
    model = KerasModel('model_64x64_b4_e2000_p2')
    model.build_pipeline()
    model.train()
    model.save()

    model1 = KerasModel('model_64x64_b4_e2000_p2')
    model1.load()

