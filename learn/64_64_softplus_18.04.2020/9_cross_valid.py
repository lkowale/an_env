import numpy as np
import pandas as pd
import time
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from keras import regularizers
from keras.layers import Activation
from sklearn.externals import joblib
import os


df = pd.read_csv("./marker_recorder_data.csv")
# get rid of any rows that doesnt have one contour for each mask
# red_ball_side_red_cx,red_ball_side_red_cy,red_ball_side_red_quantity,red_ball_upper_red_cx,red_ball_upper_red_cy,red_ball_upper_red_quantity,x,y,z
dataset = df.dropna()
# df = df.head(20)

# X - input has current object position and current arm position
X = dataset[['upper_red_c_x', 'upper_red_c_y', 'side_red_c_x', 'side_red_c_y']]
# Y - output has arm joints paramaters in both positions left - desired, right - current
Y = dataset[['x', 'y', 'z']]

# # define base model
# def model_base():
#     # create model
#     model = Sequential()
#     model.add(Dense(64, input_dim=4, kernel_initializer='normal', activation='relu'))
#     # model.add(Dense(64, kernel_initializer='normal', activation='relu', kernel_regularizer=regularizers.l2(0.001)))
#     model.add(Dense(64, kernel_initializer='normal', activation='relu', activity_regularizer=regularizers.l2(0.001)))
#     # model.add(Dense(64, kernel_initializer='normal', activation='relu'))
#     model.add(Dense(3, kernel_initializer='normal', activation='linear'))
#     # Compile model
#     model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
#     return model

# # define base model
# def model_base():
#     # create model
#     model = Sequential()
#     model.add(Dense(64, input_dim=4, kernel_initializer='normal', activation='relu'))
#     # model.add(Dense(64, kernel_initializer='normal', activation='relu', kernel_regularizer=regularizers.l2(0.001)))
#     model.add(Dense(64, kernel_initializer='normal', activation='linear', activity_regularizer=regularizers.l2(0.001)))
#     model.add(Activation('relu'))
#     # model.add(Dense(64, kernel_initializer='normal', activation='relu'))
#     model.add(Dense(3, kernel_initializer='normal', activation='linear'))
#     # Compile model
#     model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
#     return model

def model_base():
    # create model
    model = Sequential()
    model.add(Dense(64, input_dim=4, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(64, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(3, kernel_initializer='normal', activation='linear'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
    return model

# def model_base():
#     # create model
#     model = Sequential()
#     model.add(Dense(32, input_dim=4, kernel_initializer='normal', activation='softplus'))
#     model.add(Dense(32, kernel_initializer='normal', activation='softplus'))
#     model.add(Dense(3, kernel_initializer='normal', activation='linear'))
#     # Compile model
#     model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
#     return model

seed = 2020
np.random.seed(seed)
estimators = []
estimators.append(('standardize', StandardScaler()))
epochs = 2000
batch_size = 4
estimators.append(('regresor', KerasRegressor(build_fn=model_base, epochs=epochs, batch_size=batch_size, verbose=1)))
pipeline = Pipeline(estimators)
kfold = KFold(n_splits=4, shuffle=True, random_state=seed)

start_time = time.time()
results = cross_val_score(pipeline, X, Y, cv=kfold)
elapsed_time = time.time() - start_time

with open(os.path.basename(__file__), 'a') as fd:
    model = model_base()
    model_name = "model"
    for layer in model._layers[1:]:
        if layer.name.startswith('dense'):
            app = str(layer.units)
        if layer.name.startswith('activ'):
            app = 'actv'
        if layer.name.startswith('drop'):
            app = 'dp' + str(layer.rate)

        model_name = model_name + '_' + app
        
    model_name = model_name + '_b' + str(batch_size)
    model_name = model_name + ' Epochs:' + str(epochs)
    message = "\n# " + model_name + " Std: {:.2f} ({:.2f}) MSE  Elapsed time:{}".format(results.mean(), results.std(), time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))

    fd.write(message)

print(results)

# pipeline.fit(X, Y)
print(" Std: %.2f (%.2f) MSE" % (results.mean(), results.std()))
# your script

print("Elapsed time " + time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))
# model_128_3_b4 Epochs:10Standardized: -46308.81142064145 (15896.710684090434) MSE  Elapsed time:00:00:01
# model_128_3_b4 Epochs:10Standardized: -46306.98386101974 (15898.031207730335) MSE  Elapsed time:00:00:01
# model_128_3_b4 Epochs:10 Std: -46311.91064967105 (15895.769842239122) MSE  Elapsed time:00:00:01
# model_128_3_b4 Epochs:1000 Std: -798.694503001163 (625.3287909576684) MSE  Elapsed time:00:00:53
# model_128_3_b4 Epochs:2000 Std: -685.5114639282226 (608.7400223957309) MSE  Elapsed time:00:01:47
# model_128_3_b4 Epochs:2000 Std: -111.26682401958264 (31.567167177196932) MSE  Elapsed time:00:01:46           <--- after adding 'shuffle=True' to kfold
# model_128_3_b4 Epochs:2000 Std: -110.79731224461605 (31.84930797024875) MSE  Elapsed time:00:01:49            <--- kernel_regularizer=regularizers.l2(0.001)
# model_128_3_b4 Epochs:5000 Std: -29.806802026849045 (5.396185201721931) MSE  Elapsed time:00:04:19
# model_128_3_b4 Epochs:10000 Std: -26.306730671932826 (2.5053148434469854) MSE  Elapsed time:00:08:50


# model_64_64_3_b4 Epochs:5000 Std: -33.54 (7.19) MSE  Elapsed time:00:04:29
# model_64_64_3_b4 Epochs:5000 Std: -33.75 (12.47) MSE  Elapsed time:00:04:33           kernel_regularizer=regularizers.l2(0.001)
# model_64_64_3_b4 Epochs:5000 Std: -41.95 (12.64) MSE  Elapsed time:00:04:40           activity_regularizer=regularizers.l2(0.001))

# model_64_64_actv_3_b4 Epochs:5000 Std: -35.00 (2.29) MSE  Elapsed time:00:04:35
# model_64_actv_64_actv_3_b4 Epochs:5000 Std: -497.06 (43.64) MSE  Elapsed time:00:05:06
# model_64_dp0.5_64_dp0.5_3_b4 Epochs:5000 Std: -491.37 (46.97) MSE  Elapsed time:00:05:05
# model_64_dp0.2_64_dp0.2_3_b4 Epochs:5000 Std: -134.63 (22.59) MSE  Elapsed time:00:05:12
# model_64_dp0.1_64_dp0.1_3_b4 Epochs:5000 Std: -90.91 (9.23) MSE  Elapsed time:00:05:19
# model_64_dp0.05_64_dp0.05_3_b4 Epochs:5000 Std: -68.61 (15.45) MSE  Elapsed time:00:05:08
# model_64_dp0.05_64_dp0.05_3_b4 Epochs:5000 Std: -68.82 (17.16) MSE  Elapsed time:00:05:04
# model_64_dp0.05_64_dp0.05_3_b4 Epochs:5000 Std: -40.50 (8.65) MSE  Elapsed time:00:05:14      changed activation from relu to elu
# model_64_64_3_b4 Epochs:5000 Std: -21.63 (6.22) MSE  Elapsed time:00:04:34
# model_64_64_3_b4 Epochs:5000 Std: -527.56 (153.99) MSE  Elapsed time:00:04:39                 changed activation from elu to linear
# model_32_32_3_b4 Epochs:1000 Std: -242.03 (87.25) MSE  Elapsed time:00:00:57
# model_32_32_3_b4 Epochs:5000 Std: -16.07 (5.74) MSE  Elapsed time:00:04:39
# model_64_64_3_b4 Epochs:5000 Std: -18.00 (5.76) MSE  Elapsed time:00:04:39
##### 18.04.2020
# model_64_64_3_b4 Epochs:100 Std: -7480.96 (617.35) MSE  Elapsed time:00:00:05
# model_64_64_3_b4 Epochs:1000 Std: -130.92 (43.91) MSE  Elapsed time:00:00:40
# model_32_32_3_b4 Epochs:1000 Std: -433.88 (185.85) MSE  Elapsed time:00:00:41
# model_64_64_3_b16 Epochs:1000 Std: -707.30 (160.14) MSE  Elapsed time:00:00:14
# model_64_64_3_b4 Epochs:2000 Std: -46.71 (28.55) MSE  Elapsed time:00:01:19