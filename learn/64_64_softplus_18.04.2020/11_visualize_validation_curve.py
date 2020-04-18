from sklearn.model_selection import validation_curve
import numpy as np
from matplotlib import pyplot as plt
import numpy
import pandas as pd
import time
from keras.models import Sequential
from keras import regularizers
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from keras import regularizers
from keras.layers import Dense, Dropout, Activation

df = pd.read_csv("./marker_recorder_data.csv")
# get rid of any rows that doesnt have one contour for each mask
# red_ball_side_red_cx,red_ball_side_red_cy,red_ball_side_red_quantity,red_ball_upper_red_cx,red_ball_upper_red_cy,red_ball_upper_red_quantity,x,y,z
dataset = df.dropna()
# df = df.head(20)

# X - input has current object position and current arm position
X = dataset[['upper_red_c_x', 'upper_red_c_y', 'side_red_c_x', 'side_red_c_y']]
# Y - output has arm joints paramaters in both positions left - desired, right - current
Y = dataset[['x', 'y', 'z']]

start_time = time.time()
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
#     model.add(Dense(64, input_dim=4, kernel_initializer='normal', activation='relu'))
#     model.add(Dense(64, kernel_initializer='normal', activation='relu'))
#     model.add(Dense(3, kernel_initializer='normal', activation='linear'))
#     # Compile model
#     model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
#     return model

# def model_base():
#     # create model
#     model = Sequential()
#     model.add(Dense(64, input_dim=4, kernel_initializer='normal', activation='relu'))
#     model.add(Dropout(0.5))
#     model.add(Dense(64, kernel_initializer='normal', activation='relu'))
#     model.add(Dropout(0.5))
#     model.add(Dense(3, kernel_initializer='normal', activation='linear'))
#     # Compile model
#     model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
#     return model

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


# fix random seed for reproducibility
seed = 2020
numpy.random.seed(seed)

x_scaler = StandardScaler()
y_scaler = StandardScaler()
X_scaled = x_scaler.fit_transform(X)
Y_scaled = y_scaler.fit_transform(Y)

# Fit the model
history = model_base().fit(X_scaled, Y_scaled, validation_split=0.25, epochs=2000, batch_size=4, verbose=1)
# list all data in history
print(history.history.keys())

# summarize history MAE
plt.figure(1)
plt.plot(history.history['mae'])
plt.plot(history.history['val_mae'])
plt.title('mean_absolute_error')
plt.ylabel('mean_absolute_error')
plt.xlabel('epoch')
plt.legend(['train', 'val'], loc='upper left')

# summarize history for MSE (loss)
plt.figure(2)
plt.plot(history.history['loss'][-1500:])
plt.plot(history.history['val_loss'][-1500:])
plt.title('MSE (loss)')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'validate'], loc='upper left')
plt.show()

