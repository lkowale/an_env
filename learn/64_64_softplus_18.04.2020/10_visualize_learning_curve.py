from sklearn.model_selection import learning_curve
import numpy as np
from matplotlib import pyplot as plt
import numpy
import pandas as pd
import time
from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.externals import joblib
from keras import regularizers
from keras.layers import Activation

# https://www.kaggle.com/grfiv4/learning-curves-1
def plot_learning_curve(estimator, title, X, y, ylim=None, cv=None, scoring=None, obj_line=None,
                        n_jobs=1, train_sizes=np.linspace(.1, 1.0, 5)):

    plt.figure()
    plt.title(title)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.xlabel("Training examples")
    plt.ylabel("Score")
    train_sizes, train_scores, test_scores = learning_curve(
        estimator, X, y, cv=cv, scoring=scoring, n_jobs=n_jobs, train_sizes=train_sizes)
    train_scores_mean = np.mean(train_scores, axis=1)
    train_scores_std = np.std(train_scores, axis=1)
    test_scores_mean = np.mean(test_scores, axis=1)
    test_scores_std = np.std(test_scores, axis=1)
    plt.grid()

    plt.fill_between(train_sizes, train_scores_mean - train_scores_std,
                     train_scores_mean + train_scores_std, alpha=0.1,
                     color="r")
    plt.fill_between(train_sizes, test_scores_mean - test_scores_std,
                     test_scores_mean + test_scores_std, alpha=0.1, color="g")
    plt.plot(train_sizes, train_scores_mean, 'o-', color="r",
             label="Training score")
    plt.plot(train_sizes, test_scores_mean, 'o-', color="g",
             label="Cross-validation score")

    if obj_line:
        plt.axhline(y=obj_line, color='blue')

    plt.legend(loc="best")
    return plt



df = pd.read_csv("./marker_recorder_data.csv")
# get rid of any rows that doesnt have one contour for each mask
# red_ball_side_red_cx,red_ball_side_red_cy,red_ball_side_red_quantity,red_ball_upper_red_cx,red_ball_upper_red_cy,red_ball_upper_red_quantity,x,y,z
dataset = df.dropna()
# df = df.head(20)

# X - input has current object position and current arm position
X = dataset[['upper_red_c_x', 'upper_red_c_y', 'side_red_c_x', 'side_red_c_y']]
# Y - output has arm joints paramaters in both positions left - desired, right - current
Y = dataset[['x', 'y', 'z']]

def model_base():
    # create model
    model = Sequential()
    model.add(Dense(64, input_dim=4, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(64, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(3, kernel_initializer='normal', activation='linear'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
    return model


# fix random seed for reproducibility
seed = 2020
np.random.seed(seed)
estimators = []
estimators.append(('standardize', StandardScaler()))
epochs = 200
batch_size = 4
estimators.append(('regresor', KerasRegressor(build_fn=model_base, epochs=epochs, batch_size=batch_size, verbose=1)))
pipeline = Pipeline(estimators)
# kfold = KFold(n_splits=4, random_state=seed)
# results = cross_val_score(pipeline, X, Y, cv=kfold)

start_time = time.time()

# pipeline.fit(X, Y)
# print("Standardized: %.2f (%.2f) MSE" % (results.mean(), results.std()))
# your script
elapsed_time = time.time() - start_time
print("Elapsed time " + time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))

plot_learning_curve(pipeline, 'tt', X, Y).show()
