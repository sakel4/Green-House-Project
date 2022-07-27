import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.tree import DecisionTreeClassifier
import pickle

filename = 'finalized_model.sav'
from micromlgen import port
    
def createAndTrain():
    csv = pd.read_csv('./Final_csv.csv')
    x = csv.iloc[:, :-1].values
    y = csv.iloc[:, -1].values
    # print(x);
    # print(y);

    X_train, X_test, y_train, y_test = train_test_split(x, y, test_size = 0.25, random_state = 0)

    sc = StandardScaler()
    X_train = sc.fit_transform(X_train)
    X_test = sc.transform(X_test)

    model = DecisionTreeClassifier(criterion = 'entropy', random_state = 0)
    clf = model.fit(X_train, y_train)

    print(model.predict(sc.transform([[57,125]])))

    y_pred = model.predict(X_test)
    # save the model to disk
    pickle.dump(model, open(filename, 'wb'))
    
def loadModelAndPredict():
    loaded_model = pickle.load(open(filename, 'rb'))
    result = loaded_model.score([[76,125]], ['M'])
    print(result)
    
def loadModelAndConvertToC():
    loaded_model = pickle.load(open(filename, 'rb'))
    classmap={
        0: 'Partial',
        1: 'Full',
    }
    c_code = port(loaded_model,classmap=classmap)
    print(c_code)

createAndTrain();
loadModelAndPredict();
loadModelAndConvertToC();