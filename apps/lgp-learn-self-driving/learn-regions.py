#!/usr/bin/env python

import sys, os
import numpy as np
from joblib import dump, load
from sklearn import svm
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt
from mlxtend.plotting import plot_decision_regions

SKELETON_HASH_HEADER = 'skeleton_hash'

def extract_data(f):
  qs_list = []
  max_width = 0

  for line in f.readlines():
    qs = np.fromstring(line, sep=";")
    if len(qs) > 0:
      qs_list.append(qs)
      max_width = max(max_width, qs.shape[0])

  data = np.full([len(qs_list), max_width], np.nan)  # np.loadtxt(f, delimiter=";")

  for i, qs in enumerate(qs_list):
    data[i, 0:len(qs)] = qs
  return data


def extract_header(f):
  header_size = int(f.readline().split(";")[1])
  n_worlds = int(f.readline().split(";")[1])
  qmask = np.array(f.readline().split(";")[1:-1])
  steps_per_phase = int(f.readline().split(";")[1])
  table_header = f.readline().replace('\n', '').split(";")

  print("header_size:{}".format(header_size))
  print("n_worlds:{}".format(n_worlds))
  print("qmask:{}".format(qmask))
  print("steps_per_phase:{}".format(steps_per_phase))
  print("table_header:{}".format(table_header))

  return n_worlds, qmask, steps_per_phase,table_header

def plot_data(XY, figure_filepath=None):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  fig, ax = plt.subplots()
  ax.set_title('All Data', size=16)
  ax.scatter(X[:,0], X[:,1], c=Y)

  for i, xy in enumerate(X):
    ax.annotate(int(Y[i]), (xy[0], xy[1]))

  if figure_filepath:
    fig.savefig(figure_filepath)

def plot_boundaries(clf, XY, figure_filepath=None):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  # Plot Decision Region using mlxtend's awesome plotting function
  fig, ax = plt.subplots()
  plot_decision_regions(X=X,
                        y=Y,
                        ax=ax,
                        clf=clf,
                        legend=2)
  ax.set_title('SVM Decision Region Boundary', size=16)

  if figure_filepath:
    fig.savefig(figure_filepath)

def print_centroids(XY):
  last_col = XY.shape[1] - 1
  Y = XY[:, last_col].astype(int)
  number_of_skeletons = np.max(Y)+1

  centroids = np.zeros(shape=(number_of_skeletons, 2))

  for i in range(np.max(Y)+1):
    X_of_i = XY[XY[:,last_col] == i][:, 0:2]
    centroid = np.average(X_of_i, axis=0)
    print("centroid of skeleton {} is: {},{}".format(i, centroid[0], centroid[1]))

def index_of_skeleton_hash(header):
  for i, h in enumerate(header) :
    if h == SKELETON_HASH_HEADER:
      return i
  return None

def retrieve_classes(header, xyq):
  skeleton_column_index = index_of_skeleton_hash(header)
  XZ = xyq[:, 0: skeleton_column_index+1].copy()

  hashToClass = {}
  for i, hash in enumerate(xyq[:, skeleton_column_index]):
    if not hash in hashToClass:
      hashToClass[hash]=len(hashToClass)
    XZ[i, -1] = hashToClass[hash]

  for hash, id in hashToClass.items():
    print("skeleton: {} <-> {}".format(id, int(hash)))

  return XZ

def separate_data_set(XY):
  training_XY = []
  test_XY = []

  for i, xy in enumerate(XY):
    if i % 2 == 0:
      training_XY.append(xy)
    else:
      test_XY.append(xy)
  return np.array(training_XY), np.array(test_XY)

def learn_model(XY):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  clf = svm.SVC(gamma='scale') #SVC(gamma='auto') #SVC(gamma='scale') #LinearSVC() #class_weight='balanced'  #, C=1.5 #, kernel='poly', degree=1
  print(clf.fit(X, Y))

  return clf

def evaluate_model(clf,XY):
  last_col = XY.shape[1] - 1
  X = XY[:, 0:last_col]
  Y = XY[:, last_col].astype(int)

  Y_pred = clf.predict(X)

  return accuracy_score(Y, Y_pred)

def analyse(dataset_filepath, output_dir):
  with open(dataset_filepath) as f:
    n_worlds, qmask, steps_per_phase, table_header = extract_header(f)
    data = extract_data(f)

  xyq = data[1:,0:data.shape[1]]

  # separate classes
  XZ = retrieve_classes(table_header, xyq)

  plot_data(XZ, figure_filepath=os.path.join(output_dir, "all_data.svg"))

  # separate data
  training_XY, test_XY = separate_data_set(XZ)
  print_centroids(XZ)

  # learn and plot model
  clf = learn_model(training_XY)
  plot_boundaries(clf, training_XY, figure_filepath=os.path.join(output_dir, "decision_regions.svg"))

  # save and log results
  print("accuracy score (training set):{}".format(evaluate_model(clf, training_XY)))
  print("accuracy score (test set):{}".format(evaluate_model(clf, test_XY)))

  dump(clf, os.path.join(output_dir, "learned_regions.joblib"))

  plt.show()

if __name__ == "__main__":
  if len(sys.argv) > 1:
    dataset_filepath = sys.argv[1]
    output_dir = os.path.dirname(dataset_filepath)
    analyse(dataset_filepath, output_dir)


