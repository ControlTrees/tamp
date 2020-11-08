#!/usr/bin/env python

import sys, os
import fnmatch
import shutil

RESULT_DATA_FILE = 'result-data.csv'
SKELETON_FORMATS = ["*.png", "*.gv", "*.po"]

def merge(datasets_dir, output_dir):
  #print ("datasets_dir {}, output_dir {}".format(datasets_dir, output_dir))
  output_file_path = os.path.join(output_dir, RESULT_DATA_FILE)

  # manage output folder
  if os.path.exists(output_file_path):
    os.remove(output_file_path)
  if not os.path.exists(output_dir):
    os.mkdir(output_dir)

  # find result files
  result_files = []
  for root, dirnames, filenames in os.walk(datasets_dir):
    for filename in fnmatch.filter(filenames, RESULT_DATA_FILE):
      result_files.append(os.path.join(root, filename))

  # concatenate files
  with open(output_file_path, 'w') as outfile:
    for filename in result_files:
      with open(filename) as file:
        for line in file:
          outfile.write(line)

  # move skeletons
  for root, dirnames, filenames in os.walk(datasets_dir):
    for ext in SKELETON_FORMATS:
      for filename in fnmatch.filter(filenames, ext):
        if not os.path.exists( output_dir + "/" + filename ):
          shutil.copy(root + "/" + filename, output_dir)

if __name__ == "__main__":
  if len(sys.argv) > 1:
    datasets_dir = sys.argv[1]
    output_dir = os.path.join(datasets_dir, "merged")
    merge(datasets_dir, output_dir)


