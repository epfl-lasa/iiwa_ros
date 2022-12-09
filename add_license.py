#!/usr/bin/env python
# encoding: utf-8
#|
#|    Copyright (C) 2019-2022 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
#|              Matthias Mayr
#|              Bernardo Fichera
#|    email:    costashatz@gmail.com
#|              matthias.mayr@cs.lth.se
#|              bernardo.fichera@epfl.ch
#|    Other contributors:
#|              Yoan Mollard (yoan@aubrune.eu)
#|              Walid Amanhoud (walid.amanhoud@epfl.ch)
#|    website:  lasa.epfl.ch
#|
#|    This file is part of iiwa_ros.
#|
#|    iiwa_ros is free software: you can redistribute it and/or modify
#|    it under the terms of the GNU General Public License as published by
#|    the Free Software Foundation, either version 3 of the License, or
#|    (at your option) any later version.
#|
#|    iiwa_ros is distributed in the hope that it will be useful,
#|    but WITHOUT ANY WARRANTY; without even the implied warranty of
#|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#|    GNU General Public License for more details.
#|
import fnmatch,re
import os, shutil, sys

license = '''Copyright (C) 2019-2022 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
Authors:  Konstantinos Chatzilygeroudis (maintainer)
          Matthias Mayr
          Bernardo Fichera
email:    costashatz@gmail.com
          matthias.mayr@cs.lth.se
          bernardo.fichera@epfl.ch
Other contributors:
          Yoan Mollard (yoan@aubrune.eu)
          Walid Amanhoud (walid.amanhoud@epfl.ch)
website:  lasa.epfl.ch

This file is part of iiwa_ros.

iiwa_ros is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

iiwa_ros is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
'''

def make_dirlist(folder, extensions):
    matches = []
    for root, dirnames, filenames in os.walk(folder):
        for ext in extensions:
            for filename in fnmatch.filter(filenames, '*' + ext):
                matches.append(os.path.join(root, filename))
    return matches

def insert_header(fname, prefix, postfix, license, kept_header = []):
    input = open(fname, 'r')
    ofname = '/tmp/' + fname.split('/')[-1]
    output = open(ofname, 'w')
    for line in kept_header:
        output.write(line + '\n')
    output.write(prefix + '\n')
    has_postfix = len(postfix)>0
    my_prefix = prefix
    if has_postfix:
        my_prefix = ''
    for line in license.split('\n'):
        if len(line)>0:
            output.write(my_prefix + '    ' + line + '\n')
        else:
            output.write(my_prefix + '\n')
    if has_postfix:
        output.write(postfix + '\n')
    in_header = False
    for line in input:
        header = len(list(filter(lambda x: x == line[0:len(x)], kept_header))) != 0
        check_prefix = (line[0:len(prefix)] == prefix)
        check_postfix = (has_postfix and (line[0:len(postfix)] == postfix))
        if check_prefix and has_postfix:
            in_header = True
        if check_postfix:
            in_header = False
        if (not in_header) and (not check_prefix) and (not header) and (not check_postfix):
            output.write(line)
    output.close()
    shutil.move(ofname, fname)

def insert(directory):
    # cpp
    cpp =  make_dirlist(directory, ['.hpp', '.cpp', '.h', '.c', '.cc'])
    for i in cpp:
        insert_header(i, '//|', '', license)
    # py
    py = make_dirlist(directory, ['.py'])
    for i in py:
        insert_header(i, '#|', '', license, ['#!/usr/bin/env python', '# encoding: utf-8'])
    # CMake
    cmake = make_dirlist(directory, ['CMakeLists.txt'])
    for i in cmake:
        # metapackages should not have any comments
        if i.endswith('iiwa_ros/CMakeLists.txt'):
            continue
        insert_header(i, '#|', '', license)
    # # XML/URDF
    xml_urdf = make_dirlist(directory, ['.xml', '.urdf', '.xacro', '.launch'])
    for i in xml_urdf:
        header = ['<?xml version="1.0"?>']
        insert_header(i, '<!--|', '|-->', license, header)

if __name__ == '__main__':
    insert('.')