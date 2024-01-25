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
import sys
import subprocess

def read_file(fname):
    with open(fname) as f:
        return f.readlines()
    return []


def remove_initial_comments(data):
    i = 0
    for k in range(len(data)):
        if data[k][0] == '#':
            i = k
            break
    return data[i:]

def remove_ifdefs(data, label):
    to_del = []

    depth = 0
    for k in range(len(data)):
        line = data[k]
        if line.startswith("#ifndef " + label) or line.startswith("#ifdef " + label):
            depth = depth + 1
        if depth > 0:
            to_del.append(k)
            if line.startswith("#endif"):
                depth = depth - 1
    
    cdata = data
    for i in range(len(to_del)):
        del cdata[to_del[i]-i]
    return cdata

def struct_names():
    return ['Params', 'Vars', 'Workspace', 'Settings']

def function_names():
    return ['ldl_solve', 'ldl_factor', 'check_factorization', 'matrix_multiply', 'check_residual', 'fill_KKT', 'multbymA', 'multbymAT', 'multbymG', 'multbymGT', 'multbyP', 'fillq', 'fillh', 'fillb', 'pre_ops', 'eval_gap', 'set_defaults', 'setup_pointers', 'setup_indexed_params', 'setup_indexing', 'set_start', 'eval_objv', 'fillrhs_aff', 'fillrhs_cc', 'refine', 'calc_ineq_resid_squared', 'calc_eq_resid_squared', 'better_start', 'fillrhs_start', 'solve', 'printmatrix', 'unif', 'ran1', 'randn_internal', 'randn', 'reset_rand']
    # return ['ldl_solve', 'ldl_factor', 'check_factorization', 'matrix_multiply', 'check_residual', 'fill_KKT', 'multbymA', 'multbymAT', 'multbymG', 'multbymGT', 'multbyP', 'fillq', 'fillh', 'fillb', 'pre_ops', 'eval_gap', 'set_defaults', 'setup_pointers', 'setup_indexed_params', 'setup_indexing', 'set_start', 'eval_objv', 'fillrhs_aff', 'fillrhs_cc', 'refine', 'calc_ineq_resid_squared', 'calc_eq_resid_squared', 'better_start', 'fillrhs_start', 'solve', 'tic', 'toc', 'tocq', 'printmatrix', 'unif', 'ran1', 'randn_internal', 'randn', 'reset_rand']

def get_function(data, fname):
    func_name = fname + "("
    ret_data = []
    depth = 0

    for l in data:
        line = l.strip()
        if line.startswith('void ' + func_name) or line.startswith('long ' + func_name) or line.startswith('double ' + func_name) or line.startswith('int ' + func_name) or line.startswith('float ' + func_name):
            depth = depth + 1
        if depth > 0:
            ret_data.append(l)
            if line.endswith('{') and (func_name not in line):
                depth = depth + 1
            if line.startswith('}') or line.endswith('}'):
                depth = depth - 1
    
    return ret_data

def get_struct(data, sname):
    struct_name = 'typedef struct ' + sname + "_t"
    ret_data = []
    depth = 0

    for line in data:
        if depth > 0:
            if line.startswith('}'):
                depth = depth - 1
            else:
                ret_data.append(line)
        
        if line.startswith(struct_name):
            depth = depth + 1
    
    return ret_data

if __name__== "__main__":
    arg_num = len(sys.argv[1:])

    prefix = 'cvxgen'
    directory = "./"
    if arg_num >=1:
        directory = sys.argv[1] + "/"
    if arg_num >=2:
        prefix = sys.argv[2]
    output = prefix + ".hpp"
    if arg_num >=3:
        output = sys.argv[3]
    
    filenames = ['solver.h', 'solver.c', 'ldl.c', 'util.c', 'matrix_support.c']
    filenames = [directory + f for f in filenames]

    sources = []
    for f in filenames:
        sources.append(remove_initial_comments(read_file(f)))
    
    structs = []
    for sname in struct_names():
        structs.append(get_struct(sources[0], sname))
    
    functions = []

    for func_name in function_names():
        for source in sources[1:]:
            d = get_function(source, func_name)
            if len(d)>0:
                functions.append(d)
                break

    tab = "    "
    out_file = "#include <math.h>\n";
    out_file += "#include <stdio.h>\n";
    out_file += "#include <time.h>\n\n";

    out_file += "#define IA 16807\n";
    out_file += "#define IM 2147483647\n"
    out_file += "#define AM (1.0/IM)\n"
    out_file += "#define IQ 127773\n"
    out_file += "#define IR 2836\n"
    out_file += "#define NTAB 32\n"
    out_file += "#define NDIV (1+(IM-1)/NTAB)\n"
    out_file += "#define EPS 1.2e-7\n"
    out_file += "#define RNMX (1.0-EPS)\n\n"

    out_file += "namespace " + prefix + " {\n\n"

    for i in range(len(structs)):
        name = struct_names()[i]
        out_file += tab + "struct " + name + " {\n"
        for l in structs[i]:
            out_file += tab + l
        out_file += tab + "};\n\n"
    
    out_file += tab + "class Solver {\n" + tab + "public:\n";

    # out_file += "\n" + tab + "protected:\n";
    
    for i in range(len(structs)):
        name = struct_names()[i]
        lower = name.lower()
        if name == 'Workspace':
            lower = 'work'
        out_file += tab + tab + name + " " + lower + ";\n"
    
    out_file += "\n" + tab + tab + "long global_seed = 1;\n";
    # out_file += tab + tab + "clock_t tic_timestart;\n\n"
    out_file += "\n"

    for f in functions:
        for l in f:
            out_file += tab + tab + l
        out_file += "\n"
    
    out_file += tab + "};\n}"

    text_file = open(directory + output, "w")
    text_file.write(out_file)
    text_file.close()

    subprocess.call("clang-format -i " + directory + output, shell=True)

    # for arg in sys.argv[1:]:
    #     print arg