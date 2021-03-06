#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Copyright (C) 2018 Sovrasov V. - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 * You should have received a copy of the MIT license with
 * this file. If not visit https://opensource.org/licenses/MIT
'''
import argparse
import os
import math
import json
import time

from utils import load_problem
from transportation_solver import solve_transportation, branch_strategies

def main(args):
    problems = []
    for file in sorted(os.listdir(args.problems_dir)):
        if file.endswith('.txt'):
            problems.append(load_problem(os.path.join(args.problems_dir, file)))

    results = []
    t_values = []
    for problem in problems:
        print('-'*100)
        print('Problem: ' + problem.name)
        start = time.time()
        value, order, iters = solve_transportation(problem, args.branch_strategy)
        end = time.time()
        t_values.append(1. - float(iters) / math.factorial(problem.n))
        print('Found solution: {}'.format(order))
        print('Criterion value: {}'.format(value))
        print('T={}'.format(t_values[-1]))
        print('Iterations performed {}'.format(iters))
        print('Time elapsed {}'.format(end - start))

        results.append({'problem_name': problem.name,
                        'opt_value': value,
                        'solution': order,
                        'iterations': iters,
                        'T': t_values[-1]})

        assert value == problem.opt_value

    t_avg = sum(t_values) / len(t_values)
    print('T_avg={}'.format(t_avg))
    if args.output_log:
        with open(args.output_log, 'w') as f:
            json.dump(results, f)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Transportation problems solver')
    parser.add_argument('--problems_dir', type=str, default='./problems')
    parser.add_argument('--branch_strategy', type=str, choices=branch_strategies, default='breadth-first')
    parser.add_argument('--output_log', type=str, default='')

    main(parser.parse_args())
