import argparse
import os
import queue
import math
import json

from utils import load_problem

def compute_partial_objective(v, problem):
    assert v

    objective = 0
    time = problem['delays_matrix'][0][v[0]]
    if time > problem['limits'][v[0] - 1]:
        objective += 1
    for i in range(1, len(v)):
        time += problem['delays_matrix'][v[i - 1]][v[i]]
        if time > problem['limits'][v[i] - 1]:
            objective += 1

    return time, objective

def compute_objective(v, problem):
    assert len(v) == problem['n']
    return compute_partial_objective(v, problem)[1]

def compute_lower_bound_parallel(v, problem):
    if len(v) == problem['n']:
        return compute_objective(v, problem)

    time, objective = compute_partial_objective(v, problem)
    not_visited = [i for i in range(1, problem['n'] + 1) if not i in v]
    for i in not_visited:
        if time + problem['delays_matrix'][v[-1]][i] > problem['limits'][i - 1]:
            objective += 1

    return objective

def compute_upper_bound_greedy(v, problem):
    if len(v) == problem['n']:
        return compute_objective(v, problem)

    v_ = list(v)

    while len(v_) < problem['n']:
        not_visited = [i for i in range(1, problem['n'] + 1) if not i in v_]
        w = []
        z, _ = compute_partial_objective(v_, problem)
        for beta in not_visited:
            t_d = problem['limits'][beta - 1]
            t = z + problem['delays_matrix'][v[-1]][beta]
            if t > t_d:
                w.append(float('inf'))
            else:
                w.append(t_d - t)
        best_idx = w.index(min(w))
        v_.append(not_visited[best_idx])

    return v_, compute_objective(v_, problem)

def branch(v, n):
    assert len(v) <= n
    new_nodes = []

    for i in range(1, n + 1):
        if i not in v:
            new_nodes.append(v + [i])

    return new_nodes

branch_strategies = ['breadth-first', 'depth-first', 'optim', 'real']

def solve_transportation(problem, branch_strategy='breadth-first',
                         compute_lower_bound=compute_lower_bound_parallel,
                         compute_upper_bound=compute_upper_bound_greedy):
    if branch_strategy == 'breadth-first':
        vertexes = queue.Queue()
    elif branch_strategy == 'depth-first':
        vertexes = queue.LifoQueue()

    n = problem['n']
    vertexes.put(([], n, n))
    best_point = ([], n, n)
    best_upper_bound = n
    iters = 0

    while not vertexes.empty():
        iters += 1
        v = vertexes.get()

        if len(v[0]) == n:
            assert v[1] == v[2]
            if v[1] < best_point[1]:
                best_point = v
        else:
            new_v = branch(v[0], n)
            for v in new_v:
                lower = compute_lower_bound(v, problem)
                if len(v) < n:
                    v_, upper = compute_upper_bound(v, problem)
                    assert len(v_) == n
                    if upper < best_point[1]:
                        best_point = (v_, upper, upper)
                else:
                    upper = lower
                best_upper_bound = min(best_upper_bound, upper)
                if not (lower >= best_upper_bound):
                    vertexes.put((v, lower, upper))

    return best_point[1], best_point[0], iters

def main(args):
    problems = []
    for file in sorted(os.listdir(args.problems_dir)):
        if file.endswith('.txt'):
            problems.append(load_problem(os.path.join(args.problems_dir, file)))

    results = []
    t_values = []
    for problem in problems:
        print('-'*100)
        print('Problem: ' + problem['name'])
        value, order, iters = solve_transportation(problem, args.branch_strategy)
        t_values.append(1. - float(iters) / math.factorial(problem['n']))
        print('Found solution: {}'.format(order))
        print('Criterion value: {}'.format(value))
        print('T={}'.format(t_values[-1]))

        results.append({'problem_name': problem['name'],
                        'opt_value': value,
                        'solution': order,
                        'T': t_values[-1]})

        assert value == problem['opt_value']

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
    parser.add_argument('--verbose', action='store_true', help='Print additional info to console')

    main(parser.parse_args())
