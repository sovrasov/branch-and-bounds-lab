import argparse
import os
import queue

from utils import load_problem


def compute_objective(v, problem):
    assert len(v) == problem['n']

    objective = 0
    time = problem['delays_matrix'][0][v[0]]
    if time > problem['limits'][v[0] - 1]:
        objective += 1
    for i in range(1, problem['n']):
        time += problem['delays_matrix'][v[i - 1]][v[i]]
        if time > problem['limits'][v[i] - 1]:
            objective += 1

    return objective

def compute_lower_bound(v, problem):
    if len(v) == problem['n']:
        return compute_objective(v, problem)
    return float('inf')

def compute_upper_bound(v, problem):
    if len(v) == problem['n']:
        return compute_objective(v, problem)
    return float('inf')

def branch(v, n):
    assert len(v) <= n
    new_nodes = []

    for i in range(1, n + 1):
        if i not in v:
            new_nodes.append(v + [i])

    return new_nodes

def solve_transportation(problem):
    vertexes = queue.Queue()
    vertexes.put(([], float('-inf'), float('-inf')))
    best_point = ([], float('inf'), float('inf'))
    n = problem['n']

    while not vertexes.empty():
        v = vertexes.get()

        if len(v[0]) == n:
            obj = compute_objective(v[0], problem)
            if obj < best_point[1]:
                best_point = (v[0], obj, obj)
        else:
            new_v = branch(v[0], n)
            for v in new_v:
                lower = compute_lower_bound(v, problem)
                upper = compute_upper_bound(v, problem)
                if not (lower > best_point[2]):
                    vertexes.put((v, lower, upper))

    return best_point[1], best_point[0]

def main(args):
    problems = []
    for file in os.listdir(args.problems_dir):
        if file.endswith('.txt'):
            problems.append(load_problem(os.path.join(args.problems_dir, file)))

    for problem in problems:
        value, order = solve_transportation(problem)
        print('Problem: ' + problem['name'])
        print('Found solution: {}'.format(order))
        print('Criterion value: {}'.format(value))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Transportation problems solver')
    parser.add_argument('--problems_dir', type=str, default='./problems')
    parser.add_argument('--branch_strategy', type=str, choices=['breadth-first', 'depth-fist'], default='breadth-first')
    parser.add_argument('--verbose', action='store_true', help='Print additional info to console')

    main(parser.parse_args())
