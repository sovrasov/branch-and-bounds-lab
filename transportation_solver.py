import argparse
import os

from utils import load_problem

def main(args):
    problems = []
    for file in os.listdir(args.problems_dir):
        if file.endswith('.txt'):
            problems.append(load_problem(os.path.join(args.problems_dir, file)))

    for problem in problems:
        print(problem)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Transportation problems solver')
    parser.add_argument('--problems_dir', type=str, default='./problems')
    parser.add_argument('--branch_strategy', type=str, choices=['breadth-first', 'depth-fist'], default='breadth-first')
    parser.add_argument('--verbose', action='store_true', help='Print additional info to console')

    main(parser.parse_args())
