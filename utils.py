import os

def list_to_ints(lst):
    return [int(x) for x in lst]

class TransportationProblem:
    def __init__(self):
        self.name = None
        self.limits = None
        self.n = None
        self.delays_matrix = None
        self.opt_solution = None
        self.opt_value = None
        self.feasible_coordinates = None

def load_problem(path):
    with open(path) as f:
        problem = TransportationProblem()
        problem.name = os.path.splitext(os.path.basename(path))[0]
        data = [line.strip() for line in f.readlines()]
        assert data
        problem.n = int(data[0])
        assert len(data) == problem.n + 5
        problem.limits = list_to_ints(data[1].split())
        problem.delays_matrix = []
        for i in range(problem.n + 1):
            problem.delays_matrix.append(list_to_ints(data[2 + i].split()))
        problem.opt_solution = list_to_ints(data[problem.n + 3].split())
        problem.opt_value = int(data[-1])
        problem.feasible_coordinates = set(range(1, problem.n + 1))

        return problem

    return None
