import queue
import math
import abc

class branch_strategy:
    def __init__(self):
        pass

    @abc.abstractmethod
    def put(self, v):
        pass

    @abc.abstractmethod
    def get(self):
        pass

    @abc.abstractmethod
    def empty(self):
        pass

class breadth_first(branch_strategy):
    def __init__(self):
        self.storage = queue.Queue()

    def put(self, v):
        self.storage.put(v)

    def get(self):
        return self.storage.get()

    def empty(self):
        return self.storage.empty()

class depth_first(branch_strategy):
    def __init__(self):
        self.storage = queue.LifoQueue()

    def put(self, v):
        self.storage.put(v)

    def get(self):
        return self.storage.get()

    def empty(self):
        return self.storage.empty()

class optimistic(branch_strategy):
    def __init__(self):
        self.storage = queue.PriorityQueue()

    def empty(self):
        return self.storage.empty()

    def get(self):
        key, v = self.storage.get()
        return v

    def put(self, v):
        self.storage.put((v[1], v))

class realistic(branch_strategy):
    def __init__(self):
        self.storage = queue.PriorityQueue()

    def empty(self):
        return self.storage.empty()

    def get(self):
        key, v = self.storage.get()
        return v

    def put(self, v):
        self.storage.put((v[2], v))

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
        vertexes = breadth_first()
    elif branch_strategy == 'depth-first':
        vertexes = depth_first()
    elif branch_strategy == 'optim':
        vertexes = optimistic()
    elif branch_strategy == 'real':
        vertexes = realistic()

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
