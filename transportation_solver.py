import queue
import math
import abc

FLT_INF = float('inf')

class node:
    def __init__(self, v=[], upper=FLT_INF, lower=FLT_INF, partial_obective=FLT_INF):
        self.v = v
        self.upper = upper
        self.lower = lower
        self.partial_objective = partial_obective

    def __lt__(self, other):
        return self.v < other.v

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
        self.storage.put((v.lower, v))

class realistic(branch_strategy):
    def __init__(self):
        self.storage = queue.PriorityQueue()

    def empty(self):
        return self.storage.empty()

    def get(self):
        key, v = self.storage.get()
        return v

    def put(self, v):
        self.storage.put((v.upper, v))

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
            t = z + problem['delays_matrix'][v_[-1]][beta]
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
        nodes = breadth_first()
    elif branch_strategy == 'depth-first':
        nodes = depth_first()
    elif branch_strategy == 'optim':
        nodes = optimistic()
    elif branch_strategy == 'real':
        nodes = realistic()

    n = problem['n']
    nodes.put(node([], n, n))
    best_point = node([], n, n)
    best_upper_bound = n
    iters = 0

    while not nodes.empty():
        iters += 1
        vertex = nodes.get()

        if len(vertex.v) == n:
            assert vertex.lower == vertex.upper
            if vertex.lower < best_point.lower:
                best_point = vertex
        else:
            new_v = branch(vertex.v, n)
            for v in new_v:
                lower = compute_lower_bound(v, problem)
                if len(v) < n:
                    v_, upper = compute_upper_bound(v, problem)
                    assert len(v_) == n
                    if upper < best_point.lower:
                        best_point = node(v_, upper, upper)
                else:
                    upper = lower
                best_upper_bound = min(best_upper_bound, upper)
                if not (lower >= best_upper_bound):
                    nodes.put(node(v, lower, upper))

    return best_point.lower, best_point.v, iters
