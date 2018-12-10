import queue
import math
import abc

FLT_INF = float('inf')

class node:
    def __init__(self, v=[], upper=FLT_INF, lower=FLT_INF, partial_objective=FLT_INF, partial_time=0):
        self.v = v
        self.upper = upper
        self.lower = lower
        self.partial_objective = partial_objective
        self.partial_time = partial_time

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

def update_partial_opjective(vertex, problem):
    assert vertex.v

    t = len(vertex.v) - 1
    if t == 0:
        time = problem['delays_matrix'][0][vertex.v[0]]
        objective = 0
        if time > problem['limits'][vertex.v[0] - 1]:
            objective += 1
        vertex.partial_objective = objective
        vertex.partial_time = time
    else:
        vertex.partial_time += problem['delays_matrix'][vertex.v[t - 1]][vertex.v[t]]
        if vertex.partial_time > problem['limits'][vertex.v[t] - 1]:
            vertex.partial_objective += 1

    return vertex

def compute_lower_bound_parallel(vertex, problem):
    if len(vertex.v) == problem['n']:
        return vertex.partial_obective

    not_visited = list(set(range(1, problem['n'] + 1)) - set(vertex.v))
    time = vertex.partial_time
    objective = vertex.partial_objective

    for i in not_visited:
        if time + problem['delays_matrix'][vertex.v[-1]][i] > problem['limits'][i - 1]:
            objective += 1

    return objective

def compute_upper_bound_greedy(vertex, problem):
    if len(vertex.v) == problem['n']:
        return vertex.partial_obective

    greedy_v = node(list(vertex.v), partial_objective=vertex.partial_objective, partial_time=vertex.partial_time)

    while len(greedy_v.v) < problem['n']:
        not_visited = list(set(range(1, problem['n'] + 1)) - set(greedy_v.v))
        w = []
        z = greedy_v.partial_time
        for beta in not_visited:
            t_d = problem['limits'][beta - 1]
            t = z + problem['delays_matrix'][greedy_v.v[-1]][beta]
            if t > t_d:
                w.append(float('inf'))
            else:
                w.append(t_d - t)
        best_idx = w.index(min(w))
        greedy_v.v.append(not_visited[best_idx])
        greedy_v = update_partial_opjective(greedy_v, problem)

    return greedy_v.v, greedy_v.partial_objective

def branch(vertex, n):
    assert len(vertex.v) < n
    new_nodes = []

    not_visited = list(set(range(1, n + 1)) - set(vertex.v))
    for i in not_visited:
        new_nodes.append(node(vertex.v + [i], vertex.upper, vertex.lower, vertex.partial_objective, vertex.partial_time))

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
            new_nodes = branch(vertex, n)
            for new_vertex in new_nodes:
                new_vertex = update_partial_opjective(new_vertex, problem)
                new_vertex.lower = compute_lower_bound(new_vertex, problem)
                if len(new_vertex.v) < n:
                    greedy_solution, new_vertex.upper = compute_upper_bound(new_vertex, problem)
                    assert len(greedy_solution) == n
                    if new_vertex.upper < best_point.lower:
                        best_point = node(greedy_solution, new_vertex.upper, new_vertex.upper)
                else:
                    new_vertex.upper = new_vertex.lower
                best_upper_bound = min(best_upper_bound, new_vertex.upper)
                if not (new_vertex.lower >= best_upper_bound):
                    nodes.put(new_vertex)

    return best_point.lower, best_point.v, iters
