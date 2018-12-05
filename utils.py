def list_to_ints(lst):
    return [int(x) for x in lst]

def load_problem(path):
    with open(path) as f:
        problem = {}
        data = [line.strip() for line in f.readlines()]
        assert data
        problem['n'] = int(data[0])
        assert len(data) == problem['n'] + 5
        problem['limits'] = list_to_ints(data[1].split())
        problem['delays_matrix'] = []
        for i in range(problem['n'] + 1):
            problem['delays_matrix'].append(list_to_ints(data[2 + i].split()))
        problem['opt_solution'] = list_to_ints(data[problem['n'] + 3].split())
        problem['opt_value'] = int(data[-1])

        return problem

    return None
