import operator
import argparse
import json
import csv
import os
from tabulate import tabulate

def main(args):
    all_stats = {}
    for subdir, dirs, files in sorted(os.walk(args.root_folder)):
        print('Prosessing directory ' + subdir)
        for file in files:
            _, ext = os.path.splitext(file)
            if ext == '.json':
                full_path = os.path.join(subdir, file)
                stats = {}
                with open(full_path, 'r') as f:
                    stats = json.load(f)
                    method_name = os.path.basename(file).split('.')[0]
                    all_stats[method_name] = stats

    table = {}
    columns = ['Problem']
    for capture in sorted(all_stats.keys()):
        columns.append(capture)

    rows = [item['problem_name'] for item in all_stats[columns[1]]]
    rows = {el:[] for el in rows}
    avg_t = {el:0 for el in columns[1:]}

    for capture in columns[1:]:
        for item in all_stats[capture]:
            rows[item['problem_name']].append(str(round(item['T'], 11)) + '(' + str(item['iterations']) + ')')
            avg_t[capture] += item['T']
        avg_t[capture] /= len(all_stats[capture])

    table_rows = []
    for problem_name in sorted(rows.keys()):
        table_rows.append([problem_name.split('_')[-1]] + rows[problem_name])

    last_row = [r'$T_{avg}$']
    for capture in columns[1:]:
        last_row.append(str(round(avg_t[capture], 11)))
    table_rows.append(last_row)

    table = tabulate(table_rows, headers=columns, tablefmt='latex', numalign='center')
    print(table)
    print('The best method is ' + max(avg_t.items(), key=operator.itemgetter(1))[0])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('root_folder', type=str)
    main(parser.parse_args())
