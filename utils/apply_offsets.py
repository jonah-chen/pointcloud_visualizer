
import argparse
import numpy as np

parser = argparse.ArgumentParser(
    description='Convert a text file in the same format as the S3DIS dataset to a ply file.')
parser.add_argument("input", type=str, help='Input text file.')
parser.add_argument("shift", type=str, help='Shift numpy file.')

args = parser.parse_args()
input_path = args.input
shifts_path = args.shift
output_path = input_path.replace('.txt', '_shifted.txt')

shifts = np.load(shifts_path) * -1
with open(input_path, 'r') as f:
    data = f.readlines()
with open(output_path, 'w') as f:
    for line, shift in zip(data, shifts):
        line = line.split()
        f.write(f"{float(line[0])-shift[0]:.6f} {float(line[1])-shift[1]:.6f} {float(line[2])-shift[2]:.6f} {int(line[3].split('.')[0])} {int(line[4].split('.')[0])} {int(line[5].split('.')[0])}\n")
