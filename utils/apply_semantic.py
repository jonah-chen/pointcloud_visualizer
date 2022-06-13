import argparse
import numpy as np
import matplotlib.pyplot as plt


parser = argparse.ArgumentParser(
    description='Convert a text file in the same format as the S3DIS dataset to a ply file.')
parser.add_argument("input", type=str, help='Input text file.')
parser.add_argument("semantic", type=str, help='Semantic predictions numpy file.')
parser.add_argument("classes", type=int, default=0, help='Number of classes.')

args = parser.parse_args()
input_path = args.input
semantic_path = args.semantic
output_path = input_path.replace('.txt', '_semantic.txt')

semantic = np.load(semantic_path)

num_classes = np.max(semantic) + 1 if args.classes == 0 else args.classes

jet = plt.get_cmap('jet')
colors = [jet(i)[:3] for i in np.linspace(0, 1, num_classes)]

with open(input_path, 'r') as f:
    data = f.readlines()
with open(output_path, 'w') as f:
    for line, cls in zip(data, semantic):
        line = line.split()
        f.write(f"{line[0]} {line[1]} {line[2]} {int(255*colors[cls][0])} {int(255*colors[cls][1])} {int(255*colors[cls][2])}\n")
