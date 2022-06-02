CLASSES = ('ceiling', 'floor', 'wall', 'beam', 'column', 'window', 'door', 'chair', 'table',
               'bookcase', 'sofa', 'board', 'clutter')
THRESH = 0.1

import argparse
import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='Convert softgroup output to msk format')
parser.add_argument('input', help='Input softgroup file')
input_path = parser.parse_args().input
output_path = input_path.replace('.txt', '.msk')

class Instance:
    def __init__(self, line):
        [self.path, label, confidence] = line.split()
        self.label = CLASSES[int(label) - 1]
        self.confidence = float(confidence)

    def __lt__(self, other):
        return self.confidence < other.confidence

    def to_string(self, color):
        hex = ''.join(['%02x' % int(x * 255) for x in color])
        return f'{self.path} {hex} {self.label}'


with open(input_path, 'r') as f:
    lines = f.readlines()
    
instances = [Instance(line) for line in lines]
instances = sorted([i for i in instances if i.confidence > THRESH])[::-1]

# jet color map
cmap = plt.get_cmap('jet')

hex_colors = [cmap(i)[:3] for i in np.linspace(0, 1, len(instances))]

with open(output_path, 'w') as f:
    for i, hex in zip(instances, hex_colors):
        f.write(i.to_string(hex) + '\n')
