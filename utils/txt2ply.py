import argparse

parser = argparse.ArgumentParser(
    description='Convert a text file in the same format as the S3DIS dataset to a ply file.')
parser.add_argument("input", type=str, help='Input text file.')

input_path = parser.parse_args().input
output_path = input_path.replace('.txt', '.ply')
with open(input_path, 'r') as f:
    data = f.readlines()

with open(output_path, 'w') as f:
    f.write("ply\n")
    f.write("format ascii 1.0\n")
    f.write("element vertex " + str(len(data)) + "\n")
    f.write("property float x\n")
    f.write("property float y\n")
    f.write("property float z\n")
    f.write("property uchar red\n")
    f.write("property uchar green\n")
    f.write("property uchar blue\n")
    f.write("end_header\n")
    for line in data:
        line = line.split()
        f.write(f"{line[0]} {line[1]} {line[2]} {int(line[3].split('.')[0])} {int(line[4].split('.')[0])} {int(line[5].split('.')[0])}\n")