import h5py
import sys

def print_h5_structure(name, obj):
    indent = '  ' * name.count('/')
    if isinstance(obj, h5py.Group):
        print(f"{indent} Group: {name}")
    elif isinstance(obj, h5py.Dataset):
        print(f"{indent} Dataset: {name} | shape: {obj.shape}, dtype: {obj.dtype}")
        if obj.attrs:
            print(f"{indent}  ↳ Attributes:")
            for key, val in obj.attrs.items():
                print(f"{indent}    - {key}: {val}")
    else:
        print(f"{indent} Unknown object: {name}")

def inspect_h5_file(file_path):
    try:
        with h5py.File(file_path, 'r') as f:
            print(f"Inspecting file: {file_path}")
            f.visititems(print_h5_structure)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python inspect_h5.py <file_path.h5>")
    else:
        inspect_h5_file(sys.argv[1])
