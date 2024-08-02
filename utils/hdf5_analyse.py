import h5py

def print_hdf5_structure_2(file_name):
    f = h5py.File(file_name, "r")  # mode = {'w', 'r', 'a'}

    # Print the keys of groups and datasets under '/'.
    print(f.filename, ":")
    print([key for key in f.keys()], "\n")

    d = f["observations"]
    print(d.name, ":")

    print([key for key in d.keys()], "\n")

    dd = d['images']
    print(dd.keys())
    # print(dd["top"][0])
    print(dd.name, ":")

    dd = d['qpos']
    print(dd.name, ":")
    print(dd[5], "\n")

    dd = d['qvel']
    print(dd.name, ":")
    print(dd[5], "\n")

    print(f["/action"][54])

    f.close()

def print_hdf5_structure(file_name):
    def print_structure(name, obj):
        print(name)
        if isinstance(obj, h5py.Group):
            for key, value in obj.attrs.items():
                print(f"  Attribute: {key} => {value}")
        elif isinstance(obj, h5py.Dataset):
            print(f"  Shape: {obj.shape}")
            print(f"  Data type: {obj.dtype}")
            for key, value in obj.attrs.items():
                print(f"  Attribute: {key} => {value}")

    with h5py.File(file_name, 'r') as file:
        file.visititems(print_structure)

# 使用示例
file_name = '/home/zme/Downloads/episode_50.hdf5'
print_hdf5_structure(file_name)
# print_hdf5_structure_2(file_name)
