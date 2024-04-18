import transforms3d as tf
import numpy as np
import struct

import sys

class Scene:
    def __init__(self):
        self.objects = []
    
    def add_object(self, bin_name, position, quat, scale):
        self.objects.append({
            "fn" : bin_name,
            "pos" : position,
            "quat" : quat,
            "scale" : scale,
        })

    def write(self, fn):
        with open(fn, 'wb') as f:
            f.write(struct.pack('i', len(self.objects)))
            for obj in self.objects:
                bts = (obj["fn"] + "\0").encode(encoding="ascii")[:128].ljust(128)
                # bts[min(len(obj["fn"]), 127)] = '\0'
                f.write(bts)
                for v in obj["pos"] + obj["quat"] + obj["scale"]:
                    # print(v, struct.pack('f', v))
                    f.write(struct.pack('f', v))
    
    def read(self, fn):
        self.objects = []
        with open(fn, 'rb') as f:
            num_objects = struct.unpack('i', f.read(4))[0]
            for _ in range(num_objects):
                bin_name = f.read(128).decode(encoding="ascii").split('\0', 1)[0]
                position = []
                for _ in range(3):
                    position.append(struct.unpack('f', f.read(4))[0])
                quat = []
                for _ in range(4):
                    quat.append(struct.unpack('f', f.read(4))[0])
                scale = []
                for _ in range(3):
                    scale.append(struct.unpack('f', f.read(4))[0])
                self.add_object(bin_name, position, quat, scale)


def test_scene():
    scn = Scene()
    scn.add_object("models/sponza/sponza.bin", [0, 0, 0], list(tf.euler.euler2quat(np.pi / 2, 0, 0, 'sxyz')), [.01, .01, .01])
    scn.add_object("models/bunny/bunny.bin", [-0.05, 2.0, 0], list(tf.euler.euler2quat(np.pi / 2, 0, 0, 'sxyz')), [1, 1, 1])
    return scn

# {-0.05, 0, -2}, {0, 1, 0}, 0, {1, 1, 1}

scenes = {
    "test" : test_scene
}

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage python3 scene.py <scene name : test> <output file name>")

    scn = scenes[sys.argv[1]]()
    scn.write(sys.argv[2])
