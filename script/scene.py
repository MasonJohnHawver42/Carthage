import transforms3d as tf
import struct

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

def test_scene():
    scn = Scene()
    scn.add_object("models/sponza/sponza.bin", [0, 0, 0], list(tf.euler.euler2quat(0, 0, 0, 'sxyz')), [.01, .01, .01])
    scn.add_object("models/bunny/bunny.bin", [-0.05, 0, -2.0], list(tf.euler.euler2quat(0, 0, 0, 'sxyz')), [1, 1, 1])
    return scn

# {-0.05, 0, -2}, {0, 1, 0}, 0, {1, 1, 1}

scn = test_scene()
scn.write("data/scenes/test.scn")