Author: Mason Hawver

# Compiling & Running it

```
mkdir build
cmake -B build
cd build
make
./aaa_main
```

# Gallery

![render0](assets/promo/sponzarender0.png) 

# Todo

- [X] texture loading
- [X] model loading / compression
- [X] Debug shapes
- [X] Multiple Objects
- [ ] Scene files
- [ ] Build Octrees
- [ ] Render Octrees
- [ ] Build RRT
- [ ] Metropolis Hastings Trajectory sampeling
- [ ] Frame Buffer
- [ ] Add Documentation

# Data Pipeline

| Proc | Lang |
|------|------|
|.obj -> .bin  | c++  (Done) |
|.bin -> .scene    | python (Todo) |
|.scene -> .octree | python (Todo) |
|.octree -> .rrt   | python (Todo -> if theres not a work around) |
|.bin .octree .rrt -> data | c++ |
|data -> cnn | python (Big Todo) |
|cnn -> inferance | c++ (Big Todo) |

