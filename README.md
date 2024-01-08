# SEG-MAT: 3D Shape Segmentation Using Medial Axis Transform

This repository contains the source code for the TVCG 2020 paper [SEG-MAT: 3D Shape Segmentation Using Medial Axis Transform](https://arxiv.org/abs/2010.11488). In this work, we present an efficient method to automatically segment an arbitary object into meaningful parts based on medial axis transform (MAT). Please check out the [project page](https://enigma-li.github.io/projects/segmat/SEG-MAT.html) for more detail.


<a>
    <img src="doc/demo.png" width="90% height="90%"/>
</a>


## Executable
Download the executable program [SEG-MAT-x64](https://clinplayer.github.io/resources/SEG-MAT-Release.zip) with sample data or you can find it from the `exe program` folder.

A simple demo:
```
SEG-MAT -m data\teddy.off -b data\teddy_mat.ma -s data\teddy_smat.ma
```

Usage:
```
Usage: SEG-MAT [-h,--help] -m,--mesh -b,--bmat -s,--smat [-g,--grow] [-n,--min] [-p,--prim]

Arguments:
    -h, --help    show this help message and exit
    -m, --mesh    path to the surface mesh (.off)
    -b, --bmat    path to the base MAT (.ma)
    -s, --smat    path to the structure MAT (.ma)
    -g, --grow    (optional) growing threshold, default 0.015
    -n, --min     (optional) minimal region, default 0.002
    -p, --prim    (optional) whether to compute the primitive representation (0:no, 1:yes)
```




## Code
### Installation
Use the CMakelist.txt to compile the project.

Third part libraries： 
  - [CGAL](https://www.cgal.org/) versions 4.9, 4.11 are tested. Boost 1.59.0 is needed to compile CGAL. Please refer to the [link](https://doc.cgal.org/latest/Manual/installation.html) here.
  - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) version 3.3 is already included in the source files.

Example compile process on Windows:
  - Use CMake UI or commond tool to build the project
  - Specify the CGAL library path
  - Make it to a specific IDE project, e.g., Visual Studio, XCode, etc.

<a>
    <img src="doc/cmake_example.png" width="100% height="100%"/>
</a>


### Data
Download the computed MAT (*.ma format) of Princeton Segmentation Benchmark [PSB_MAT.zip](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chlin_connect_hku_hk/Edu-XmRNhsRMvqZmOwEduYYB0KgToomB4FakkZV3jf0TnA?e=HoDcFq) and a subset of ShapeNet [ShapeNet_MAT.zip](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chlin_connect_hku_hk/EaoKYVbKcshMi7kRUhkL2cQBo-XUl1wOLCKpojMMuomBkQ?e=zMsdUe).

### MAT Computation (for your own data)
Given a 3D mesh, we use [Q-MAT](https://binwangthss.github.io/qmat/qmat.html)) to compute the base MAT and the structure MAT for segmentation (Download Q-MAT [exe](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chlin_connect_hku_hk/EQ7qXlADSq5IuseCeCqMPncBbwOoDqRYrTgKEKolrfOUpA?e=KTcaLm)  [code](https://connecthkuhk-my.sharepoint.com/:u:/g/personal/chlin_connect_hku_hk/ETyMY_FyJeRJvw9SlJQiNIoBGq7KL8z50UAI-W3NYHqgvA?e=B1Gnnq)). The Q-MAT simplification is based on vertex number, while we recommend the users to set the vertex number of the base MAT to 2000, and the vertex number of the structure MAT to max{0.001\*initial_MAT_vertex_number, 15}.

For a non-watertight or a poor-quality mesh, you may need to first convert the mesh to a watertight one using the [tool](https://github.com/hjwdzh/Manifold) for robust MAT computation.  


## Citation
If you find this work useful, please consider citing:
```
@article{lin2022seg,
  title={SEG-MAT: 3D Shape Segmentation Using Medial Axis Transform},
  author={Lin, Cheng and Liu, Lingjie and Li, Changjian and Kobbelt, Leif and Wang, Bin and Xin, Shiqing and Wang, Wenping},
  journal={IEEE transactions on visualization and computer graphics},
  volume={28},
  number={6},
  pages={2430--2444},
  year={2022}
}
```

## Acknowlegement

We would like to acknowledge the following projects:

[Multi-label optimization](https://vision.cs.uwaterloo.ca/code/)

[Implementation of the Earth Movers Distance (EMD)](http://robotics.stanford.edu/~rubner/emd/default.htm)

[Oriented minimal bounding box](https://sarielhp.org/p/00/diameter/diam_prog.html)


## Contact
If you have any questions, please email [Cheng Lin](https://clinplayer.github.io/) at chlin@hku.hk or [Changjian Li](https://enigma-li.github.io/) at chjili2011@gmail.com.

