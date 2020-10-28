# SEG-MAT: 3D Shape Segmentation Using Medial Axis Transform

This repository contains the source code for the TVCG 2020 paper [SEG-MAT: 3D Shape Segmentation Using Medial Axis Transform](https://arxiv.org/abs/2010.11488). In this work, we present an efficient method to automatically segment an arbitary object into meaningful parts based on medial axis transform (MAT). 


<a>
    <img src="figure/demo.png" width="90% height="90%"/>
</a>


## Executable Tool
Download the executable file with sample data [SEG-MAT-exe-x64](https://clinplayer.github.io/resources/SEG-MAT-Release.zip).

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
You need to install [CGAL](https://www.cgal.org/), [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) and [Boost](https://www.boost.org/). Then edit the `.props` files in `config\` by changing the paths to yours.

This code was tested under Windows 10, Visual Studio 2015 with CGAL 4.9, Eigen 3.3.4 and Boost.1.59.0. 

### Data
Download the computed MAT (*.ma format) of Princeton Segmentation Benchmark [PSB_MAT.zip](https://drive.google.com/file/d/1hLa-by6f_v8Hbw5qln_TAORQn1tGzA2d/view?usp=sharing) and a subset of ShapeNet [ShapeNet_MAT.zip](https://drive.google.com/file/d/1GGZEE2w4YcXfQH8DLvc7lZrOiSP7oCYi/view?usp=sharing).

### MAT Computation (for your own data)
Given a 3D mesh, we use [Q-MAT](http://cgcad.thss.tsinghua.edu.cn/wangbin/qmat/qmat.html) to compute the base MAT and the structure MAT for segmentation. The Q-MAT simplification is based on vertex number, while we recommend the users to set the vertex number of the base MAT to 2000, and the vertex number of the structure MAT to max{0.001\*initial_MAT_vertex_number, 15}.

For a non-watertight or a poor-quality mesh, you may need to first convert the mesh to a watertight one using the [tool](https://github.com/hjwdzh/Manifold) for robust MAT computation.  


## Citation
If you find this work useful, please consider citing:
```
@article{SEG-MAT-2020,
   title={SEG-MAT: 3D Shape Segmentation Using Medial Axis Transform},
   ISSN={2160-9306},
   url={http://dx.doi.org/10.1109/TVCG.2020.3032566},
   DOI={10.1109/tvcg.2020.3032566},
   journal={IEEE Transactions on Visualization and Computer Graphics},
   publisher={Institute of Electrical and Electronics Engineers (IEEE)},
   author={Lin, Cheng and Liu, Lingjie and Li, Changjian and Kobbelt, Leif and Wang, Bin and Xin, Shiqing and Wang, Wenping},
   year={2020},
   pages={1–1}
}
```

## Acknowlegement

We would like to acknowledge the following open sources:

[Multi-label optimization](https://vision.cs.uwaterloo.ca/code/)

[Implementation of the Earth Movers Distance (EMD)](http://robotics.stanford.edu/~rubner/emd/default.htm)

[Oriented minimal bounding box](https://sarielhp.org/p/00/diameter/diam_prog.html)


## Contact
If you have any questions, please email [Cheng Lin](https://clinplayer.github.io/) at chlin@hku.hk.

