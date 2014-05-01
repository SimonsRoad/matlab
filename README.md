Dense RGBD Tracker
==================

This is an implementation of a dense tracker, to compute the pose (rotation and translation) of the camera between successive frames.

This implementation uses 2 image and a depth image (for a Kinect for instance) for the first image.


Credits
=======
This work is a somewhat naive implementation inspired and derived from these work:

[Real-Time Visual Odometry from Dense RGB-D Images](https://vision.in.tum.de/_media/spezial/bib/steinbruecker_sturm_cremers_iccv11.pdf)
[Robust Odometry Estimation for RGB-D Cameras](http://vision.in.tum.de/_media/spezial/bib/kerl13icra.pdf)

Results
=======

Inputs:

Image1 :
![alt tag](https://raw.githubusercontent.com/mtourne/matlab/master/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.211214.png)

Image1 Depth:
![alt tag](https://raw.githubusercontent.com/mtourne/matlab/master/data/rgbd_dataset_freiburg1_xyz/depth/1305031102.226738.png)

Image2:
![alt tag](https://raw.githubusercontent.com/mtourne/matlab/master/data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.275326.png)

Outputs :

Camera Relative pose between images :
```
guessed_pose =

    0.9999    0.0024    0.0122   -0.0171
   -0.0021    0.9997   -0.0241    0.0877
   -0.0122    0.0240    0.9996   -1.8645
         0         0         0    1.0000
```

Residual Image :
![alt tag](https://raw.githubusercontent.com/mtourne/matlab/master/dens_tracking_residual.jpg)

Useful reads
============
[Lucas-Kanade 20 Years On: A Unifying Framework](http://www.cs.cmu.edu/afs/cs/academic/class/15385-s12/www/lec_slides/Baker\x26Matthews.pdf)

[TechRep: A tutorial on SE(3) transformation parameterizations and on-manifold optimization](http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf)
