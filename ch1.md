# Introduction

- inverse models
  - Seek to recover unknowns given insufficient information to fully specify the solution
  - Resort to physics based models, probabilistic models, or ML
- forward models??
- Mostly probability models in this book
- Well-known algorithms too
  - This could be useful for route detection
  - Robust to noise, reasonably efficient
- OpenCV is BGR and not RGB
  - [source](https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html#ga397ae87e1288a81d2363b61574eb8cab)

### 1990s image processing innovations

- projective reconstructions
  - Computation of a structure of a scene from images taken with
    uncalibrated cameras
      - An advancement
- factorization techniques
  - orthographic camera approximations applicable
    - Objects farther from the camera appear the same size regardless of distance
    - Used to simplify 3D reconstruction
    - Don't use in depth sensitive applicatons
  - Pose estimation
    - Answers: "Where is it, and how is it oriented?"
      - Eg. Where head, arms, and legs are on a person | How body is rotated | Where
        standing, sitting, or walking
      - 2D and 3D
- Bundle Adjustment
  - Used in photogrammetry
    - Obtaining reliable info about physical objects and environment via
      process of recording, measuring, and interpreting photographic images
        - Out of date for GIS stuff now?
  - Refine estimates of 3D structure and camera poses
  - Minimizing reprojection error across multiple values
- Graph Cut Techniques
  - Image smoothing, stereo correspondence problem, image segmentation 
  - Formulated in terms of energy minimization
    - From computational chemistry field
    - Solution approximated by using solving a maximum flow problem in a graph
- Active contours
  - Example: snakes, particle filters, and level sets
  - Also have: intensity-based (direct) techniques
  - Snake description
    - Energy minimization
    - Matches deformable model to an image by means of energy minimization
    - Used in edge detection
    - Is there something better than this?
  - For tracking
  - Might be able to extract the contours of the climbing holds and then cluster those
- Image Segmentation
  - Minimum energy, miniumum description length, normalized cuts, and mean shift
  - Minimum description length leclerc paper
    - SLIC
    - RAG
    - scikit-image has this stuff if I want to use it
  - Normalized Cuts
    - From [here](https://michaelkarpe.github.io/computer-vision-projects/segmentation/)
    - Efficient
    - Graph representation of the image
  - Mean Shift
    - From [here](https://michaelkarpe.github.io/computer-vision-projects/segmentation/)
    - Find local maxima in high-dimensional data distribution *without* computing the later
    - Uses multiple restart gradient descent with a kernel function
      - Common kernels: Gaussian kernel and Epanechnikov kernel
      - [gaussian kernel](https://www.geeksforgeeks.org/machine-learning/gaussian-kernel/)
      - [many kernels include Epanechnikov](https://en.wikipedia.org/wiki/Kernel_%28statistics%29)
- Statistical Learning Techniques
  - eigenface analysis for facial recognition
  - linear dynamical systems for curve tracking
    - Dynamical systems definition [wikipedia](https://en.wikipedia.org/wiki/Dynamical_system)
      - A function defines the time dependence of a point in ambient space
      - Lorenz attractor from the Lorenz oscillator
      - Swinging of a clock pendulum, flow of water in a pipe
- Computer Graphics Increase
  - cross-disciplinary area of image-based modeling and rendering
  - image morphing techniques for manipulating real-world images into animations

### 2000s image processing innovations

- Introduced:
  - image stitching
  - light-field capture and rendering
  - High Dynamic Range (HDR) image capture via exposure bracketing
    - New tone mapping algorithms
      - Process of mapping one set of colors to another to approximate appearance of HDR images
        in a medium that has a more limited dynamic range
      - Basically just fake an HDR image
- Texture Synthesis, quilting, and inpainting
  - Classified as computational photography techniques
- Emergence of feature-based techniques for object recognition
  - Constellation Models
    - Generative model for category-level object recognition
  - Pictoral structures
- Development of more efficient global optimization problems
  - Loopy belief propagation
- Most notible: Sophisticated ML techniques

### 2010s image processing innovations

- Trend of large labeled datasets exploded
- AlexNet first NN to win yearly ImageNet large-scale visual recognition challenge
- Optical Flow
- Deniosing
- Monocular Depth Inference

## 1.3 Book Overview

- Get some sample applications
- Sets of exercises
