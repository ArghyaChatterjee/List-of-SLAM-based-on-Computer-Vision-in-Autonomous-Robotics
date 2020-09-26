# SLAM-Algorithms-in-Autonomous-Robotics
It's a repository where I keep updated about SLAM algorithms for state estimation based on sensor fusion in the Field of Robotics and Autonomous Navigation.
## SLAM Algorithms for Drones and Ground Rovers based on Computer Vision:
### Monocular camera:
1. SVO+MSF
2. MSCKF
3. ROVIO
4. OKVIS
5. VINS-Mono
6. SVO+GTSAM
7. [ORB SLAM](https://github.com/raulmur/ORB_SLAM)
8. [ORB SLAM 2](https://github.com/raulmur/ORB_SLAM2)
9. [ORB SLAM 3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
10. Tiny SLAM
11. Xivo 
12. SLAM Toolbox
13. LAMA

### Stereo & RGB-D camera:
1. Gmapping (2D SLAM)
2. Hector SLAM (2D SLAM)
3. Cartographer (2D & 3D SLAM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/cartographer-project/cartographer)> <[ROS Support](https://github.com/cartographer-project/cartographer_ros)>
4. RTAB MAP (3D SLAM)
5. KIMERA (3D SLAM)
6. TEASER ++ (3D SLAM)
7. [ORB SLAM (3D SLAM)](https://github.com/raulmur/ORB_SLAM)
8. [ORB SLAM 2 (3D SLAM)](https://github.com/raulmur/ORB_SLAM2)
9. [ORB SLAM 3 (3D SLAM)](https://github.com/UZ-SLAMLab/ORB_SLAM3)
10. ROVIO (3D SLAM)

### LIDAR based SLAM:
1. Gmapping (2D SLAM) <[paper](https://www.researchgate.net/publication/257523133)> <[code](https://github.com/OctoMap/octomap)> <[ROS Support](https://wiki.ros.org/octomap)>
2. Hector SLAM (2D SLAM) <[paper](https://www.researchgate.net/publication/257523133)> <[code](https://github.com/OctoMap/octomap)> <[ROS Support](https://wiki.ros.org/octomap)>
3. Cartographer (2D & 3D SLAM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/cartographer-project/cartographer)> <[ROS Support](https://github.com/cartographer-project/cartographer_ros)>
4. Lidar Odometry And Mapping (LOAM, 3D SLAM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/cartographer-project/cartographer)> <[ROS Support](https://github.com/laboshinl/loam_velodyne)>
5. Light Weight and Ground Optimized Lidar Odometry and Mapping (LeGO-LOAM, 3D SLAM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)>
6. Intensity SLAM Context based Lidar Odometry and Mapping (ISC LOAM, 3D SLAM) <[paper](https://arxiv.org/pdf/2003.05656.pdf)> <[code](https://github.com/wh200720041/iscloam)>
7. SLAM Context based LeGo LOAM (SC LeGO-LOAM, 3D SLAM) <[paper](https://arxiv.org/pdf/2003.05656.pdf)> <[code](https://github.com/irapkaist/SC-LeGO-LOAM)>
8. Lidar Inertial Odometry and Mapping (LIO-Mapping, 3D SLAM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/cartographer-project/cartographer)> <[ROS Support](https://github.com/hyye/lio-mapping)>
9. Lidar Inertial Odometry Smoothing and Mapping (LIO-SAM. 3D SLAM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/TixiaoShan/LIO-SAM)>
10. Advanced-Lidar Odometry And Mapping (A-LOAM, 3D SLAM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/HKUST-Aerial-Robotics/A-LOAM)>
11. Faster Lidar Odometry and Mapping (F-LOAM, 3D SLAM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/wh200720041/floam)>
12. Modular System for Localization and Mapping (MOLA, 3D SLAM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/wh200720041/floam)> <[Tutorial](https://docs.mola-slam.org/latest/)>
13. Berkeley Localization and Mapping (BLAM, 3D SALM) <[paper](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf)> <[code](https://github.com/luhongquan66/BLAM)>

### Mapping Framework:
1. Octomapping (3D Mapping) <[paper](https://www.researchgate.net/publication/257523133)> <[code](https://github.com/OctoMap/octomap)> <[ROS Support](https://wiki.ros.org/octomap)>
2. VoxBlox (3D Mapping) <[paper](http://helenol.github.io/publications/iros_2017_voxblox.pdf)> <[code](https://github.com/ethz-asl/voxblox)> <[ROS Support](https://voxblox.readthedocs.io/en/latest/pages/Installation.html)>
3. VoxGraph (3D Mapping) <[paper](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/385682/Voxgraph-ETHpreprintversion.pdf)> <[code](https://github.com/ethz-asl/voxgraph)> <[ROS Support](https://github.com/ethz-asl/voxgraph)>
4. MRFMapping (3D Mapping) <[paper](https://arxiv.org/pdf/2006.03512.pdf)> <[code](https://github.com/mrfmap/mrfmap)> <[ROS Support](https://github.com/mrfmap/mrfmap_ros)>
3. PointClouds (3D Mapping)
4. Elevation Maps (3D Mapping)
5. Multi-level Surface Maps (3D Mapping) 
6. Gmapping (2D Mapping)
7. Hector Mapping (2D Mapping)

## Tracking Algorithms in Visual Inertial Navigation for SLAM in OpenCV:
### [Pre-Tracker](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_table_of_contents_feature2d/py_table_of_contents_feature2d.html)
1. [Harris Corner Detection](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_features_harris/py_features_harris.html#harris-corners)
2. [Shi-Tomasi Corner Detector & Good Features to Track](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_shi_tomasi/py_shi_tomasi.html#shi-tomasi)
3. [SIFT (Scale-Invariant Feature Transform)](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_sift_intro/py_sift_intro.html#sift-intro)
4. [SURF (Speeded-Up Robust Features)](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_surf_intro/py_surf_intro.html#surf)
5. [FAST Algorithm for Corner Detection](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_fast/py_fast.html#fast)
6. [BRIEF (Binary Robust Independent Elementary Features)](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_brief/py_brief.html#brief)
7. [ORB (Oriented FAST and Rotated BRIEF)](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_orb/py_orb.html#orb)

### Post-Tracker
1. [Feature Matching (Brute-Force matcher and FLANN based matcher)](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_matcher/py_matcher.html#matcher)
2. [Feature Matching + Homography to find Objects](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html#py-feature-homography)
