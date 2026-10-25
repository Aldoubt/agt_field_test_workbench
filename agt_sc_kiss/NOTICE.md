# Noncommercial research component

Scan Context: https://github.com/gisbi-kim/scancontext
commit `93672835bb85e9c03fedf0fbaf2d0491df12161e`, copyright KAIST and NAVER LABS,
authors Giseop Kim and Ayoung Kim. Cite **Scan Context: Egocentric Spatial
Descriptor for Place Recognition Within 3D Point Cloud Map**, IROS 2018.

Upstream code and this adapted component are distributed under
**CC BY-NC-SA 4.0**, https://creativecommons.org/licenses/by-nc-sa/4.0/ .
This component is for noncommercial research. Attribution and ShareAlike apply.
Upstream nanoflann/KD-tree files retain their BSD notices; tictoc.h retains
Tong Qin and Shaozu Cao's attribution. Sources are fetched by immutable lock;
CMake verifies all five source checksums before compilation.

Changes: remove unused ROS1/OpenCV includes, use 0 m height offset for body
coordinates and a shared 30 m radius. The descriptor, ring/sector keys, distance,
and yaw alignment are upstream implementations. A thin static-map Top-K query
uses upstream KD-tree and distance functions, without the temporal recent-frame
exclusion intended for online loop closure. The query is never inserted in the DB.

KISS-Matcher `8b58b118e00c7a5a22448741f8d956d638a44ae2`: MIT-SPARK, MIT license;
ROBIN and other transitive versions remain in `dependencies/kiss_python_build.json`.
small_gicp `2c6c7831bc1f5ada2ee5300027fe7d7aee66827d`: Kenji Koide, MIT license.
Neither registration algorithm is copied or reimplemented here.
