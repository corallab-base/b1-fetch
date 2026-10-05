Unitree's realsense-ros 2.3.2 + "Add D430i support" (3760e280), the driver
for the B1's D430i cameras, kept in the repo so every board builds the same
code. Only realsense2_camera: realsense2_description (70 MB of camera meshes,
only used by an RViz example launch) is left out. Change from upstream:
CMakeLists.txt requires librealsense exactly 2.50.0 (see the comment there).
