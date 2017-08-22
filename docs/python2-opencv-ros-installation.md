# Python2 OpenCV Installation 

**25 Jule 2017** 
sudo apt-get install python-opencv

   `asekerci@tony-utc ~/ebug-swarm/nrf-bridge/notes : sudo apt-get install python-opencv
    Reading package lists... Done
  Building dependency tree       
  Reading state information... Done
    The following additional packages will be installed:
    libaec0 libarmadillo6 libarpack2 libdap17v5 libdapclient6v5 libepsilon1
    libfreexl1 libgdal1i libgeos-3.5.0 libgeos-c1v5 libgtkglext1 libhdf4-0-alt
    libhdf5-10 libhdf5-openmpi-10 libhwloc-plugins libhwloc5 libibverbs1
    libkmlbase1 libkmldom1 libkmlengine1 libminizip1 libnetcdf-c++4 libnetcdf11
    libogdi3.2 libopencv-calib3d3.0 libopencv-core-dev libopencv-core3.0
    libopencv-features2d3.0 libopencv-flann3.0 libopencv-highgui3.0
    libopencv-imgcodecs3.0 libopencv-imgproc-dev libopencv-imgproc3.0
    libopencv-ml3.0 libopencv-objdetect3.0 libopencv-photo3.0 libopencv-shape3.0
    libopencv-stitching3.0 libopencv-superres3.0 libopencv-video-dev
    libopencv-video3.0 libopencv-videoio3.0 libopencv-videostab3.0
    libopencv-viz3.0 libopenmpi1.10 libproj9 libspatialite7 libsuperlu4 libsz2
    liburiparser1 libvtk6.2 openmpi-bin openmpi-common proj-bin proj-data
  Suggested packages:
    libhdf4-doc libhdf4-alt-dev hdf4-tools libnetcdf4 libhwloc-contrib-plugins
    ogdi-bin mpi-default-bin vtk6-doc vtk6-examples openmpi-checkpoint
  The following NEW packages will be installed:
    libaec0 libarmadillo6 libarpack2 libdap17v5 libdapclient6v5 libepsilon1
    libfreexl1 libgdal1i libgeos-3.5.0 libgeos-c1v5 libgtkglext1 libhdf4-0-alt
    libhdf5-10 libhdf5-openmpi-10 libhwloc-plugins libhwloc5 libibverbs1
    libkmlbase1 libkmldom1 libkmlengine1 libminizip1 libnetcdf-c++4 libnetcdf11
    libogdi3.2 libopencv-calib3d3.0 libopencv-core-dev libopencv-core3.0
    libopencv-features2d3.0 libopencv-flann3.0 libopencv-highgui3.0
    libopencv-imgcodecs3.0 libopencv-imgproc-dev libopencv-imgproc3.0
    libopencv-ml3.0 libopencv-objdetect3.0 libopencv-photo3.0 libopencv-shape3.0
    libopencv-stitching3.0 libopencv-superres3.0 libopencv-video-dev
    libopencv-video3.0 libopencv-videoio3.0 libopencv-videostab3.0
    libopencv-viz3.0 libopenmpi1.10 libproj9 libspatialite7 libsuperlu4 libsz2
    liburiparser1 libvtk6.2 openmpi-bin openmpi-common proj-bin proj-data
    python-opencv
  0 upgraded, 56 newly installed, 0 to remove and 0 not upgraded.
  Need to get 0 B/58.0 MB of archives.
  After this operation, 254 MB of additional disk space will be used.
`
# ROS Installation

We have Ubuntu 16.04.2 LTS (tony-pc). Installed: `http://wiki.ros.org/kinetic/Installation/Ubuntu`

We continue with ROS tutorials as suggested. 

I am here:
http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem

# Fixing NRF Code 
## Let's start with assign_addresses()


  asekerci@tony-utc ~/ebug-swarm/nrf-bridge-py3/my_archive : ipython
  Python 2.7.12 (default, Nov 19 2016, 06:48:10)
  Type "copyright", "credits" or "license" for more information.

  IPython 2.4.1 -- An enhanced Interactive Python.
  ?         -> Introduction and overview of IPython's features.
  %quickref -> Quick reference.
  help      -> Python's own help system.
  object?   -> Details about 'object', use 'object??' for extra details.

  In [1]: from nrf import bridge

  In [2]: nrf = bridge()

  In [3]: nrf.assign_addresses()
  Out[3]: {1: (15, 19, 25, 605, 25, 68)}

  In [4]: 

New one is using xrange... Tony uses range.






