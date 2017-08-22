16 June 2017 Ahmet
------------------
Using:
https://launchpad.net/~orangain/+archive/ubuntu/opencv

asekerci@tony-utc 14:18 /usr/local/lib  : sudo apt-get install python3-opencv
Reading package lists... Done
Building dependency tree       
Reading state information... Done
The following additional packages will be installed:
  libaec0 libarmadillo6 libarpack2 libdap17v5 libdapclient6v5 libepsilon1 libfreexl1 libgdal1i libgeos-3.5.0 libgeos-c1v5
  libhdf4-0-alt libhdf5-10 libhwloc-plugins libhwloc5 libibverbs1 libkmlbase1 libkmldom1 libkmlengine1 libminizip1 libnetcdf-c++4
  libnetcdf11 libogdi3.2 libopencv-calib3d3.0 libopencv-core-dev libopencv-core3.0 libopencv-features2d3.0 libopencv-flann3.0
  libopencv-highgui3.0 libopencv-imgcodecs3.0 libopencv-imgproc-dev libopencv-imgproc3.0 libopencv-ml3.0 libopencv-objdetect3.0
  libopencv-photo3.0 libopencv-shape3.0 libopencv-stitching3.0 libopencv-superres3.0 libopencv-video-dev libopencv-video3.0
  libopencv-videoio3.0 libopencv-videostab3.0 libopencv-viz3.0 libopenmpi1.10 libpq5 libproj9 libspatialite7 libsuperlu4 libsz2
  liburiparser1 libvtk6.2 odbcinst odbcinst1debian2 openmpi-bin openmpi-common proj-bin proj-data
Suggested packages:
  libhdf4-doc libhdf4-alt-dev hdf4-tools libnetcdf4 libhwloc-contrib-plugins ogdi-bin mpi-default-bin vtk6-doc vtk6-examples
  openmpi-checkpoint
The following NEW packages will be installed:
  libaec0 libarmadillo6 libarpack2 libdap17v5 libdapclient6v5 libepsilon1 libfreexl1 libgdal1i libgeos-3.5.0 libgeos-c1v5
  libhdf4-0-alt libhdf5-10 libhwloc-plugins libhwloc5 libibverbs1 libkmlbase1 libkmldom1 libkmlengine1 libminizip1 libnetcdf-c++4
  libnetcdf11 libogdi3.2 libopencv-calib3d3.0 libopencv-core-dev libopencv-core3.0 libopencv-features2d3.0 libopencv-flann3.0
  libopencv-highgui3.0 libopencv-imgcodecs3.0 libopencv-imgproc-dev libopencv-imgproc3.0 libopencv-ml3.0 libopencv-objdetect3.0
  libopencv-photo3.0 libopencv-shape3.0 libopencv-stitching3.0 libopencv-superres3.0 libopencv-video-dev libopencv-video3.0
  libopencv-videoio3.0 libopencv-videostab3.0 libopencv-viz3.0 libopenmpi1.10 libpq5 libproj9 libspatialite7 libsuperlu4 libsz2
  liburiparser1 libvtk6.2 odbcinst odbcinst1debian2 openmpi-bin openmpi-common proj-bin proj-data python3-opencv
0 upgraded, 57 newly installed, 0 to remove and 5 not upgraded.
Need to get 0 B/56.9 MB of archives.
After this operation, 251 MB of additional disk space will be used.

--------
asekerci@tony-utc 15:40 /usr/local/lib  : dpkg --get-selections | grep opencv | grep 3
libopencv-calib3d3.0:amd64                      install
libopencv-core3.0:amd64                         install
libopencv-features2d3.0:amd64                   install
libopencv-flann3.0:amd64                        install
libopencv-highgui3.0:amd64                      install
libopencv-imgcodecs3.0:amd64                    install
libopencv-imgproc3.0:amd64                      install
libopencv-ml3.0:amd64                           install
libopencv-objdetect3.0:amd64                    install
libopencv-photo3.0:amd64                        install
libopencv-shape3.0:amd64                        install
libopencv-stitching3.0:amd64                    install
libopencv-superres3.0:amd64                     install
libopencv-video3.0:amd64                        install
libopencv-videoio3.0:amd64                      install
libopencv-videostab3.0:amd64                    install
libopencv-viz3.0:amd64                          install
python3-opencv                                  install
----------
Unfortunately it didn't work.
I will back out now:
 asekerci@tony-utc 15:49 ~/ebug-swarm/nrf-bridge  : sudo apt-get remove python3-opencv
[sudo] password for asekerci:
Reading package lists... Done
Building dependency tree       
Reading state information... Done
The following packages were automatically installed and are no longer required:
  libaec0 libarmadillo6 libarpack2 libdap17v5 libdapclient6v5 libepsilon1 libfreexl1 libgdal1i libgeos-3.5.0 libgeos-c1v5
  libhdf4-0-alt libhdf5-10 libhwloc-plugins libhwloc5 libibverbs1 libkmlbase1 libkmldom1 libkmlengine1 libminizip1 libnetcdf-c++4
  libnetcdf11 libogdi3.2 libopencv-calib3d3.0 libopencv-core-dev libopencv-core3.0 libopencv-features2d3.0 libopencv-flann3.0
  libopencv-highgui3.0 libopencv-imgcodecs3.0 libopencv-imgproc-dev libopencv-imgproc3.0 libopencv-ml3.0 libopencv-objdetect3.0
  libopencv-photo3.0 libopencv-shape3.0 libopencv-stitching3.0 libopencv-superres3.0 libopencv-video-dev libopencv-video3.0
  libopencv-videoio3.0 libopencv-videostab3.0 libopencv-viz3.0 libopenmpi1.10 libpq5 libproj9 libspatialite7 libsuperlu4 libsz2
  liburiparser1 libvtk6.2 odbcinst odbcinst1debian2 openmpi-bin openmpi-common proj-bin proj-data
Use 'sudo apt autoremove' to remove them.
The following packages will be REMOVED:
  python3-opencv
0 upgraded, 0 newly installed, 1 to remove and 5 not upgraded.
After this operation, 11.3 MB disk space will be freed.
Do you want to continue? [Y/n] y
-----------
Hmmm... the libopencv libraries are still there but they are "amd64" libraries???
asekerci@tony-utc 15:41 /usr/local/lib  : dpkg --get-selections | grep opencv | grep 3
libopencv-calib3d3.0:amd64                      install
libopencv-core3.0:amd64                         install
libopencv-features2d3.0:amd64                   install
libopencv-flann3.0:amd64                        install
libopencv-highgui3.0:amd64                      install
libopencv-imgcodecs3.0:amd64                    install
libopencv-imgproc3.0:amd64                      install
libopencv-ml3.0:amd64                           install
libopencv-objdetect3.0:amd64                    install
libopencv-photo3.0:amd64                        install
libopencv-shape3.0:amd64                        install
libopencv-stitching3.0:amd64                    install
libopencv-superres3.0:amd64                     install
libopencv-video3.0:amd64                        install
libopencv-videoio3.0:amd64                      install
libopencv-videostab3.0:amd64                    install
libopencv-viz3.0:amd64                          install
------------
our machine is:
asekerci@tony-utc 15:51 /usr/local/lib  : arch
x86_64

There is no difference: they are different names for the same
thing. Actually, it was AMD themselves who started switching the name
from AMD64 to x86_64... Now x86_64 is the "generic" name for AMD64 and
EM64T (Extended Memory 64-bit Technology) as Intel named its
implementation.
-------------
Uninstalled opencv_python-3.2.0.6 
asekerci@tony-utc 16:21 ~  : sudo uninstall opencv-python
The directory '/home/asekerci/.cache/pip/http' or its parent directory is not owned by the current user and the cache has been disabled. Please check the permissions and owner of that directory. If executing pip with sudo, you may want sudo's -H flag.
Uninstalling opencv-python-3.2.0.6:
  /usr/local/lib/python3.5/dist-packages/cv2/__init__.py
  /usr/local/lib/python3.5/dist-packages/cv2/__pycache__/__init__.cpython-35.pyc
  /usr/local/lib/python3.5/dist-packages/cv2/cv2.cpython-35m-x86_64-linux-gnu.so
  /usr/local/lib/python3.5/dist-packages/opencv_python-3.2.0.6.dist-info/DESCRIPTION.rst
  /usr/local/lib/python3.5/dist-packages/opencv_python-3.2.0.6.dist-info/INSTALLER
  /usr/local/lib/python3.5/dist-packages/opencv_python-3.2.0.6.dist-info/METADATA
  /usr/local/lib/python3.5/dist-packages/opencv_python-3.2.0.6.dist-info/RECORD
  /usr/local/lib/python3.5/dist-packages/opencv_python-3.2.0.6.dist-info/WHEEL
  /usr/local/lib/python3.5/dist-packages/opencv_python-3.2.0.6.dist-info/metadata.json
  /usr/local/lib/python3.5/dist-packages/opencv_python-3.2.0.6.dist-info/top_level.txt
Proceed (y/n)? y
  Successfully uninstalled opencv-python-3.2.0.6
-----------
I still see import cv2 successful:
 asekerci@tony-utc 16:22 ~/ebug-swarm/nrf-bridge  : ipython3
Python 3.5.2 (default, Nov 17 2016, 17:05:23)
Type "copyright", "credits" or "license" for more information.

IPython 5.3.0 -- An enhanced Interactive Python.
?         -> Introduction and overview of IPython's features.
%quickref -> Quick reference.
help      -> Python's own help system.
object?   -> Details about 'object', use 'object??' for extra details.

In [1]: import cv2
-----------
uninstalled python3.5 (minimal python3.5 is still there):
asekerci@tony-utc 16:58 ~  : dpkg --get-selections | grep python3

libpython3-stdlib:amd64                         install
libpython3.5:amd64                              install
libpython3.5-minimal:amd64                      install
libpython3.5-stdlib:amd64                       install
python3                                         deinstall
python3-aptdaemon.pkcompat                      deinstall
python3-cupshelpers                             deinstall
python3-minimal                                 install
python3.5-minimal                               install

-----------
uninstalled opencv 2.45 stuff..
sudo apt remove libopencv-core2.4v5
-----------
Still cv2 ... this is python's own stuff.

asekerci@tony-utc 16:50 /usr/local/lib/python3.5/dist-packages  : python3
Python 3.5.2 (default, Nov 17 2016, 17:05:23)
[GCC 5.4.0 20160609] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import cv2
>>> cv2.__
cv2.__UMAT_USAGE_FLAGS_32BIT  cv2.__format__(               cv2.__lt__(                   cv2.__setattr__(
cv2.__class__(                cv2.__ge__(                   cv2.__name__                  cv2.__sizeof__(
cv2.__delattr__(              cv2.__getattribute__(         cv2.__ne__(                   cv2.__spec__
cv2.__dict__                  cv2.__gt__(                   cv2.__new__(                  cv2.__str__(
cv2.__dir__(                  cv2.__hash__(                 cv2.__package__               cv2.__subclasshook__(
cv2.__doc__                   cv2.__init__(                 cv2.__reduce__(               cv2.__version__
cv2.__eq__(                   cv2.__le__(                   cv2.__reduce_ex__(
cv2.__file__                  cv2.__loader__                cv2.__repr__(
>>> cv2.__file__
'/usr/local/lib/python3.5/dist-packages/cv2.cpython-35m-x86_64-linux-gnu.so'
------------
17 June 2017 I continue installations. Reinstalling the python3 stuff:
sudo apt-get install python3
sudo apt-get install python3-doc python3-tk python3-venv python3.5-venv python3.5-doc
sudo apt-get remove python-pip

sudo easy_install pip
Searching for pip
Reading https://pypi.python.org/simple/pip/
Best match: pip 9.0.1
Downloading https://pypi.python.org/packages/11/b6/abcb525026a4be042b486df43905d6893fb04f05aac21c32c638e939e447/pip-9.0.1.tar.gz#md5=35f01da33009719497f01a4ba69d63c9
Processing pip-9.0.1.tar.gz
Writing /tmp/easy_install-XKyZoa/pip-9.0.1/setup.cfg
Running pip-9.0.1/setup.py -q bdist_egg --dist-dir /tmp/easy_install-XKyZoa/pip-9.0.1/egg-dist-tmp-3Wb0pO
/usr/lib/python2.7/distutils/dist.py:267: UserWarning: Unknown distribution option: 'python_requires'
  warnings.warn(msg)
warning: no previously-included files found matching '.coveragerc'
warning: no previously-included files found matching '.mailmap'
warning: no previously-included files found matching '.travis.yml'
warning: no previously-included files found matching '.landscape.yml'
warning: no previously-included files found matching 'pip/_vendor/Makefile'
warning: no previously-included files found matching 'tox.ini'
warning: no previously-included files found matching 'dev-requirements.txt'
warning: no previously-included files found matching 'appveyor.yml'
no previously-included directories found matching '.github'
no previously-included directories found matching '.travis'
no previously-included directories found matching 'docs/_build'
no previously-included directories found matching 'contrib'
no previously-included directories found matching 'tasks'
no previously-included directories found matching 'tests'
creating /usr/local/lib/python2.7/dist-packages/pip-9.0.1-py2.7.egg
Extracting pip-9.0.1-py2.7.egg to /usr/local/lib/python2.7/dist-packages
Adding pip 9.0.1 to easy-install.pth file
Installing pip script to /usr/local/bin
Installing pip2.7 script to /usr/local/bin
Installing pip2 script to /usr/local/bin

Installed /usr/local/lib/python2.7/dist-packages/pip-9.0.1-py2.7.egg
Processing dependencies for pip
Finished processing dependencies for pip

Note: there is a sytem pip : sudo -H pip list
pip is the pip in my local path.
-------------------
asekerci@tony-utc 13:46 /usr/local/bin  : sudo -H pip3 list
DEPRECATION: The default format will switch to columns in the future. You can use --format=(legacy|columns) (or define a format=(legacy|columns) in your pip.conf under the [list] section) to disable this warning.
decorator (4.0.11)
ipdb (0.10.2)
ipython (5.3.0)
ipython-genutils (0.2.0)
numpy (1.12.1)
pbr (2.0.0)
pickleshare (0.7.4)
pip (9.0.1)
prompt-toolkit (1.0.14)
Pygments (2.2.0)
pyserial (3.3)
scikit-learn (0.18.1)
scipy (0.19.0)
simplegeneric (0.8.1)
sklearn (0.0)
stevedore (1.21.0)
traitlets (4.3.2)
virtualenv (15.1.0)
virtualenv-clone (0.2.6)
virtualenvwrapper (4.7.2)
wcwidth (0.1.7)
websockets (3.2)

no cv library here.... so this file comes from an individual installation
/usr/local/lib/python3.5/dist-packages/cv2.cpython-35m-x86_64-linux-gnu.so'

I rename the /usr/local/lib files:

-rw-r--r-- 1 root staff 2392480 Jun 12 15:52 dist-packages/cv2.cpython-35m-x86_6
4-linux-gnu.so.bak
-rw-r--r-- 1 root staff 3550168 Jun 12 14:09 dist-packages/cv2.so.bak

-rw-r--r-- 1 root staff 3550168 May 12 13:26 site-packages/cv2.cpython-35m-x86_6
4-linux-gnu.so.bak

------------------
Installed
sudo apt-get install python3-opencv
(This is from https://launchpad.net/~orangain/+archive/ubuntu/opencv)

asekerci@tony-utc 13:55 ~  : python3
Python 3.5.2 (default, Nov 17 2016, 17:05:23)
[GCC 5.4.0 20160609] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import cv2
>>> cv2.__file__
'/usr/lib/python3/dist-packages/cv2.cpython-35m-x86_64-linux-gnu.so'
>>>
