### Installation instructions

Start of by going to the home directory of the Raspberry Pi: `cd ~`

#### ZBar
Commands:
1. `git clone https://github.com/mchehab/zbar`
2. `cd zbar`
3. `autoreconf -vfi && ./configure` (if error `autoreconf: failed to run autopoint` see following [article](https://dausruddin.com/autoreconf-failed-to-run-autopoint-no-such-file-or-directory/))
4. `make`
5. `make install`

If Libtool error issue, see following [article](https://stackoverflow.com/questions/18978252/error-libtool-library-used-but-libtool-is-undefined)

Again enter the home directory of the Raspberry Pi: `cd ~`

#### OpenCV
Make sure you are in fact in the home directory of the Raspberry Pi!

Commands:
1. `git clone https://github.com/opencv/opencv`
2. `git clone https://github.com/opencv/opencv_contrib`

If you check the directory contents, there should now be two folders in your home directory: `opencv` and `opencv_contrib`

3. `cd opencv`
4. `mkdir build`
5. `cd build`
6. `cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D BUILD_opencv_java=OFF -D BUILD_opencv_python3=OFF -D BUILD_opencv_python2=OFF -D WITH_V4L=ON -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D OPENCV_GENERATE_PKGCONFIG=ON -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..`

Increase swap size by carrying out the following commands below:

7. `sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=1024/g' /etc/dphys-swapfile`
8. `sudo /etc/init.d/dphys-swapfile stop`
9. `sudo /etc/init.d/dphys-swapfile start`

Now make sure you're still in the `build` folder

10. `make` (this will take a billion years, don't exit even if it seems to hang at 100%!)

Once finished:

11. `make install`

Go to home directory

12. `cd ~`

13. `sudo ldconfig` (as long as it does not give an error output it is OK)

Decrease swap size again

14. `sudo sed -i 's/CONF_SWAPSIZE=1024/CONF_SWAPSIZE=100/g' /etc/dphys-swapfile`
15. `sudo /etc/init.d/dphys-swapfile stop`
16. `sudo /etc/init.d/dphys-swapfile start`

#### Enable camera
[Use the instructions here to enable the camera on the Raspberry Pi](https://www.raspberrypi.org/documentation/usage/camera/installing.md) (make sure you click FINISH and do a reboot!)

#### nlohmann/json

TODO
