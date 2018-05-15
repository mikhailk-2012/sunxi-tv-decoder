These are the TV-decoder drivers for the Allwinner A20.

* sun7i_tvd-linux-3.3: This is the driver as found in Linux 3.3 for Cubieboard
(https://github.com/cubieboard2/linux-sunxi/tree/sunxi-3.3-cb2)
* sun7i_tvd-lichee-android-linux3.3: The working driver distributed in Android 4.2 for MarsBoard 
(http://www.haoyuelectronics.com/service/A10-A20/A20-android-source-code/)
* sun7i_tvd-linux-3.4-working: Barebones porting to Linux 3.4

The Cubieboard and Android versions have some slight differences in the registers initialized and
in clock initialization code. My porting is pretty rough but can capture images (changes by Rodolfo
Zitellini):

* I "added" some missing #defines form missing files (this should be done in a better way)
* I forced the video field to be returned V4L2_FIELD_NONE as the docs specify
* I fixed the returned video dimensions (so ffmpeg is happy for example)

Changes by Iñigo Huguet:

* Partially fixed clock selection: select a used clock if it has the desired frequency, keeping the other free
* Return correct bytesperline format info
* Return applied format in s_fmt ioctl, as required by v4l2
* Inform about min buffers num required
* Fixed bug with buffers management when dequeueing is not fast enough
* Makefile allow to build as loadadble module
* Enum formats and framesizes in different ioctl, as required by v4l2
* Select camera or multi-camera using s_input (before it could only be done in a driver-specific way)
* Allow fps selection


BUILDING
========

To have it working add the following lines to Kconfig:

```
source "drivers/media/video/sun7i_tvd/Kconfig"
```

and Makefile in the /drivers/media/video directory.:

```
obj-$(CONFIG_SUN7I_TVD) += sun7i_tvd/
```
Then build/rebuild the kernel.

You can also build it as loadable module:

```
cd SOURCE_DIR
make
make install
```

and load it with:

```
modprobe sunxi_tvd
```


TESTING
=======

Test with ffmpeg:

```
ffmpeg -f v4l2 -s 720x576 -pix_fmt nv12 -r 25 -t 10 -i /dev/video1 m.mpg
```

Test with gstreamer (save to file and play with mpv player, gstreamer videosink doesn't work, I don't know why):

```
gst-launch-1.0 v4l2src device=/dev/video1 ! video/x-raw,format=NV12,width=720,height=576,framerate=25/1 ! filesink location=raw_video.bin
mpv --demuxer rawvideo --demuxer-rawvideo-w 720 --demuxer-rawvideo-h 576 --demuxer-rawvideo-mp-format nv12 --demuxer-rawvideo-fps 25 raw_video.bin
```


SELECTING CAMERA/INPUT
======================

* Check available inputs: `v4l2-ctl -d /dev/video1 --list-inputs`
* Get current input: `v4l2-ctl -d /dev/video1 --get-input`
* Change input: `v4l2-ctl -d /dev/video1 --set-input=X`
* Check available formats and framesizes FOR CURRENT INPUT: `v4l2-ctl -d /dev/video1 -list-formats-ext`

Available inputs:

* input 0 = TV input 1
* input 1 = TV input 2
* input 2 = TV input 3
* input 3 = TV input 4
* input 4 = TV inputs 1 + 2
* input 5 = TV inputs 1 + 3
* input 6 = TV inputs 1 + 4
* input 7 = TV inputs 2 + 3
* input 8 = TV inputs 2 + 4
* input 9 = TV inputs 3 + 4
* input 10 = TV inputs 1 + 2 + 3 + 4

Input types:

* Inputs 0 - 3: video from only one camera
* Inputs 4 - 9: video splited in 2 parts, one camera in each one
* Input 10: video splited in 4 parts, one camera in each one


SELECTING FRAMESIZE
===================

Framesize can be selected by Gstreamer or ffmpeg as shown above. They select
the framesize calling s_fmt ioctl of the driver.

Available framesizes:

* for single camera (inputs 0 - 3): 720x480, 720x576, 704x480, 704x576
* for 2 cameras, vertical split (inputs 4 - 9): 1440x480, 1440x576, 1408x480, 1408x576
* for 2 cameras, horizontal split (inputs 4 - 9): 720x960, 720x1152, 704x960, 704x1152
* for 4 cameras (input 10): 1440x960, 1440x1152, 1408x960, 1408x1152

To display multi-camera with ffmpeg or gstreamer, select input with v4l2-ctl
first and run ffmpeg or gstreamer with commands from above with a valid resolution
for the chosen input.


SELECTING FPS
=============

Different framerates (fps) can be selected, use lower framerates if the
hardware can't process video at the camera framerate (usually 25fps for PAL
cameras and 30fps for NTSC cameras). Framerate is not ensured to be applied
with high precission, and it highly depends on the camera framerate, over
which the driver have no control at all (real fps will be the camera fps
divided by an integer).

Gstreamer and ffmpeg can select fps calling to s_parm ioctl of the driver.

