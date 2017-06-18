## WhyCon

### A precise, efficient and low-cost localization system 

_WhyCon_ is a version of a vision-based localization system that can be used with low-cost web cameras, and achieves millimiter precision with very high performance.
The system is capable of efficient real-time detection and precise position estimation of several circular markers in a video stream. 
It can be used both off-line, as a source of ground-truth for robotics experiments, or on-line as a component of robotic systems that require real-time, precise position estimation.
_WhyCon_ is meant as an alternative to widely used and expensive localization systems. It is fully open-source.
_WhyCon-orig_ is WhyCon's original, minimalistic version that was supposed to be ROS and openCV independent.


| WhyCon example application (video)  | Scenario description |
| ------ | ----------- |
|[![WhyCon applications](https://raw.githubusercontent.com/wiki/gestom/WhyCon/pics/whycon.png)](https://www.youtube.com/watch?v=KgKrN8_EmUA"AAAA")|-precise docking to a charging station (EU project STRANDS),<br/> -fitness evaluation for self-evolving robots (EU proj. SYMBRION),<br/>-relative localization of UAV-UGV formations (CZ-USA project COLOS),<br/>-energy source localization in (EU proj REPLICATOR),<br/>-robotic swarm localization (EU proj HAZCEPT).|

The _WhyCon_ system was developed as a joint project between the University of Buenos Aires, Czech Technical University and University of Lincoln, UK.
The main contributors were [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao), [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) and [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en). Each of these contributors maintains a slightly different version of WhyCon.

| WhyCon version  | Application | Main features | Maintainer|
| --------------- | ----------- | ------ | ----- |
| [WhyCon-orig](../../) | general | 2D, 3D, lightweight, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao)|
| [WhyCon-ROS](https://github.com/lrse/whycon) | general | 2D, 3D, ROS + lightweight | [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao) |
| [SwarmCon](https://github.com/gestom/CosPhi/tree/master/Localization) | μ-swarms | 2D, individual IDs, autocalibration | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |
| [Caspa-WhyCon](http://robotics.fel.cvut.cz/faigl/caspa/) | UAVs | embedded, open HW-SW solution | [Jan Faigl](https://scholar.google.co.uk/citations?user=-finD_sAAAAJ&hl=en) |
| [Social-card](https://github.com/strands-project/strands_social/tree/hydro-devel/social_card_reader) | HRI | orientation translated to commands  | [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) |


#### Where is it described ?

<i>WhyCon</i> was first presented on International Conference on Advanced Robotics 2013 [[1](#references)], later in the Journal of Intelligent and Robotics Systems [[2](#references)] and finally at the Workshop on Open Source Aerial Robotics during the International Conference on Intelligent Robotic Systems, 2015 [[3](#references)]. Its early version was also presented at the International Conference of Robotics and Automation, 2013 [[4](#references)].
If you decide to use this software for your research, please cite <i>WhyCon</i> using the one of the references provided in this [bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/WhyCon.bib) file.

-----

### Setting up WhyCon 

#### Quick setup for initial testing

1. Install the required <a href="#libraries">libraries</a>.
1. Check the available resolutions, framerates and formats of your USB camera by running <b>luvcview -L</b>. 
1. Run <b>luvcview</b> and adjust your camera settings (exposure, brightness etc).
1. Download the software from GitHub and go to the <b>src</b> directory.
1. Adjust the camera resolution in the <b>main/whycon.cpp</b>.
1. Compile the software - just type <b>make</b>.
1. Download, resize and print one circular <a href="etc/test.pdf">pattern</a> - you have the pattern also in the <b>etc/test.pdf</b> file.
1. Try a test run - you need to run the binary from the <b>bin</b> directory. Type <b>./whycon /dev/videoX 1</b>, where X is the number of the camera and 1 tells the system to track one pattern.</li> 
1. You should see the image with some numbers below the circle. Pressing <b>D</b> shows the segmentation result.
1. At this point, you can also change camera brightness, exposure, contrast by pressing <i>(SHIFT) b, e, c</i> respectively. These settings are stored in <i>etc/camera.cfg</i> and reloaded on restart.
1. Open your browser to view localhost:6666. You should see the circle position.

#### Setting up the coordinate system

1. Calibrate your camera using the MATLAB (or Octave) calibration toolbox and put the <b>Calib_Results.m</b> in the <b>etc</b> directory.
1. If you have resized the markers (their default size is 122mm), then adjust their diameter in the <b>main/whycon.cpp</b> file.
1. Print additional four circular <a href="etc/test.pdf">markers</a> and place to the corners of your (reclangular) operational space.
1. Modify the dimensions of the operation space in the <b>main/whycon.cpp</b> and call <b>make</b> to recompile - the system will now assume that the four markers are at positions [0,0],[fieldLength,0], [0,fieldWidth],[fieldLength,fieldWidth]. 
1. Position and fixate your camera so that it has all four circles in it's field of view.
1. Go to <b>bin</b> directory and run  <b>./whycon /dev/videoX Y</b>, where X is the number of your camera and Y is the number of patterns you want to track, i.e. Y=NxM+4.
1. Once all the patterns are found, press <b>a</b> and the four outermost patterns will be used to calculate the coordinate system.
1. Alternatively, you can press <b>r</b> and then click the four circles that define the coordinate system.
1. Pressing 1 should show you the patterns' positions in camera-centric coordinates (x-axis equals to camera optical axis), pressing 2 and 3 will display marker coordinates in user-defined 2D or 3D coordinate systems.
1. Pressing <b>+</b>,<b>-</b> changes the number of localized patterns.

#### To postprocess the stored videos

1. To create a log of the robot positions, simply create an <b>output</b> folder at the directory where you run the <b>whycon</b>.
2. If your camera supports the MJPEG format, then the system will create a video in the <b>output</b> folder as well.
3. If your camera does not support MJPEG, <b>whycon</b> will save the video feed as a serie of bitmaps, that you can process later as well.
4. You can run <b>whycon video_file_name Y</b> to process that video in the same way as when using the camera, i.e. <b>video_file_name</b> instead of <b>/dev/videoX</b>.
5. Processing a saved video rather than the camera feed is likely to provide more precise results.
6. Running the system with a <b>nogui</b> argument e.g. <b>./whycon /dev/video0 1 nogui</b> causes text-only output - this can speed-up postprocessing.
7. Logs and videos might be large - to prevent saving logs and videos, run the system with <b>nolog</b> or <b>novideo</b> argument.

#### Some additional remarks

2. At this point, you can start experimenting with the syste by adding whatever features you might think useful.
3. We have tried to comment the code so an experienced programmer should be able to alter the system accordingly. However, if you have any questions regarding the code, feel free to contact [Tom Krajnik](http://scholar.google.co.uk/citations?user=Qv3nqgsAAAAJ&hl=en&oi=ao) or [Matias Nitsche](https://scholar.google.co.uk/citations?user=Z0hQoRUAAAAJ&hl=en&oi=ao)
4. If you use this localization system for your research, please don't forget to cite at least one relevant paper from these [bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/WhyCon.bib) records.
5. Have fun!
</ol>

<hr>
### Dependencies

All the following libraries are probably in your packages.

1. <b>libsdl1.2-dev</b> for graphical user interface.
2. <b>libsdl-ttf2.0-dev</b> to print stuff in the GUI.
3. <b>libncurses5-dev</b> to print stuff on the terminal.
4. <b>luvcview</b> to set-up the camera.

### References
1. T. Krajník, M. Nitsche et al.: <b>[External localization system for mobile robotics.](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icar_whycon.pdf)</b> International Conference on Advanced Robotics (ICAR), 2013. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icar_whycon.bib)].
2. T. Krajník, M. Nitsche et al.: <b>[A Practical Multirobot Localization System.](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_JINT_whycon.pdf)</b> Journal of Intelligent and Robotic Systems (JINT), 2014. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_JINT_whycon.bib)].
3. M. Nitsche, T. Krajník et al.: <b>[WhyCon: An Efficent, Marker-based Localization System.](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_irososar_whycon.pdf)</b> IROS Workshop on Open Source Aerial Robotics, 2015. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2015_irososar_whycon.bib)].
4. J. Faigl, T. Krajník et al.: <b>[Low-cost embedded system for relative localization in robotic swarms.](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6630694)</b> International Conference on Robotics and Automation (ICRA), 2013. [[bibtex](http://raw.githubusercontent.com/wiki/gestom/CosPhi/papers/2013_icra_whycon.bib)].

### Acknowledgements

The development of this work is currently supported by the Czech Science Foundation project 17-27006Y _STRoLL_.
In the past, the work was supported by EU within its Seventh Framework Programme project ICT-600623 _STRANDS_.
The Czech Republic and Argentina have given support through projects 7AMB12AR022, ARC/11/11 and 13-18316P.
We sincerely acknowledge [Jean Pierre Moreau](http://jean-pierre.moreau.pagesperso-orange.fr/infos.html) for his excellent libraries for numerical analysis that we use in our project. 
