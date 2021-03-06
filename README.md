# Self Driving RC Car
The aim of this project was to develop an on-board computer vision system for an RC car to make it navigate autonomously in indoor and outdoor environments.

You can see all features of the code in this repository in the following video:

[![Link to video](video-link_image.png)](https://www.youtube.com/watch?v=NFtwV7u9N58&t=4s)

The whole project was documented on:

https://futuretechmaker.wordpress.com/projects/cv_autonomous_car/

This was a high-school project, you can read the final report here:

https://drive.google.com/file/d/0B9eB9OXoMMAbc3pSX0RPaGdIaXc/view
- - -

### Version 0.99 (27 Feb 2016)
Almost all the code has been refactored and cleaned. It has been tested to work on Mac and Linux (Ubuntu 14.04, probably also in most other systems). Windows is unfortunately not supported because I only wrote the file path handling for Unix systems, the rest of the code should work. Note that the camera 
Development is at the moment abandoned. If you have any question/suggestion please don't hesitate to contact me.

### Version 0.9 (16 Jun 2015)
- Part of the avoidance path planning algorithm has been rewritten. No known bugs so far.
- Setting the DO_MAIN_LOOP variable to true makes the stereoCam and the avoidance algorithms work together in real-time with nice 2D visualisation using the Simulator.
- 3D reprojection has been improved.
- The global variables are now in the `Globals.h` header file.
- Some improvements for populating obstacle scenarios from 3D maps.

### Version 0.5 (24 Apr 2015)
- Added PCL library support for 3D point-cloud visualisation.
- Added a class for communicating with Arduino.
- Streamlined calibration procedure for real-time calibration pattern detection.
- Corrected several bugs in the custom reprojection method.
- Many other bugs were also corrected.

### Version 0.3.5
- Added Arduino serial communication (platform-independent).
- Improved the camera calibration routine (16 images of the calibration pattern instead of 9, this really improves the results)

### Version 0.3.2
- Added obstacle detection (alpha) and avoidance.
- Different SGBM parameters for generic stereo cameras and DUO3D camera.

### Version 0.3
- Added support for the DUO3D camera.
- Added option to use uncalibrated stereo cameras.
- Added makefile.

### Version 0.2.6
- Many bug corrections, now the path planning algorithm does not produce any error at all.
- Also streamlined the simulator interface.

### Version 0.2.5
- Avoidance path planning works! There are still some subtle bugs to correct.

### Version 0.2.4.2
- Major re-write of the avoidance path planning algorithm. Still running tests.

### Version 0.2.4
- Avoidance path planner now generates some circular avoidance paths, but is still under active development.

### Version 0.2.3.9
- Avoidance path planer now detects if going straight is possible.

### Version 0.2.3.8
- Created an avoidance simulator interface and navigation simulator interface.

### Version 0.2.3.5
- Improved simulator interface.

### Version 0.2.3
- Added `PathPlaning`, `Simulator` and `ObstacleScenario` classes.
- Finished simulator and started path planing.

### Version 0.2
- Many improvements on the StereoCamera and Odometry classes.
- `StereoCamera`: added an interactive camera calibration tool and other useful functions.

### Version 0.1.2
- Code is now carefully commented and there are lots of command line feedback.

### Version 0.1.1
- Added documentation.

### Version 0.1
- Initial skeleton.
