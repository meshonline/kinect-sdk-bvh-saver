![](http://www.mesh-online.net/mocap.png)
# kinect-sdk-bvh-saver
This project is based on the official Kinect SDK example, it will automatically save many skeleton animation clips to bvh files.<br/>
<br/>
Then you can import the bvh files into Blender or other softwares to smooth animation curves. I use Blender's default smooth key setting to smooth all rotation channels, the results is acceptable.<br/>
<br/>
The project has disabled kinect's built in smooth feature by default, for the result is poor, if you want to enable the built in smooth feature, there is a macro definition named 'FILTER_MODE', change it from 0 to 1, 2, 3 to enable three types of smooth feature.
### Requirements
Windows 7(32 bit or 64 bit)<br/>
kinect for xbox 360<br/>
Microsoft Kinect for Windows SDK 1.7<br/>
Microsoft Visual Studio 2010 Express Edition
### How to use
1.Install Microsoft Kinect for Windows SDK 1.7.<br/>
2.Plug kinect for xbox 360<br/>
3.Install Microsoft Visual Studio 2010 Express Edition.<br/>
4.Open the project, build and run.<br/>
<br/>
When the Kinect camera detects your body, the software will automatically record the skeleton animation to a bvh file, after the Kinect camera can not detect your body, the software will finish recording the bvh file, the output path is the current directory.<br/>
<br/>
Enter the Kinect camera's viewport, perform actions, then leave the viewport, repeat the steps, you can record many skeleton animation clips at one time.
### Thanks
1.[Derek Hendrickx's KinectMotionCapture](https://github.com/derekhendrickx/KinectMotionCapture)<br/>
2.[Kyle Weicht's 3D math library](https://github.com/awesomekyle/math)<br/>
3.[Birdy's Notebook](http://bediyap.com/programming/convert-quaternion-to-euler-rotations/)<br/>
4.[sunchy's Kinect_to_BVH_Console](https://github.com/isunchy/Kinect_to_BVH_Console)
### License
The MIT License (MIT)
