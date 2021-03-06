# Pick and Place Vision-Guided Robotic Arm (Playground)
**This is one of 3 branches, this branch is the main app, the playground, you can only play in this branch, if you want to record data go to the "Recorder App" branch, else if you want to playback your recording, go to "Recorder App" branch.**

Kinematic 2D 2 link arm for pick and place controlled by a color blob detected by an Image processing algorithm.

![image](https://user-images.githubusercontent.com/29716233/136808799-4d03863d-6508-4bb3-a105-9fadffd595cb.png)

## Getting Started

Download the project and open it in processing 3.5.4, to be able to run the code, it is indispensable to have the Box2D library installed. 

The project has two subprojects, the first one being the "BloblDetectionWithColorGiver", this code is to help you select the parameters for the object with which you plan to do the tracking.

The second subproject is the robotic arm, once the parameters are passed to the main tab ("Box2dRoboticArmWithVision"), the program is ready to use.


### Prerequisites

Requirements to be able to run de code
- [Processing 3.5.4](https://processing.org/)
- [Box2d for processing](https://github.com/shiffman/Box2D-for-Processing)
- [Video The Processing Foundation](https://processing.org/reference/libraries/video/index.html)

### Installing

Install processing 3.5.4 or the latest stable release, and open it.

To install the librarie go to Sketch -> Import Library -> Add Library... and search for

    Box2D for Processing

and

    Video The Processing Foundation

To install the project you can download the zip file or you can clone it

    $ git clone https://github.com/carlossando/RoboticArmWithVision.git

## Running the code
### BloblDetectionWithColorGiver

This code is going to be used to obtain the detection parameters of the color blob for our second program. 

Once you run the program, you will press the color you want to track with the mouse. To adjust the parameters you will use A and Z for the distance threshold, and S and X for the color threshold. Your target is to find a single blob that doesnt fluctuate. These parameters will vary depending on the background, lighting, etc.


![image](https://user-images.githubusercontent.com/29716233/136818668-fed2a0e8-d0f9-4e92-a2eb-85e35b014bc3.png)

Once you got the parameters your are going to pass the to the next program. In this case the parameters are:

     Color: -13280642  //Given in the Console
     
     Distance Threshold: 50  //Given in the Console or in the top right area of the screen
     
     Color Threshold: 40 //Given in the Console or in the top right area of the screen
     
![image](https://user-images.githubusercontent.com/29716233/136819330-5a16b9fe-c730-419b-9f74-377b25237f7e.png)

To get the robotic arm to follow the blob, show the blob to the camera and click on the gripper, then you will be able to see the force vector and the robotic arm trying to follow the blob.

![image](https://user-images.githubusercontent.com/29716233/136820710-db1de3d7-c7bf-4941-9eab-6a938aaf8843.png)


## Authors

  - **Carlos Sandoval Contreras** -
    [CarlosSando](https://github.com/carlossando)

See also the list of
[contributors](https://github.com/carlossando/RoboticArmWithVision/graphs/contributors)
who participated in this project.
