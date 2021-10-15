# Pick and Place Vision-Guided Robotic Arm
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

### Installing

Install processing 3.5.4 or the latest stable release, and open it.

To install the library go to Sketch -> Import Library -> Add Library... and search for

    Box2D for Processing

To install the project you can download the zip file or you can clone it

    $ git clone https://github.com/carlossando/RoboticArmWithVision.git

## Running the code

This code is going to be used to obtain the detection parameters of the color blob for our second program. 

Once you run the program, you will press the color you want to track with the mouse. To adjust the parameters you will use A and Z for the distance threshold, and S and X for the color threshold. Your target is to find a single blob that doesnt fluctuate. These parameters will vary depending on the background, lighting, etc.


![image](https://user-images.githubusercontent.com/29716233/136818668-fed2a0e8-d0f9-4e92-a2eb-85e35b014bc3.png)

Once you got the parameters your are going to pass the to the next program. In this case the parameters are:

     Color: -13280642  //Given in the Console
     
     Distance Threshold: 50  //Given in the Console or in the top right area of the screen
     
     Color Threshold: 40 //Given in the Console or in the top right area of the screen
     
![image](https://user-images.githubusercontent.com/29716233/136819330-5a16b9fe-c730-419b-9f74-377b25237f7e.png)

To get the robotic arm to follow the blob, **show the blob to the camera and _CLICK_ on the gripper, Once it is clicked the program will start**, then you will be able to see the force vector and the robotic arm trying to follow the blob, and the program will start recording data to a txt file.

![image](https://user-images.githubusercontent.com/29716233/136820710-db1de3d7-c7bf-4941-9eab-6a938aaf8843.png)

## Recording data

Once you´ve clicked on the gripper while showing the tracking object to the camera, you'll see everything start moving, and you will be able to control the robotic arm with the blob. The program will start recording the coordenates of the tracking object(blob) and will save the data in a text file called **dataset.txt**. **IT IS IMPORTANT TO END THE PROGRAM USING THE _"s"_ KEY**, to make sure it saves the dataset correctly.

![image](https://user-images.githubusercontent.com/29716233/137501088-02b7c70f-de38-4fbb-b03b-88f01e723171.png)

Once it is done, rename the file to something different if you dont want your data overwritten.

The data should look somthing like this

![image](https://user-images.githubusercontent.com/29716233/137501746-16890c9c-c689-4696-81f1-31641ec3874f.png)


## Authors

  - **Carlos Sandoval Contreras** -
    [CarlosSando](https://github.com/carlossando)

See also the list of
[contributors](https://github.com/carlossando/RoboticArmWithVision/graphs/contributors)
who participated in this project.
