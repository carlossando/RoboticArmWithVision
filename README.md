# Pick and Place Vision-Guided Robotic Arm (Playback Recording Data)

**This is one of 3 branches, this branch is the playback-recordings, just for visualizing the data, if you want to record data go to the "Recorder App" branch, else if you want to play with the physics, go to "Playground App" branch.**

Kinematic 2D 2 link arm for pick and place controlled by a color blob detected by an Image processing algorithm.
The program is modified to play the data: This program will only play back the runs you did in the "recording-app" branch, for that, you must save the data in a folder and export it to the folder named "data" in the workspace.

![image](https://user-images.githubusercontent.com/29716233/137936486-9e31cea9-9680-484d-b4c4-87a1b4800974.png)

## Getting Started

Download the project and open it in processing 3.5.4, to be able to run the code, it is indispensable to have the Box2D library and the Video library by The Processing foundation installed. 

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

First you must have recorded some previous data from the "Recorder App" branch, and stored it in a safe place. Then you must export the .JSON files to this workspace inside a folder called data.

![image](https://user-images.githubusercontent.com/29716233/137937000-582e1e61-ad3a-42e4-bb00-febbbc4a74a3.png)

![image](https://user-images.githubusercontent.com/29716233/137937140-acfda9e2-8c69-4cc0-829d-e3c9f7459e6d.png)

Make sure to number the files in ascending order starting with the number 1 and in the format of "data#" so the program can read everything successfully.

After you have done that, make sure to modify the line 24 of the main program "Box2dRoboticArmWithVision.pde" with the number of files your program has.

![image](https://user-images.githubusercontent.com/29716233/137937969-1be76bfc-2193-42d2-b1ad-9f03fa7e862c.png)

## To start running the recordings
Finally you are ready to run the program, just play start, and **press anywhere in the screen.**

![image](https://user-images.githubusercontent.com/29716233/137938617-6f23e37a-cd35-406c-903c-b126f41b5989.png)

## Authors

  - **Carlos Sandoval Contreras** -
    [CarlosSando](https://github.com/carlossando)

See also the list of
[contributors](https://github.com/carlossando/RoboticArmWithVision/graphs/contributors)
who participated in this project.
