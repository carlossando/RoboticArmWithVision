import shiffman.box2d.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.contacts.*;
import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import processing.video.*;

//Video for image recognition
Capture video;

//Create an object to save info to a txt file
PrintWriter output;

//--------------------------
//Blob parameters --> Set up this parametes based on the other program
color trackColor = -3978719;
float threshold = 60;
float distThreshold = 70;
//--------------------------
//keep a track of the blobs
ArrayList<Blob> blobs = new ArrayList<Blob>();

// A reference to our box2d world
Box2DProcessing box2d;
boolean recordingData;

//List of our boundaries
ArrayList<Boundary> boundaries;
//List of our Robot Linkages
RobotArm robot;
// The playBox
Box box;

//Floor Boundary variables
float floorVal[] = {540, 425 ,80 ,50};

void setup() {
    size(1200, 800);
    smooth();
    
    // Initialize box2d physics and create the world
    box2d = new Box2DProcessing(this);
    box2d.createWorld();
    // Set a Gravity for the system
    box2d.setGravity(0, -25);

    //Boundaries
    boundaries = new ArrayList<Boundary>();
    AddBoundaries(floorVal[0],floorVal[1],floorVal[2],floorVal[3]);

    //Create the Robot
    robot = new RobotArm(270,405);
    //Create the play Box
    box = new Box( 350, 325);

    // Create a new file in the sketch directory
    output = createWriter("dataset.txt"); 

    //Add a listener for collisions
    box2d.world.setContactListener(new CustomListener());
    
    //Video processing
    video = new Capture(this,"pipeline:autovideosrc");
    video.start();
}

void captureEvent(Capture video) {
  video.read();
}

void draw() {
    //Process the image before all
    processImage();

    background(200);
    //println(new Vec2(mouseX,mouseY));

    // We must always step through time!
    if (recordingData) {
        box2d.step();   
    }

    // Display all the boundaries
    for (Boundary wall: boundaries) {
        wall.display();
    }
    //Display the Robot
    robot.display();
    for (Blob b : blobs) {
        if (b.size() > 500) {
            Vec2 center = b.getCenter();
            robot.followColor(center.x,center.y);
        }
    }
    //Display the Play Box
    box.display();
    
    if (recordingData) {
        for (Blob b : blobs) {
            if (b.size() > 500) {
                Vec2 center = b.getCenter();
                output.print("(" + center.x + "),(" + center.y + "),"); // Write the coordinate to the file
            }
        }
    }
    
    //Display the Text
    fill(0);
    text("Press the space bar to open and close the gripper",10,20);
}

void processImage(){
    video.loadPixels();
    //image(video, 0, 0);
    blobs.clear();

    // Begin loop to walk through every pixel
    for (int x = 0; x < video.width; x++ ) {
        for (int y = 0; y < video.height; y++ ) {
            int loc = x + y * video.width;
            // What is current color
            color currentColor = video.pixels[loc];
            float r1 = red(currentColor);
            float g1 = green(currentColor);
            float b1 = blue(currentColor);
            float r2 = red(trackColor);
            float g2 = green(trackColor);
            float b2 = blue(trackColor);

            float d = distSq(r1, g1, b1, r2, g2, b2); 

            if (d < threshold*threshold) {

                boolean found = false;
                for (Blob b : blobs) {
                if (b.isNear(x, y)) {
                    b.add(x, y);
                    found = true;
                    break;
                }
                }

                if (!found) {
                    Blob b = new Blob(x, y);
                    blobs.add(b);
                }
            }
        }
    }

    // for (Blob b : blobs) {
    //     if (b.size() > 500) {
    //     //b.show();
    //     println("blob center: "+ b.minx + " " + b.miny);
    //     }
    // }
}


void mouseReleased() {
//    robot.mouseReleased();
}

void mousePressed() {
    for (Blob b : blobs) {
        if (b.size() > 500) {
            robot.mousePressed(b.minx, b.miny);
            recordingData = !recordingData;
        }
    }
}

void AddBoundaries(float x, float y, float _width, float _height){
  boundaries.add(new Boundary(x/2, y, x, 1));
  boundaries.add(new Boundary(x, y+ _height/2, 1, _height));
  boundaries.add(new Boundary(x + _width/2, y + _height, _width,1));
  boundaries.add(new Boundary(x + _width, y+ _height/2, 1, _height));
  boundaries.add(new Boundary(x + _width + x/2, y, x, 1));
}

void keyPressed() {
    if (keyCode == 32) {
        robot.toggleMotor();
        output.print("(ClawOperated),");
    }else if (keyCode == 83) {
        output.flush(); // Writes the remaining data to the file
        output.close(); // Finishes the file
        exit(); // Stops the program
    }
}


// Custom distance functions w/ no square root for optimization
float distSq(float x1, float y1, float x2, float y2) {
  float d = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
  return d;
}


float distSq(float x1, float y1, float z1, float x2, float y2, float z2) {
  float d = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) +(z2-z1)*(z2-z1);
  return d;
}
