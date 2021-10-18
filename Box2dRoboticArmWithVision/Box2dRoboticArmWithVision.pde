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

//Json reader
JSONArray values;

ArrayList<Vec2> trackerPosArr = new ArrayList<Vec2>();
int readingVal = 0;
// A reference to our box2d world
Box2DProcessing box2d;
boolean playingData;

//List of our boundaries
ArrayList<Boundary> boundaries;
//List of our Robot Linkages
RobotArm robot;
// The playBox
Box box;

//Floor Boundary variables
float floorVal[] = {540, 425 ,80 ,50};

//Timer variables
boolean timerOn = false;
long startTime = 0;
long ellapsedTime = 0;

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
    box = new Box( 370, 325);
    
    //Add a listener for collisions
    box2d.world.setContactListener(new CustomListener());
    
    //load Json file
    jsonLoader();
}

void jsonLoader(){
    values = loadJSONArray("data1.json");
    JSONArray tempPosArr = values.getJSONArray(0);
    for (int i = 0; i < tempPosArr.size(); i++) {
        JSONObject item = tempPosArr.getJSONObject(i); 
        Vec2 center = new Vec2();
        center.x = item.getInt("x");
        center.y = item.getInt("y");
        trackerPosArr.add(center);
    }
    //println(trackerPosArr);
}

void draw() {
    background(200);
    //println(new Vec2(mouseX,mouseY));

    // We must always step through time!
    if (playingData) {
        box2d.step();   
    }

    // Display all the boundaries
    for (Boundary wall: boundaries) {
        wall.display();
    }
    //Display the Robot
    robot.display();

    if (playingData) {
        robot.followColor(trackerPosArr.get(readingVal).x,trackerPosArr.get(readingVal).y);
        readingVal++;
    }

    if (trackerPosArr.size() == readingVal) {
        playingData = false;
    }

    //Display the Play Box
    box.display();

    //Time
    if (timerOn) {
        ellapsedTime = (millis() - startTime);
    }

    //Display the Text
    fill(0);
    text("Ellapsed Time= " + ellapsedTime + " ms", 10,10);
}

void mouseReleased() {
//    robot.mouseReleased();
}

void mousePressed() {
    // for (Blob b : blobs) {
    //     if (b.size() > 500) {
    //         robot.mousePressed(b.minx, b.miny);
    //         startTime = millis();
    //         timerOn= true;
    //     }
    // }
    
    if (!playingData) {
        playingData = true;
        startTime = millis();
        timerOn= true;
        robot.mousePressed(480, 181);
        //robot.mousePressed(robot.gripper.body.getWorldCenter().x,robot.gripper.body.getWorldCenter().y);
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
    }else if (keyCode == 83) {
        
    }
}
