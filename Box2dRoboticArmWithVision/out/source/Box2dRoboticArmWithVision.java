import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

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
import org.jbox2d.callbacks.ContactImpulse; 
import org.jbox2d.callbacks.ContactListener; 
import org.jbox2d.collision.Manifold; 
import org.jbox2d.dynamics.contacts.Contact; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class Box2dRoboticArmWithVision extends PApplet {















//Json reader
JSONObject values;
ArrayList<Vec2> trackerPosArr = new ArrayList<Vec2>();
FloatList gripperTimeArr = new FloatList();
//file
String filenamebase = "data";
int fileNum = 1;
String extension = ".json";
String file = filenamebase + fileNum + extension;
int maxNumFiles = 410;

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

long timedonems = 0;
boolean timedone = true;

public void setup() {
    
    
    
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

public void jsonLoader(){
    values = loadJSONObject(file);
    //Get Tracked Vectors
    JSONArray tempPosArr = values.getJSONArray("TrackerVectors");
    for (int i = 0; i < tempPosArr.size(); i++) {
        JSONObject item = tempPosArr.getJSONObject(i); 
        Vec2 center = new Vec2();
        center.x = item.getInt("x");
        center.y = item.getInt("y");
        trackerPosArr.add(center);
    }
    //Get Gripper Values
    JSONArray tempGripperArr = values.getJSONArray("ClawFunction");
    for (int i = 0; i < tempGripperArr.size(); i++) {
        JSONObject item = tempGripperArr.getJSONObject(i);
        //gripperFuncArr.add(item.getInt("Function"));
        gripperTimeArr.append(item.getInt("time"));
    }
    //println(gripperTimeArr);
}

public void draw() {
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

    for (float f : gripperTimeArr) {
        if (f > ellapsedTime-17 && f < ellapsedTime+50 && timedone == true) {
            robot.toggleMotor();
            timedone = false;
            timedonems = ellapsedTime;
        }
    }
    
    if (timedone == false) {
        if (ellapsedTime > timedonems+80) {
            timedone = true;
        }
    }

    if (trackerPosArr.size() == readingVal) {
        playingData = false;
        timerOn = false;
        delay(1000);
        RestartApp();
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
    text("Reading file data" + fileNum, 10,25);
    text("Press < or > keys to navigate between files", 10, 40);
}

public void RestartApp(){
    if(fileNum < maxNumFiles){
        fileNum++;
        file = filenamebase + fileNum + extension;
        readingVal = 0;
        trackerPosArr.clear();
        gripperTimeArr.clear();
        robot.killBody();
        box.killBody();
        // Create the Robot
        robot = new RobotArm(270,405);
        // Create the play Box
        box = new Box( 370, 325);
        jsonLoader();
        ellapsedTime=0;
        mousePressed();
    }
}

public void mouseReleased() {
//    robot.mouseReleased();
}

public void mousePressed() {
    if (!playingData) {
        playingData = true;
        startTime = millis();
        timerOn= true;
        robot.mousePressed(470, 175);
    }
}

public void AddBoundaries(float x, float y, float _width, float _height){
  boundaries.add(new Boundary(x/2, y, x, 1));
  boundaries.add(new Boundary(x, y+ _height/2, 1, _height));
  boundaries.add(new Boundary(x + _width/2, y + _height, _width,1));
  boundaries.add(new Boundary(x + _width, y+ _height/2, 1, _height));
  boundaries.add(new Boundary(x + _width + x/2, y, x, 1));
}

public void keyPressed() {
    if (keyCode == 32) {
        robot.toggleMotor();
    }else if (keyCode == 39) { //Next
        playingData = false;
        timerOn = false;
        RestartApp();
    }else if (keyCode == 37) { //Prev
        if (fileNum > 1) {
            playingData = false;
            timerOn = false;
            fileNum = fileNum -2; 
            RestartApp();
        }
    }
}

// Display a second window with the Box2D debug draw
public void drawBox2dDebug() {
    box2d.draw();
}
// A fixed boundary class

class Boundary {

  // A boundary is a simple rectangle with x,y,width,and height
  float x,y;
  float w,h;
  
  // But we also have to make a body for box2d to know about it
  Body b;

  Boundary(float x_,float y_, float w_, float h_) {
    x = x_;
    y = y_;
    w = w_;
    h = h_;

    // Define the polygon
    PolygonShape sd = new PolygonShape();
    // Figure out the box2d coordinates
    float box2dW = box2d.scalarPixelsToWorld(w/2);
    float box2dH = box2d.scalarPixelsToWorld(h/2);
    // We're just a box
    sd.setAsBox(box2dW, box2dH);


    // Create the body
    BodyDef bd = new BodyDef();
    bd.type = BodyType.STATIC;
    bd.position.set(box2d.coordPixelsToWorld(x,y));
    b = box2d.createBody(bd);
    
    // Attached the shape to the body using a Fixture
    b.createFixture(sd,1);

    b.setUserData(this);
  }

  // Draw the boundary
  public void display() {
    fill(0);
    stroke(0);
    rectMode(CENTER);
    rect(x,y,w,h);
  }

}
// A rectangular box

class Box {

  Body body;
  float w;
  float h;
  int col;
  float x;
  float y;
  Vec2 pos;

  // Constructor
  Box(float x_, float y_) {
    x = x_;
    y = y_;
    w = 50;
    h = 50;
    // Add the box to the box2d world
    makeBody(new Vec2(x,y),w,h);
    //Set user data for collision 
    body.setUserData(this);
    //Set a specific color
    col = color(175); 
  }

  // Change color when hit
  public void change(boolean touching) {
    if (touching) {
      col = color(255, 0, 0); 
    } else {
      col = color(175); 
    }
  }

  // This function removes the particle from the box2d world
  public void killBody() {
    box2d.destroyBody(body);
  }

  public boolean contains(float x, float y) {
    Vec2 worldPoint = box2d.coordPixelsToWorld(x, y);
    Fixture f = body.getFixtureList();
    boolean inside = f.testPoint(worldPoint);
    return inside;
  }

  // Drawing the box
  public void display() {
    // We look at each body and get its screen position
    pos = box2d.getBodyPixelCoord(body);
    // Get its angle of rotation
    float a = body.getAngle();

    rectMode(PConstants.CENTER);
    pushMatrix();
    translate(pos.x,pos.y);
    rotate(a);
    fill(col);
    stroke(0);
    rect(0,0,w,h);
    popMatrix();
  }


  // This function adds the rectangle to the box2d world
  public void makeBody(Vec2 center, float w_, float h_) {
    // Define and create the body
    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position.set(box2d.coordPixelsToWorld(center));
    body = box2d.createBody(bd);

    // Define a polygon (this is what we use for a rectangle)
    PolygonShape sd = new PolygonShape();
    float box2dW = box2d.scalarPixelsToWorld(w_/2);
    float box2dH = box2d.scalarPixelsToWorld(h_/2);
    sd.setAsBox(box2dW, box2dH);

    // Define a fixture
    FixtureDef fd = new FixtureDef();
    fd.shape = sd;
    // Parameters that affect physics
    fd.density = 1;
    fd.friction = 1;
    fd.restitution = 0;

    body.createFixture(fd);
    //body.setMassFromShapes();

    // Give it some initial random velocity
    //body.setLinearVelocity(new Vec2(random(4, 5), random(0, 1)));
  }

}
// ContactListener to listen for collisions!






 class CustomListener implements ContactListener {
  CustomListener() {
  }

  // This function is called when a new collision occurs
   public void beginContact(Contact cp) {
    // Get both fixtures
    Fixture f1 = cp.getFixtureA();
    Fixture f2 = cp.getFixtureB();
    // Get both bodies
    Body b1 = f1.getBody();
    Body b2 = f2.getBody();
    // Get our objects that reference these bodies
    Object o1 = b1.getUserData();
    Object o2 = b2.getUserData();

    if (o1.getClass() == Boundary.class && o2.getClass() == Box.class) {
      Box p = (Box) o2;
      p.change(true);
    } 
    // If object 2 is a Box, then object 1 must be a particle
    else if (o2.getClass() == Boundary.class && o1.getClass() == Box.class) {
      Box p = (Box) o1;
      p.change(true);
    }
  }

   public void endContact(Contact cp) {
    // Get both fixtures
    Fixture f1 = cp.getFixtureA();
    Fixture f2 = cp.getFixtureB();
    // Get both bodies
    Body b1 = f1.getBody();
    Body b2 = f2.getBody();
    // Get our objects that reference these bodies
    Object o1 = b1.getUserData();
    Object o2 = b2.getUserData();
    if (o1.getClass() == Boundary.class && o2.getClass() == Box.class) {
      Box p = (Box) o2;
      p.change(false);
    } 
    // If object 2 is a Box, then object 1 must be a particle
    else if (o2.getClass() == Boundary.class && o1.getClass() == Box.class) {
      Box p = (Box) o1;
      p.change(false);
    }
  }

   public void preSolve(Contact contact, Manifold oldManifold) {
    // TODO Auto-generated method stub
  }

   public void postSolve(Contact contact, ContactImpulse impulse) {
    // TODO Auto-generated method stub
  }
}
//Robot gripper class

class Gripper {

    //Keep track of the Body
    Body body;

    //Constructor
    Gripper(float x, float y){
        // Add the box to the box2d world
        makeBody(new Vec2(x,y));
        body.setUserData(this);
    }

    // Drawing the box
    public void display() {
        // We look at each body and get its screen position
        Vec2 pos = box2d.getBodyPixelCoord(body);
        // Get its angle of rotation
        float a = body.getAngle();

        Fixture f = body.getFixtureList();
        PolygonShape ps = (PolygonShape) f.getShape();


        rectMode(CENTER);
        pushMatrix();
        translate(pos.x, pos.y);
        rotate(-a);
        fill(175);
        stroke(0);
        beginShape();
        //println(vertices.length);
        // For every vertex, convert to pixel vector
        for (int i = 0; i < ps.getVertexCount(); i++) {
        Vec2 v = box2d.vectorWorldToPixels(ps.getVertex(i));
        vertex(v.x, v.y);
        }
        endShape(CLOSE);
        popMatrix();
    }

    // This function adds the rectangle to the box2d world
    public void makeBody(Vec2 center) {

        // Define a polygon (this is what we use for a rectangle)
        PolygonShape sd = new PolygonShape();

        Vec2[] vertices = new Vec2[4];
        vertices[0] = box2d.vectorPixelsToWorld(new Vec2(-45, 20));
        vertices[1] = box2d.vectorPixelsToWorld(new Vec2(45, 20));
        vertices[2] = box2d.vectorPixelsToWorld(new Vec2(10, -35));
        vertices[3] = box2d.vectorPixelsToWorld(new Vec2(-10, -35));

        sd.set(vertices, vertices.length);

        // Define the body and make it from the shape
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(box2d.coordPixelsToWorld(center));
        body = box2d.createBody(bd);

        body.createFixture(sd, 1.0f);
    }
    
    public boolean contains(float x, float y) {
        Vec2 worldPoint = box2d.coordPixelsToWorld(x, y);
        Fixture f = body.getFixtureList();
        boolean inside = f.testPoint(worldPoint);
        return inside;
    }
}
// Robot Linkages
class Link {

    //Keep track of a Body and width, height
    Body body;
    float w, h;
    PShape shape;
    boolean hasShape = false;
    //Constructor 1
    Link(float x, float y, float _w, float _h, boolean lock, PShape _shape){
        w = _w;
        h = _h;
        println("_shape: "+_shape);
        shape = _shape;
        hasShape = true;
        // Add the box to the box2d world
        makeBody(new Vec2(x,y),w,h,lock);
        body.setUserData(this);
    }
    //Constructor 2
    Link(float x, float y, float _w, float _h, boolean lock){
        w = _w;
        h = _h;
        hasShape = false;
        // Add the box to the box2d world
        makeBody(new Vec2(x,y),w,h,lock);
        body.setUserData(this);
    }

    // Drawing the box
    public void display() {
        // We look at each body and get its screen position
        Vec2 pos = box2d.getBodyPixelCoord(body);
        // Get its angle of rotation
        float a = body.getAngle();

        rectMode(CENTER);
        pushMatrix();
        translate(pos.x,pos.y);
        rotate(-a);
        fill(175);
        stroke(0);
        if (hasShape) {
            shape(shape);
        }else{
            rect(0,0,w,h);
        }
        popMatrix();
    }

    // This function adds the rectangle to the box2d world
    public void makeBody(Vec2 center, float w_, float h_, boolean lock) {
        
        // Define a polygon (this is what we use for a rectangle)
        PolygonShape ps = new PolygonShape();
        float box2dW = box2d.scalarPixelsToWorld(w_/2);
        float box2dH = box2d.scalarPixelsToWorld(h_/2);
        ps.setAsBox(box2dW, box2dH);

        // Define a fixture
        FixtureDef fd = new FixtureDef();
        fd.shape = ps;
        // Parameters that affect physics
        fd.density = 1;
        fd.friction = 1;
        fd.restitution = 0;

        // Define the body and make it from the shape
        BodyDef bd = new BodyDef();
        //Define is it is static or dynamic
        if (lock) bd.type = BodyType.STATIC;
        else bd.type = BodyType.DYNAMIC;
        //bd.type = BodyType.KINEMATIC;
        bd.position.set(box2d.coordPixelsToWorld(center));

        //Create Body and Fixture
        body = box2d.createBody(bd);
        body.createFixture(fd);
    }

    public boolean contains(float x, float y) {
        Vec2 worldPoint = box2d.coordPixelsToWorld(x, y);
        Fixture f = body.getFixtureList();
        boolean inside = f.testPoint(worldPoint);
        return inside;
    }

}
//Constructor class for the robotic Arm

class RobotArm {
    //List of our Robot Linkages
    ArrayList<Link> links;
    //List of joints
    RevoluteJoint joint1;
    RevoluteJoint joint2;
    RevoluteJoint joint3;
    RevoluteJoint jointArmL;
    RevoluteJoint jointArmR;
    MouseJoint mouseJoint;
    //Mouse attraction
    Spring spring;
    // Joints Offsets
    Vec2 offsetJoint1;
    Vec2 offsetJoint2;
    Vec2 offsetJoint3;
    Vec2 offsetJoint4;
    //Gripper
    Gripper gripper;
    boolean clawIsOpen = true;
    //270,220,50
    RobotArm(float x, float y){
        links = new ArrayList<Link>();
        //Offsets between links
        Vec2 offset1 = new Vec2(0, -270/2 + 10);
        Vec2 offset2 = new Vec2(offset1.x + 220/2 - 10,offset1.y*2);
        Vec2 offset3 = new Vec2(offset2.x + 220/2 -10 ,offset2.y + 70/2 -10);
        Vec2 offset4 = new Vec2(offset3.x - 35 ,offset3.y + 70/2+10);
        Vec2 offset5 = new Vec2(offset3.x + 35 ,offset3.y + 70/2+10);
        //Create the linkages
        links.add(new Link(x, y, 100, 40, true));
        links.add(new Link(x + offset1.x, y + offset1.y, 30, 270, false));
        links.add(new Link(x + offset2.x, y + offset2.y, 220, 25, false));
        gripper = new Gripper(x + offset3.x, y+offset3.y);
        links.add(new Link(x + offset4.x, y+offset4.y, 13, 70, false));
        links.add(new Link(x + offset5.x, y+offset5.y, 13, 70, false));
        //Revolute Joints
        RevoluteJointDef rjd1 = new RevoluteJointDef();
        rjd1.initialize(links.get(0).body, links.get(1).body, links.get(0).body.getWorldCenter());

        RevoluteJointDef rjd2 = new RevoluteJointDef();
        offsetJoint1 = new Vec2(links.get(1).body.getWorldCenter().x,links.get(1).body.getWorldCenter().y + box2d.scalarPixelsToWorld(270/2) + box2d.scalarPixelsToWorld(-10));
        rjd2.initialize(links.get(1).body, links.get(2).body, offsetJoint1);

        RevoluteJointDef rjd3 = new RevoluteJointDef();
        offsetJoint2 = new Vec2(gripper.body.getWorldCenter().x ,gripper.body.getWorldCenter().y + box2d.scalarPixelsToWorld(70/2) + box2d.scalarPixelsToWorld(-10));
        rjd3.initialize(links.get(2).body, gripper.body, offsetJoint2);

        RevoluteJointDef rjd4 = new RevoluteJointDef();
        offsetJoint3 = new Vec2(gripper.body.getWorldCenter().x  + box2d.scalarPixelsToWorld(-30), gripper.body.getWorldCenter().y + box2d.scalarPixelsToWorld(-10));
        rjd4.initialize(gripper.body, links.get(3).body, offsetJoint3);

        RevoluteJointDef rjd5 = new RevoluteJointDef();
        offsetJoint4 = new Vec2(gripper.body.getWorldCenter().x  + box2d.scalarPixelsToWorld(30), gripper.body.getWorldCenter().y + box2d.scalarPixelsToWorld(-10));
        rjd5.initialize(gripper.body, links.get(4).body, offsetJoint4);
        
        //------------Motors------------
        //gripper
        rjd3.motorSpeed = PI;
        rjd3.maxMotorTorque = 3000.0f; 
        rjd3.enableMotor = true;
        //left claw  
        rjd4.motorSpeed = -PI*3;      
        rjd4.maxMotorTorque = 25000.0f;
        rjd4.enableMotor = true;
        //right claw
        rjd5.motorSpeed = PI*3;       
        rjd5.maxMotorTorque = 25000.0f; 
        rjd5.enableMotor = true;      
        //-------------------------------
        //Constrains
        rjd1.enableLimit = true;
        rjd1.lowerAngle = -PI/3;
        rjd1.upperAngle = PI/3;

        rjd2.enableLimit = true;
        rjd2.lowerAngle = -PI/2;
        rjd2.upperAngle = PI/2;

        rjd3.enableLimit = true;
        rjd3.lowerAngle = -PI/5;
        rjd3.upperAngle = PI/5;

        //Gripper constrains
        rjd4.enableLimit = true;
        rjd4.lowerAngle = -PI/9;
        rjd4.upperAngle = PI/9;

        rjd5.enableLimit = true;
        rjd5.lowerAngle = -PI/6;
        rjd5.upperAngle = PI/6;

        joint1 = (RevoluteJoint) box2d.world.createJoint(rjd1);
        joint2 = (RevoluteJoint) box2d.world.createJoint(rjd2);
        joint3 = (RevoluteJoint) box2d.world.createJoint(rjd3);
        jointArmL = (RevoluteJoint) box2d.world.createJoint(rjd4);
        jointArmR = (RevoluteJoint) box2d.world.createJoint(rjd5);
        //Spring
        spring = new Spring();
    }

    public void display() {
        // Display all the linkages
        for (Link l: links) {
            l.display();
        }
        //draw the gripper
        gripper.display();

        // Alert the spring to the new location
        //spring.update();
        // Draw the spring
        spring.display();
    }

   // This function removes the particle from the box2d world
  public void killBody() {
    for (Link l: links) {
        box2d.destroyBody(l.body);
    }
    box2d.destroyBody(gripper.body);
  }

    //Update the string location based on the tracked color blob
    public void followColor(float x, float y){
        spring.update((600-x)*1.3f,y);
    }

    // When the mouse is released we're done with the spring
    public void mouseReleased() {
        spring.destroy();
    }

    // When the mouse is pressed we. . .
    public void mousePressed(float x, float y) {
        // Check to see if the mouse was clicked on the box
        if (gripper.contains(x, y)) {
            // And if so, bind the mouse location to the box with a spring
            spring.bind(x,y,gripper.body);
        }
    }

    // Turn the motor on or off
    public void toggleMotor() {
        if (!clawIsOpen) {
            clawIsOpen = true;
            jointArmL.setMotorSpeed(-PI*2);
            jointArmR.setMotorSpeed(PI*2);
        } else {
            clawIsOpen = false;
            jointArmL.setMotorSpeed(PI*2);
            jointArmR.setMotorSpeed(-PI*2);
        }
    }

}
// Class to describe the spring joint (displayed as a line)

class Spring {

  // This is the box2d object we need to create
  MouseJoint mouseJoint;

  Spring() {
    // At first it doesn't exist
    mouseJoint = null;
  }

  // If it exists we set its target to the mouse location 
  public void update(float x, float y) {
    if (mouseJoint != null) {
      // Always convert to world coordinates!
      Vec2 mouseWorld = box2d.coordPixelsToWorld(x,y);
      mouseJoint.setTarget(mouseWorld);
    }
  }

  public void display() {
    if (mouseJoint != null) {
      // We can get the two anchor points
      Vec2 v1 = new Vec2(0,0);
      mouseJoint.getAnchorA(v1);
      Vec2 v2 = new Vec2(0,0);
      mouseJoint.getAnchorB(v2);
      // Convert them to screen coordinates
      v1 = box2d.coordWorldToPixels(v1);
      v2 = box2d.coordWorldToPixels(v2);
      // And just draw a line
      stroke(0);
      strokeWeight(1);
      line(v1.x,v1.y,v2.x,v2.y);
    }
  }


  // This is the key function where
  // we attach the spring to an x,y location
  // and the Box object's location
  public void bind(float x, float y, Body box) {
    // Define the joint
    MouseJointDef md = new MouseJointDef();
    // Body A is just a fake ground body for simplicity (there isn't anything at the mouse)
    md.bodyA = box2d.getGroundBody();
    // Body 2 is the box's boxy
    md.bodyB = box;
    // Get the mouse location in world coordinates
    Vec2 mp = box2d.coordPixelsToWorld(x,y);
    // And that's the target
    md.target.set(mp);
    // Some stuff about how strong and bouncy the spring should be
    md.maxForce = 10000.0f;
    md.frequencyHz = 5.0f;
    md.dampingRatio = 0.9f;

    // Make the joint!
    mouseJoint = (MouseJoint) box2d.world.createJoint(md);
  }

  public void destroy() {
    // We can get rid of the joint when the mouse is released
    if (mouseJoint != null) {
      box2d.world.destroyJoint(mouseJoint);
      mouseJoint = null;
    }
  }

}
  public void settings() {  size(1200, 800);  smooth(); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "Box2dRoboticArmWithVision" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
