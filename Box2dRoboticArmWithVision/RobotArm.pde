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
        rjd3.maxMotorTorque = 3000.0; 
        rjd3.enableMotor = true;
        //left claw  
        rjd4.motorSpeed = -PI*3;      
        rjd4.maxMotorTorque = 25000.0;
        rjd4.enableMotor = true;
        //right claw
        rjd5.motorSpeed = PI*3;       
        rjd5.maxMotorTorque = 25000.0; 
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

    void display() {
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
  void killBody() {
    for (Link l: links) {
        box2d.destroyBody(l.body);
    }
    box2d.destroyBody(gripper.body);
  }

    //Update the string location based on the tracked color blob
    void followColor(float x, float y){
        spring.update((600-x)*1.3,y);
    }

    // When the mouse is released we're done with the spring
    void mouseReleased() {
        spring.destroy();
    }

    // When the mouse is pressed we. . .
    void mousePressed(float x, float y) {
        // Check to see if the mouse was clicked on the box
        if (gripper.contains(x, y)) {
            // And if so, bind the mouse location to the box with a spring
            spring.bind(x,y,gripper.body);
        }
    }

    // Turn the motor on or off
    void toggleMotor() {
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