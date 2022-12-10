/// A rectangular box
class Box {
  Body body;
  float width;
  float height;
  color col;

  // Constructor
  Box(float xPos, float yPos) {
    // Store the box's dimensions
    this.width = 50;
    this.height = 50;
    // Add the box to the box2d world
    makeBody(new Vec2(xPos, yPos), width, height);
    // Set user data for collision 
    body.setUserData(this);
    // Set a specific color
    col = color(175); 
  }

  // Change color when hit
  void change(boolean touching) {
    if (touching) {
      col = color(255, 0, 0); 
    } else {
      col = color(175); 
    }
  }

  // This function removes the particle from the box2d world
  void killBody() {
    box2d.destroyBody(body);
  }

  boolean contains(float x, float y) {
    Vec2 worldPoint = box2d.coordPixelsToWorld(x, y);
    Fixture f = body.getFixtureList();
    boolean inside = f.testPoint(worldPoint);
    return inside;
  }

  // Drawing the box
  void display() {
    // We look at each body and get its screen position
    Vec2 pos = box2d.getBodyPixelCoord(body);
    // Get its angle of rotation
    float angle = body.getAngle();

    rectMode(PConstants.CENTER);
    pushMatrix();
    translate(pos.x, pos.y);
    rotate(angle);
    fill(col);
    stroke(0);
    rect(0, 0, width, height);
    popMatrix();
  }

   // This function adds the rectangle to the box2d world
  void makeBody(Vec2 center, float width, float height) {
    // Define and create the body
    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    bd.position.set(box2d.coordPixelsToWorld(center));
    body = box2d.createBody(bd);

    // Define a polygon (this is what we use for a rectangle)
    PolygonShape sd = new PolygonShape();
    float box2dWidth = box2d.scalarPixelsToWorld(width/2);
    float box2dHeight = box2d.scalarPixelsToWorld(height/2);
    sd.setAsBox(box2dWidth, box2dHeight);

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
    body.setLinearVelocity(new Vec2(random(4, 5), random(0, 1)));
  }
}
