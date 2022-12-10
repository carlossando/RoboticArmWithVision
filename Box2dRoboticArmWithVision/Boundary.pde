// A fixed boundary class
class Boundary {
  // A boundary is a simple rectangle with x,y,width,and height
  float xPos, yPos;
  float width, height;
  Body b;

  Boundary(float xPos, float yPos, float width, float height) {
    // Store the position and size of the boundary
    this.xPos = xPos;
    this.yPos = yPos;
    this.width = width;
    this.height = height;

    // Define the polygon and set its dimensions directly
    PolygonShape sd = new PolygonShape();
    sd.setAsBox(
      box2d.scalarPixelsToWorld(width/2),
      box2d.scalarPixelsToWorld(height/2)
    );

    // Create the body
    BodyDef bd = new BodyDef();
    bd.type = BodyType.STATIC;
    bd.position.set(box2d.coordPixelsToWorld(xPos, yPos));
    b = box2d.createBody(bd);

    // Attach the shape to the body using a Fixture
    b.createFixture(sd, 1);

    b.setUserData(this);
  }

  // Draw the boundary
  void display() {
    fill(0);
    stroke(0);
    rectMode(CENTER);
    rect(xPos, yPos, width, height);
  }
}

