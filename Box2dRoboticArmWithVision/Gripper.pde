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
    void display() {
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
    void makeBody(Vec2 center) {

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

        body.createFixture(sd, 1.0);
    }
    
    boolean contains(float x, float y) {
        Vec2 worldPoint = box2d.coordPixelsToWorld(x, y);
        Fixture f = body.getFixtureList();
        boolean inside = f.testPoint(worldPoint);
        return inside;
    }
}