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
    void display() {
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
    void makeBody(Vec2 center, float w_, float h_, boolean lock) {
        
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

    boolean contains(float x, float y) {
        Vec2 worldPoint = box2d.coordPixelsToWorld(x, y);
        Fixture f = body.getFixtureList();
        boolean inside = f.testPoint(worldPoint);
        return inside;
    }

}
