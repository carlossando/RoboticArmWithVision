// ContactListener to listen for collisions!

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.collision.Manifold;
import org.jbox2d.dynamics.contacts.Contact;

 class CustomListener implements ContactListener {
  CustomListener() {
  }

  // This function is called when a new collision occurs
   void beginContact(Contact cp) {
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

   void endContact(Contact cp) {
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

   void preSolve(Contact contact, Manifold oldManifold) {
    // TODO Auto-generated method stub
  }

   void postSolve(Contact contact, ContactImpulse impulse) {
    // TODO Auto-generated method stub
  }
}