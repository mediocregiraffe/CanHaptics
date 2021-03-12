/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import controlP5.*;
/* end library imports *************************************************************************************************/

ControlP5 cp5;

/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/

int damp = 0;
int low = 200;
int medium = 990;
int high = 990;
int density = 100;
/* elements definition *************************************************************************************************/
/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* end effector radius in meters */
float             rEE                                 = 0.006;


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);


PShape pGraph, joint, endEffector;

/* World boundaries */
FWorld            world;
float             worldWidth                          = 15.0;  
float             worldHeight                         = 7.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2


/* Initialization of virtual tool */
HVirtualCoupling  s;


PShape wall;
float wallW = 0.9;


/* define world objects */
FCircle           c1, c2, c3, c4, c5;


/* Define Variables*/
FBox              wall1, b1;


PImage            haplyAvatar;

/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 10;

/* text font */
PFont             F;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* screen size definition */
  size(600, 280);

  /* set font type and size */
    /* GUI setup */
  smooth();
  cp5 = new ControlP5(this);

  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
  cp5.setColorForeground(0xffaa0000);
  cp5.setColorBackground(color(255,100,100));
  cp5.setFont(font);
  cp5.setColorActive(0xffff0000);

  /* create pantagraph graphics */

  /* device setup */

  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, "COM3", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);

  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);


  widgetOne.device_set_parameters();


  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();

 
  wall1                   = new FBox(6, 1);
  wall1.setPosition(8, 4);
  wall1.setStatic(true);
  wall1.setSensor(true);
  wall1.setDensity(100);
  wall1.setNoFill();
  wall1.setNoStroke();
  world.add(wall1);
  
  damp=0;
  density = 0;
    //buttons
  cp5.addButton("One")
    .setValue(0)
    .setPosition(100,  250)
    .setSize(90, 30)
    ;
  cp5.addButton("Two")
    .setValue(0)
    .setPosition(275,  250)
    .setSize(90, 30)
    ;
  cp5.addButton("Three")
    .setValue(0)
    .setPosition(450, 250)
    .setSize(90, 30)
    ;


  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255, 0, 0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* world conditions setup */
  world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
  //world.setGravity((0.0), (0.0));
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);


  world.draw();


  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  
  if (renderingForce == false) {
    background(255);
     fill(0, 0, 0);
      textAlign(CENTER);
      text("First, click a button and then move straight down.", width/2, 70);

    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;

    if (haplyBoard.data_available()) {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();

      
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
     
      
    }
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();

      fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
      fEE.div(100000); //dynes to newtons
    

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    if (s.h_avatar.isTouchingBody(wall1)){
      s.h_avatar.setDamping(damp);
      s.h_avatar.setDensity(density);
      //wall1.setDensity(density);
        
    }else{
      s.h_avatar.setDamping(0);
      //s.h_avatar.setDensity(4);
    }
    

    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/
/* helper functions section, place helper functions here ***************************************************************/

public void One(int theValue) {
  damp = low;
  density = 300;
}
public void Two(int theValue) {
  damp = medium;
  density = 5;
}
public void Three(int theValue) {
  damp = high;
  density = high;
}
/* end helper functions section ****************************************************************************************/
