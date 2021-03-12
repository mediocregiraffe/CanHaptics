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

/* end library imports *************************************************************************************************/



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



/* elements definition *************************************************************************************************/
/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* end effector radius in meters */
float             rEE                                 = 0.006;

/* virtual wall parameter  */
float             kWall                               = 2000;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.10);

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

final int         worldPixelWidth                     = 1280;
final int         worldPixelHeight                    = 820;
PShape pGraph, joint, endEffector;

/* World boundaries */
FWorld            world;
float             worldWidth                          = 32.0;  
float             worldHeight                         = 21.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;
PShape wall;

float wallW = 0.9;


/* define world objs */
FCircle           c1, c2, c3, c4, c5;
FBox              wall1,wall2,wall3,wall4,wall5,wall6, wall7, wall8;


PImage            haplyAvatar;
boolean done=false;

/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 10;

int               mode                                =0;

/* text font */
PFont             F;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* screen size definition */
  size(1280, 820);

  /* set font type and size */
  F                   = createFont("Arial", 16, true);

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

 
 //main right 
  wall1                   = new FBox(wallW, 2);
  wall1.setPosition(7, 9);
  wall1.setStatic(true);
  //wall1.setSensor(true);
  wall1.setNoFill();
  wall1.setNoStroke();
  world.add(wall1);
  
  
    wall2                   = new FBox(wallW, 1);
  wall2.setPosition(15, 13);
  wall2.setStatic(true);
 // wall2.setSensor(true);
  wall2.setNoFill();
  wall2.setNoStroke();
  world.add(wall2);
  
    wall3                   = new FBox(wallW, 1.5);
  wall3.setPosition(16, 5);
  wall3.setStatic(true);
  wall3.setNoFill();
  wall3.setNoStroke();
  world.add(wall3);
  
    wall4                   = new FBox(wallW, 3);
  wall4.setPosition(9, 17);
  wall4.setStatic(true);
  wall4.setNoFill();
  wall4.setNoStroke();
  world.add(wall4);
  
    wall5                   = new FBox(wallW, 1);
  wall5.setPosition(25, 15);
  wall5.setStatic(true);
  wall5.setNoFill();
  wall5.setNoStroke();
  world.add(wall5);
  
    wall6                   = new FBox(wallW, 1);
  wall6.setPosition(19, 3);
  wall6.setStatic(true);
  wall6.setNoFill();
  wall6.setNoStroke();
  world.add(wall6);
  
      wall7                  = new FBox(4, 1);
  wall7.setPosition(21, 8);
  wall7.setStatic(true);
  wall7.setNoFill();
  wall7.setNoStroke();
  world.add(wall7);
  
      wall8                   = new FBox(wallW, 1);
  wall8.setPosition(28, 3);
  wall8.setStatic(true);
  wall8.setNoFill();
  wall8.setNoStroke();
  world.add(wall8);





  /* Mode 3 Button */
  c2                  = new FCircle(1.0);
  c2.setPosition(edgeTopLeftX+2, edgeTopLeftY+worldHeight/2.0-4);
  c2.setFill(200, 0, 0);
  c2.setStaticBody(true);
  c2.setSensor(true);
  world.add(c2);

  /* Mode 3 Button */
  c3                  = new FCircle(1.0);
  c3.setPosition(edgeTopLeftX+2, edgeTopLeftY+worldHeight/2.0-6);
  c3.setFill(0, 0, 200);
  c3.setStaticBody(true);
  c3.setSensor(true);
  world.add(c3);

  /* Mode 4 Button */
  c4                  = new FCircle(1.0);
  c4.setPosition(edgeTopLeftX+2, edgeTopLeftY+worldHeight/2.0-2);
  c4.setFill(100, 50, 150);
  c4.setStaticBody(true);
  c4.setSensor(true);
  world.add(c4);



  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255, 0, 0); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
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
      text("Touch a coloured circle to change settings. How does each setting make you feel? Please choose one word.", width/2, 70);
      textFont(F, 22);


    if (mode ==2) {  
          text("Mode 2", width/2, 700);
    }else if (mode ==3) {  
          text("Mode 1", width/2, 700);
    } else if (mode ==4) {
  
      text("Mode 3", width/2, 700);

    } else {

    }

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
    
    //decides what happens when we switch to each mode
      if (s.h_avatar.isTouchingBody(c2)) {
      mode =2;
      s.h_avatar.setSensor(false);
      s.h_avatar.setDamping(800);
      wall1.setSensor(false);
      wall2.setSensor(false);
      wall3.setSensor(false);
      wall4.setSensor(false);
      wall5.setSensor(false);
      wall6.setSensor(false);
      wall7.setSensor(false);
    } else if (s.h_avatar.isTouchingBody(c3)) {
      mode =3;
      s.h_avatar.setSensor(false);
      wall1.setSensor(false);
      wall2.setSensor(true);
      wall3.setSensor(true);
      s.h_avatar.setDamping(200);
    } else if (s.h_avatar.isTouchingBody(c4)) {    
      mode =4;
      s.h_avatar.setSensor(false);
      s.h_avatar.setDamping(500);
    } 

    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/
/* helper functions section, place helper functions here ***************************************************************/



/* end helper functions section ****************************************************************************************/
