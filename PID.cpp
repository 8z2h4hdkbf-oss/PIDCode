#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Intake               motor         2               
// FRW                  motor         8               
// MRW                  motor         7               
// BRW                  motor         6               
// FLW                  motor         5               
// MLW                  motor         9               
// BLW                  motor         20              
// Controller1          controller                    
// Inertial1            inertial      19              
// DescoreMech          digital_out   C               
// Lift                 digital_out   D               
// Stick                motor         10              
// Matchloader          digital_out   B               
// Blocker              digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;

// A global instance of competition
competition Competition;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //Calibrates the inertial during pre auto so it doesn't add any extra
  //time or delay onto the program at the start
  Inertial1.calibrate();
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


//Settings

//These variables are modified/tuned to determine how the robot moves while correcting

/************************************************************************
kI = Integral: I personally don't use this one at all, but it is modified
by the previous error value and can be used to change how the robot moves 
over a period of time. Essentially what it does is it determines how much
the robot tries to move to correct itself, which on a straight will cause
the robot to oscillalte more when the kI value increases. This can be
useful in various situations like if you need the robot to cover a little
bit more space on the straightaway with its front end to intake better or
something along those lines. For the most part there isn't a whole lot of
use for it.
************************************************************************/
double kI = 0.0;

/************************************************************************
kP = Potential: This variable is modified by the error of the robot and
is the variable that determines what value the robot is going to oscillate
around. It determines consistency in the movement of the robot as well as
choosing how fast the robot is going to move when it's certain distance
away from the desired point. Increasing this value will cause the robot
to oscillate further down the line than a lower value, and typically will
make the robot move faster.
************************************************************************/
double kP = 0.1;

/************************************************************************
kD = Derivative: This variable is modified by the difference between the
current error and the error in the last cycle (about 20 milliseconds).
Modifying this variable will change how much the robot oscillates, which 
means a higher value will cause it to slow quicker than a lower value. 
This determines how accurate the robot is when it is trying to get to a 
desired value.
************************************************************************/
double kD = 0.12;


//Turn Settings
//These are essentially the same concept as the previous settings, but
//for turning rather than lateral movement

double turnkP = 0.35;
double turnkI = 0.0;
double turnkD = 0.5;


//These are the variables that will be modified during autonomous to determine
//where you want the robot to be moving to.
int desiredValue = 0;
int desiredTurnValue = 0;


//maxspeed is a variable that will be modified in later functions to
//determine what the fastest the robot will go during any given movement
//will be. It is programmed in as a ratio to allow easy conversion from
//volts into percent to allow for ease of use.
double maxspeed = 100/100;


//These are the variables that will be modified later to determine
//how to make the robot move based on where it is and where it just was

//error: calculates the difference between desired value and current value
double error;

//prevError: stores the value of the error during the last cycle (about 20 msec ago)
double prevError = 0;

//derivative: the value of the difference between current error and prevError
double derivative;

//Adds the amount of error from current cycle to itself to keep track of how much
//it has been off the whole time.
double totalError = 0;


//Same concept as last set of variables, but applied to turning instead of lateral movement
double turnError;
double turnPrevError = 0;
double turnDerivative;
double turnTotalError = 0;

//Gets the gear ratio of the drive train from the user
double gearRatio = 4/5;

//Gets the wheel size diameter (in inches) from the user
double wheelSize = 3.25;

//Calculates the degrees to inches based on gear ratio and wheel size
//This allows the user to use inches instead of degrees for ease of programming
double inchesVar = (wheelSize * 3.14159) / (360 * gearRatio);

//Boolean value that can be used later in the auto program
bool resetDrive = false;


//Variable that is used to represent what direction the robot should be going while turning
float Direction;

//Variable that will be used to decide whether or not the PID function will be used
bool PIDActivated = true;


//This is where the actual PID function starts. It will use the variables previously
//defined to determine where the robot wants to go compared to where it wants to be
//and will modify its speed accordingly
int DrivePID() {

  //This while loop will check to see if the boolean PIDActivated is true, and
  //if it is it will run the PID code. This allows you to disable the PID function
  //by swithing the boolean to false if needed
  while (PIDActivated) {

    //This loop will check every cycle to see if the boolean resetDrive is true
    //and if it is it will set the position of the motors back to 0 to prevent
    //your desired values from simply adding onto each other
    if (resetDrive) {

      //Sets the resetDrive boolean back to false so the loop doesn't indefinitely run
      resetDrive = false;

      //Resets the motor position values to 0
      FLW.setPosition(0, degrees);
      FRW.setPosition(0, degrees);
      BLW.setPosition(0, degrees);
      BRW.setPosition(0, degrees);
      MLW.setPosition(0, degrees);
      MRW.setPosition(0, degrees);

      //Resets the desiredValue variable to 0 so it knows to start a new movement
      desiredValue = 0;

      //Resets the maxspeed variable to have the robot moving at a default 
      //speed of 100%
      maxspeed = 100/100;
    }

    //Thse next three lines have the brain screen print the values for
    //turnError and error so you know how far off from what you want it to be the
    //current distance and direction the robot is facing is
    Brain.Screen.print(turnError);
    Brain.Screen.newLine();
    Brain.Screen.print(error / inchesVar);

    //This is the variable from of the direction the inertial sensor is facing
    //Technically this variable is only going to be exact every 20 msec because this
    //line will only run once a cycle. This is still pretty accurate though and won't
    //be a huge issue
    float Rotation = Inertial1.heading(degrees);


    //These two variables are what determine where the robot actually is for lateral
    //movement. It is based of of the position of the motors in the middle each cycle
    double rightPosition = (MRW.position(degrees));
    double leftPosition = (MLW.position(degrees));


    /////////////////////////////////////
    ////Lateral PID movement controls////
    /////////////////////////////////////


    //The averagePosition variable combines the values of the left side and right
    //side of the drive base into one variable to attempt to determine where the robot
    //truly is. This probably isn't the most effective way to do this and may be revisited
    //later because one motor being way off could still read as in the right spot this way
    //and mess up the position of the second motor as well
    double averagePosition = (rightPosition + leftPosition) / 2;

    //This is what gives the error variable and actual value
    //It takes the desired value and subtracts the position of the
    //motors to determine how far off it is
    error = desiredValue - averagePosition;

    //This gives the derivative variable a numerical value
    //It takes the value of the error and subtracts the previous error
    //each cycle to determine what the derivative is.
    derivative = error - prevError;

    //This will add the error value to the total error each cycle.
    //You may notice this variable isn't actually used at all in this code,
    //and that's because it is used to modify kI, which I prefer not to use
    totalError += error;

    //This variable will determine how fast the robot should be moving
    //based on where it is compared to where it should be. There is a 
    //distinct lack of totalError and kI which can simply be added by adding
    // + totalError * kI to the parentheses
    double motorPower = (error * kP + derivative * kD);

    /////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////
    ////Turning PID movement controls////
    /////////////////////////////////////

    //This next section is what determines what direction to have the robot
    //turn based on which would be the fastest. It checks to see what the distance
    //from where the robot is at directionally and compares to what it's supposed to 
    //get to and will spin one direction or another based on that value.
    if (Rotation - desiredTurnValue > 0 && Rotation - desiredTurnValue < 180) {
      Direction = -1 * (Rotation - desiredTurnValue);
    } else if (Rotation - desiredTurnValue > 180) {
      Direction = 360 - (Rotation - desiredTurnValue);
    } else if (desiredTurnValue - Rotation > 0 && desiredTurnValue - Rotation < 180) {
      Direction = desiredTurnValue - Rotation;
    } else if (desiredTurnValue - Rotation > 180) {
      Direction = -1 * (360 - (desiredTurnValue - Rotation));
    }


    //This is essentially the same as the values from the last group
    //for lateral movement, but applied to turning instead.
    turnError = Direction;

    turnDerivative = turnError - turnPrevError;

    turnTotalError += turnError;

    double turnMotorPower = (turnError * turnkP + turnDerivative * turnkD);

    //////////////////////////////////////////////////////////////////////////////////////


    //This tells the motors how fast to spin and in which direction to spin
    //It takes the values from motorPower and turnMotorPower and constantly spin
    //at a rate determined by those two variables and if the variable is 0 that means
    //it's at the spot it wants to be and will be spinning at a speed of 0. It also
    //has the value of the motor powers being divided by the max speed variable which will 
    //cause it to spin at a limit of a certain amount of percent determined by the user
    FLW.spin(forward, (motorPower + turnMotorPower) / maxspeed, voltageUnits::volt);
    FRW.spin(forward, (motorPower - turnMotorPower) / maxspeed, voltageUnits::volt);
    BLW.spin(forward, (motorPower + turnMotorPower) / maxspeed, voltageUnits::volt);
    BRW.spin(forward, (motorPower - turnMotorPower) / maxspeed, voltageUnits::volt);
    MLW.spin(forward, (motorPower + turnMotorPower) / maxspeed, voltageUnits::volt);
    MRW.spin(forward, (motorPower - turnMotorPower) / maxspeed, voltageUnits::volt);


    //Sets the prevError to error every cycle (20 msec) It does this for turning as well
    turnPrevError = turnError;
    prevError = error;

    //This causes it to wait 20 msec before repeating the loops so it doesn't overload with values and crash
    vex::task::sleep(20);

    //This part simply resets the brain screen so new values can be printed on it
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
  }

  return 1;
}

//This function uses the modifiable PID variables to tell the robot
//how far to go, what speed to go, and how long to wait before moving
//on to the next command. 
//forwards(inches, speed, msec);
void forwards(int inchesFor, int speedFor, int waitTimeFor) {
  maxspeed = 100/speedFor;
  desiredValue = inchesFor * inchesVar;
  wait(waitTimeFor, msec);
}

//This function does the same thing as the previous function, but
//interperts the distance value as negative to tell the robot to go backwards
//backward(inches, speed, msec);
void backward(int inchesBack, int speedBack, int waitTimeBack) {
  maxspeed = 100/speedBack;
  desiredValue = -inchesBack * inchesVar;
  wait(waitTimeBack, msec);
}

//This function uses the PID variables to tell the robot which direction to face
//what speed to turn, and wait time between commands. If you want to do a swing turn,
//you can do this by decreasing the wait time to have the robot start its lateral movement
//while turning
//Turn(direction, speed, msec);
void Turn(int turnDirection, int speedTurn, int waitTimeTurn) {
  maxspeed = 100/speedTurn;
  desiredTurnValue = turnDirection;
  wait(waitTimeTurn, msec);
}

//This function runs the reset drive portion of the PID loop to reset the position of
//the motors. You'll notice in the reset loop it doesn't reset the position of the heading
//of the robot, so on turns 90 is alway 90 and -47 is always -47. It's based on heading
//not turn degrees. There's also a small wait time at the end of the reset function, which
//is to prevent it from trying to move immediately after resetting because it will screw up
//the distances if you don't have it do that. The wait value is modifiable if you want, but
//it can't be any less than 20 msec because that's how long a cycle is in the loop. 500 is
//a good number that runs consistently
void reset() {
  resetDrive = true;
  wait(500, msec);
}


//These are variables that I've declared early on to help with the driver control
//portion of the code.

//liftup is used to modify whether or not the lift on the robot is up or down
bool liftup = false;
//wing is used to modify whether or not the descore mech is in use or not
bool wing = false;
//stikspeedy is used later on to change whether or not the stick mech is moving full speed
bool stikspeedy = true;
//loaderdown is used to change whether or not the matchloading mechanism is in use
bool loaderdown = false;


//This next set of functions are all do the same thing in different instances
//Anytime one of them is called they toggle the corresponding boolean to flip
//values
void flyaway (){
  wing = !wing;
}
void lifting (){
  liftup = !liftup;
}
void loading (){
  loaderdown = !loaderdown;
}


//Polynomial function setup

//This function is here to make driving easier. It makes it so the drivetrain
//changes with the joy stick exponentially allowing for smaller more precise movements
//as well as full speed when pushed all the way.
double curve(double input, double k = 4) {
    double x = input / 100.0;
    double curved = k * (x * x * x) + (1 - k) * x;
    return curved * 100.0;
}


void autonomous(void) {

  //This is the first thing to change to make sure the PID task will run when called
  PIDActivated = true;

  //This is the second thing that must be added to make sure the PID task is called
  //so all that work wasn't for nothing
  vex::task PID(DrivePID);

  //This waitUntil command is here to make sure the code waits to run until the 
  //inertial sensor is done calibrating so it does not return weird heading values
  waitUntil(Inertial1.isCalibrating() == false);

  //I personally like to run a reset function right away to make sure
  //the desired value is set to 0 and nothing weird happened in pre auto
  reset();

  //Next it is a good idea to set the position and speed of each different motor
  //Sometimes a timeout is also a good idea to have hardcoded in so the motors don't
  //try to spin longer than they are able to

  /**********************************************************/
  /******DO NOT CHANGE THE WHEEL SPEEDS OR LOCATIONS!!!******/
  /**THIS WILL SCREW UP THE ENTIRE PURPOSE OF THE PID LOOP!**/
  /**********************************************************/

  Stick.setPosition(0, degrees);
  Stick.setVelocity(35, pct);
  Intake.setVelocity(100, percent);
  Stick.setTimeout(1800, msec);

  //This will run a very simple autonomous that will have the robot go 
  //forwards 24 inches at full speed, go backward 24 inches at half speed,
  //turn to heading 90 at 75% speed, and then turn to heading -90 at full speed.
  //This will let you see how it's moving different distances at different speeds
  //compared to how you want it to be moving so you can go back and modify or turn
  //the kP, kI, and kD values
  forwards(24, 100, 2000);
  reset();
  backward(24, 50, 2000);
  reset();

  //Notice you don't have to reset if you aren't moving laterally between turns
  //This is because resetting doesn't change the heading so you're free to turn
  //as much as you'd like
  Turn(90, 75, 2000);
  Turn(-90, 100, 2000);

}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void usercontrol(void) {
  // User control code here, inside the loop

  /***********************************************************************/
  /***THE PID MUST BE TURNED OFF OR THE USER CONTROL WILL BREAK DOWN!!!***/
  /***********************************************************************/
  PIDActivated = false;

  //These are the call backs that you use to activate the various pneummatics
  //All this does is when a certain button is pressed it will toggle functions from
  //earlier to change their values. Later in the code that is used to toggle the pneumatics
  Controller1.ButtonX.pressed(lifting);
  Controller1.ButtonUp.pressed(flyaway);
  Controller1.ButtonA.pressed(loading);

  //This piece of code calibrates the inertial again before driver control starts
  //This is used to print of what the heading is on the brain screen while driver 
  //control is running.
  Inertial1.calibrate();
  
  while (1) {
    //The first thing that happens here in the driver control loop is
    //it prints off the values of the inertial heading as well as the
    //values of how far the robot has gone. This can be used to help
    //make writing skeleton code much easier. You can use these values 
    //to simply push the robot around on the field while running the driver
    //control code and look at the brain to see what kind of values you need.
    //This can make writing autonomous code ten times faster and easier compared
    //to what it would be.
    Brain.Screen.print(Inertial1.heading(degrees));
    Brain.Screen.newLine();
    Brain.Screen.print(((MLW.position(degrees) + MRW.position(degrees)) / 2) / inchesVar);

    //This chunk of code will allow you to press the A button to reset the
    //values of the motor positions in the drivetrain. That way you can check
    //more than one distance for writing code without haing to restart the code
    if (Controller1.ButtonA.pressing()) {
      MLW.setPosition(0, degrees);
      MRW.setPosition(0, degrees);
    }


    //This sets the wheel stopping mode to coast
    //This is better than brake or hold because 
    //typically when a drivetrain is going full
    //speed it is going at a speed much to fast
    //to be able to stop on point without wrecking
    //motors or gears.
    FLW.setStopping(coast);
    FRW.setStopping(coast);
    BLW.setStopping(coast);
    BRW.setStopping(coast);
    MLW.setStopping(coast);
    MRW.setStopping(coast);

    //These next two variables determine how far the controller joysticks
    //are being pushed at any given moment and stores that value in a variable
    double leftInput = Controller1.Axis3.position();
    double rightInput = Controller1.Axis2.position();

    //These two determine the speed the motors should be spinning based on the
    //joystick values, and modified by the curve function to get back a value that
    //will allow the joysticks to be controlled exponentially
    double leftPower = curve(leftInput, 0.5);
    double rightPower = curve(rightInput, 0.5);

    //The wheels are technically always spinning, but when the 
    //joystick is not being pushed it is just spinning at 0 percent speed
    //The left and right power variables are put into the code to allow the
    //joysticks to determine the speed of the robot
    FLW.spin(fwd, leftPower, pct);
    FRW.spin(fwd, rightPower, pct);
    BLW.spin(fwd, leftPower, pct);
    BRW.spin(fwd, rightPower, pct);
    MLW.spin(fwd, leftPower, pct);
    MRW.spin(fwd, rightPower, pct);

    //This next group of commands are here to control the pneumatics
    //They are set up so that they are constantly being set to whatever
    //the value of a specific boolean from earlier is. They are being toggled
    //by the callbacks from pre driver
    Lift.set(liftup);
    DescoreMech.set(wing);
    Matchloader.set(loaderdown);
    
    //This next if statement will check to see if the boolean stikspeedy is
    //true, and if it is it will set the velocity of the stick motor to 100
    //percent, and if it isn't it will set to 35 percent.
    if(stikspeedy){
      Stick.setVelocity(100, pct);
    }
    else{
      Stick.setVelocity(35, pct);
    }

    //This if statement will check to see if the boolean lifup is true
    //and if it is it will set stikspeedy to false. The goal of this was
    //to have the stick motor be spinning slower while the lift is raised
    //so it would score better
    if (liftup){
      stikspeedy = false;
    }
    else{
      stikspeedy = true;
    }

    //This makes the intake spin forward when button L1 is being pressed, 
    //spins reverse when L2 is being pressed, and stops when neither button is being pressed
    if (Controller1.ButtonL1.pressing()) {
      Intake.spin(forward, 100, pct);
    } 
    else if (Controller1.ButtonL2.pressing()) {
      Intake.spin(reverse, 100, pct);
    } else {
      Intake.stop();
    }

    //This makes the stick spin forward when button L1 is being pressed, 
    //spins reverse when L2 is being pressed, and stops when neither button is being pressed
    //Notice how it always spins 100 percent in reverse. This is to make the robot run
    //more efficiently because even if it goes up slowly it'll still go down quickly
    if (Controller1.ButtonR1.pressing()){
      Stick.spin(forward);
    }
    else if (Controller1.ButtonR2.pressing()){
      Stick.spin(reverse, 100, pct);
    }
    else{
      Stick.stop(hold);
    }

    //The blocker is the mechanism that held the game objects in, so this set of
    //code will check to see if the stick motor has spun past a certain point, and if
    //it has it assumes the robot is scoring so the blocker opens
    if (Stick.position(degrees) > 10){
      Blocker.set(true);
    }
    else{
      Blocker.set(false);
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.

    //Resets the brain screen to get ready to print new values every cycle
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
  }
}


// Main will set up the competition functions and callbacks.

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);


  // Run the pre-autonomous function.
  pre_auton();


  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}