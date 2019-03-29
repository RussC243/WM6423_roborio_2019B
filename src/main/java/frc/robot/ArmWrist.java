package frc.robot;

//import static org.junit.Assume.assumeTrue;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot.OurBots;
import edu.wpi.first.wpilibj.*;

/**
 * All of the control for the arm and wrist are in this class.
 */
public class ArmWrist {
  //at 20mS per update, changing target from zero to full up or full down would take 1000*0.020 = 20 sec
  //So to speed this up,  we will change target by the following respective factors every 20mS.
  //Changing these values only changes the rate the targets change not the PID response so don't make the 
  // target change too fast compared to the PID response or the lag the driver sees will make it hard for him to control. 
  final int FAST_MOTION_FACTOR_ARM    = 5;     // 20 sec / 5 = 4 seconds
  final int FAST_MOTION_FACTOR_WRIST  = 8;     //increase to make faster
  //define the range of digital counts of the pots
  final double ARM_DIGITAL_RANGE   = 1000.0; //range is -1000 to +1000
  final double WRIST_DIGITAL_RANGE = 1000.0;
  
  //*************************************************************************************************************************
  //values with comments starting with @@@ must be determied empirically for each robot each time the pot set screw is tightned   
  //*************************************************************************************************************************
  //----- target limits so we dont over rotate ---------------------------
  //These are in units of analog to digital counts returned from the pot sensor
  final double ARM_POT_FULL_UP       =  30;   //@@@ - record from print output
  final double ARM_POT_FULL_DOWN     = -860;  //@@@ 
  final double ARM_POT_STRAIGHT_OUT  = -440;  //@@@ 
  final double ARM_POT_INITIAL       = ARM_POT_FULL_DOWN;     //@@@
  final double ARM_SAFETY_DOWN       = -1.1;  //@@@ - below this is considered a severed pot wire
  final double ARM_SAFETY_UP         =  1.1;  //@@@ - above this is considered a severed pot wire

  final double WRIST_POT_FULL_UP     =   980;  //@@@ 
  final double WRIST_POT_FULL_DOWN   =  -910;  //@@@ 
  final double WRIST_POT_STRAIGHT_OUT=  -190;  //@@@
  final double WRIST_POT_INITIAL     =  WRIST_POT_FULL_UP; //@@@
  final double WRIST_SAFETY_DOWN     =  -1.01;  //@@@ - below this is considered a severed pot wire
  final double WRIST_SAFETY_UP       =   1.01;  //@@@ - above this is considered a severed pot wire

  //The hand can point down to pick up a ball or straight up to place a disk.
  //The hand should never point further back than straight up to avoid tipping over backwards.
  //The sum of the targets of the wrist and arm will be limited to the following constant.
  //This will insure the hand is never pointed back more than straight up.   
  //As the arm and wrist move angles are traded as to not exceed the limit.
  //Add some margin past this point to allow for inaccuracys.
  //To determine value raise arm manually to about straight out and determine value that allows
  // the wrist target to point the hand a bit past straight up. Try again with arm all the way up
  final double HAND_UNDER_EXTEND_LIMIT =  300;    


  /*------- values needed to calculated the PID feed forward value to compensate for torque caused by the weight of the arm 
  We will ignore affects of the various wrist positions affecting the torque on the arm joint.
  Once the angle of the arm is known and the relative to the arm is known, we can determine the angle of the wrist
    relative to gravity.
  The following link discusses everything that was implemented in our code
   Ignore the inertia stuff in the link... You will need to understand the A cos theta stuff.
   The A value to overcome gravity will be determined empirically (try values until one work well) not calculated 
   https://www.chiefdelphi.com/t/velocity-limiting-pid/164908
   The drive needed to overcome the gravity actually consists of two parts
   1st part: the minimum drive just to get the motor to move with no gravity
   2nd part: the additional drive needed to overcome gravity  
   So... As a refinement to what was dicussed in the link, the following was implemented
   1. Determine drive value M that is 20% under value that just MOVES the motor with the chain removed.  
        Take 80% of the value where the motor just starts to overcome its own friction.
        The reason we take only 80% rather than 100% is to prevent oscillations when the PID out is near zero.
        This should be more clear after understanding the complete feed foreward solution.
   2. Determine the COMBINED drive value, C that overcomes gravity with arm straigt out at 0 degrees
   3. The value for A in the A cos (theta) term is calcualted by the following equation. 
        C = A + M
        A = C - M    (solved for A)
   4. The final drive F is
        when PidOut is positive to raise arm: F = PidOut + M + A cos(theta) 
        when PidOut is negative to lower arm: F = PidOut - M + A cos(theta) 
   Note: M and A cos(theta) are feed forward terms in PID lingo.     
   */
  final double ARM_ANGLE_FULL_UP    =  60; //@@@ degrees up from straight out  - measure with inclinometer
  final double ARM_ANGLE_FULL_DOWN  =  50; //@@@ degrees down from straight out
  final double ARM_DRIVE_M          = 0.07;//@@@ determine by trial and error so arm does not bounce around +/- pid out
  final double ARM_DRIVE_C          = 0.22;//@@@ determine by trial and error so arm does not bounce around +/- pid out
  final double WRIST_ANGLE_FULL_UP  =  135;//@@@ degrees up relative to arm    
  final double WRIST_ANGLE_FULL_DOWN=   45;//@@@ degrees down relative to arm  
  final double WRIST_DRIVE_M        = 0.10;//@@@ see comments above how to determine
  final double WRIST_DRIVE_C        = 0.28;//@@@ see comments above how to determine
  
  //------- poses (There are only a handfull so an array would add more complication than the benifit.) --------
  final double ARM_POSE_0       =  -450; //pick up ball from ground
  final double WRIST_ARM_POSE_0 =  -625;
  //-----------------------------------------------------------------
  final double ARM_POSE_1       = -805; //hatch level 1
  final double WRIST_ARM_POSE_1 =  955;
  //-----------------------------------------------------------------
  final double ARM_POSE_2       = -425; //hatch level 2
  final double WRIST_ARM_POSE_2 =  605;
  //-----------------------------------------------------------------
  final double ARM_POSE_3       =  5;//hatch level 3
  final double WRIST_ARM_POSE_3 =  210;  
  //-----------------------------------------------------------------
  private int poseSelection             = 1;    //initial pose
  final private int POSE_HIGHEST_DEFINED= 3;    //poses 0 to 3 are defined so far
  //----- When cycling poses, both buttons must be released for 1 sec before a subsequant pose change
  int poseLockOutTimer = 0;
  int POSE_LOCKOUT_TIME = 30; //30 * 20mS = 600mS
  boolean acceptButtonRequest = false;
   
  //Declare all possible objects here and instantiate what is needed for each bot in constructor
  //Peanut Bot

  WPI_VictorSPX armLeft_peanut; 
  WPI_TalonSRX  armRight_peanut; 
      //no wrist connected
  //Bag Bot 
  WPI_TalonSRX    armLeft_bag;	
  WPI_TalonSRX    armRight_bag;
  // 2nd Bot  
  WPI_TalonSRX    armLeft_2nd;	
  WPI_TalonSRX    armRight_2nd;
  // general
  HardwareMap hMap;
  SpeedController armGroup;
  Spark           wrist;      
  AnalogPotentiometer potArm;
  AnalogPotentiometer potWrist;  

  double armPositionCurrent      = 0;
  double armPositionTarget       = 0;
  double wristPositionCurrent    = 0; 
  double wristPositionTarget     = 0;
    
  
  //TODO: PID tuning
  //Back flip issue - Likely cause:
  // The arm I value was 0.001. Every 20mS, the I term builds by 0.001 * error
  // This can build to large values (even larger than P term) when the bot is sitting especially with
  //  motors disconnected.
  // This I term build up remains even after disabling and enabling teleop mode.
  //Temp solution implemented the sunday before the competition
  // I values set to 0
  // I build up set to 0 when PID crosses 0, also in teleop init
  //This should prevent the back flips but not get all the way to the target with no I term and only P term
  //Dont change I values from 0 until the following steps are done (positive up requirement)
  // 1. pot wiring: black ground, red positive, rotation in up direction increases 
  // 2. remove all negative signs that were put in to bandaid the code in process PID
  // 3. full down pot value near -1.0, for arm and wrist
  // 4. full up pot value near +1.0 for arm and about 0 for the wrist
  // 5. when target position is more positive than current position, PID out is positive
  MiniPID pidArm;
  final double P_ARM = 0.85;
  final double I_ARM = 0.01;
  final double D_ARM = 0.0;
  MiniPID pidWrist;
  final double P_WRIST = 0.75;
  final double I_WRIST = 0.006;
  final double D_WRIST = 0.0;
  
  int printCounter = 0; //used to reduce the print frequency
  //-------------------------------------
  OurBots selectedBot_local; //copy so we can pass in one in constructor
  
  public ArmWrist(OurBots selectedBot)//constructor
  {
    hMap = new HardwareMap();
    selectedBot_local = selectedBot; //copy to be used by other methods of this class
    //arm PID
    pidArm = new MiniPID(P_ARM,I_ARM,D_ARM);
    pidArm.setSetpoint(0.0);            //center of travel
    pidArm.setMaxIOutput(0.4);    //let P, M and C not I do most of the work to prevent violent movements
    //pidArm.setDirection(true);  //true is reversed
    //wrist PID
    pidWrist = new MiniPID(P_WRIST,I_WRIST,D_WRIST);
    pidWrist.setSetpoint(0.0);    
    pidWrist.setMaxIOutput(0.3);  //let P, M and C not I do most of the work to prevent violent movements   
    //pidArm.setDirection(true);  //true is reversed
    resetPids();                  //remove any I term build up from last time we used the PID
    // pots
    potArm   = new AnalogPotentiometer(hMap.potArm,   2 * ARM_DIGITAL_RANGE,   0); //channel, range, offset; [0 to 2000] will map to [-1.0 to +1.0] when read
    potWrist = new AnalogPotentiometer(hMap.potWrist, 2 * WRIST_DIGITAL_RANGE, 0); 

    switch(selectedBot)
    {
      case PEANUT:
        armLeft_peanut	= new WPI_VictorSPX(0); //retain bag bot ID
        armRight_peanut	= new WPI_TalonSRX(4);  //retain bag bot ID
        armGroup = new SpeedControllerGroup(armLeft_peanut, armRight_peanut); 
        armGroup.set(0);
        //no wrist
        break;
      case WM2019_2ND:
        armLeft_2nd = new WPI_TalonSRX(4);	
        armRight_2nd = new WPI_TalonSRX(5);
        armRight_2nd.setInverted(true); //Left arm motor drives arm up when wires are red to red.
                                        //Right arm motor drives in reverse from left, so invert
        wrist = new Spark(5);
        wrist.setInverted(true);        //invert so positive is up 
        armGroup = new SpeedControllerGroup(armLeft_2nd, armRight_2nd); 
        armGroup.set(0);
        wrist.set(0);     
        break;
      case WM2019_BAG:
      default:
        armLeft_bag = new WPI_TalonSRX(4);	
        armRight_bag = new WPI_TalonSRX(5);
        armRight_bag.setInverted(true); //Left arm motor drives arm up when wires are red to red.
                                        //Right arm motor drives in reverse from left, so invert
        wrist = new Spark(5);      
        wrist.setInverted(true);        //invert so positive is up
        armGroup = new SpeedControllerGroup(armLeft_bag, armRight_bag); 
        armGroup.set(0);
        wrist.set(0);
        break;
    }
  }

  public void upDownManualArmWrist(boolean up, boolean down)
  {
    upDownManualArm(up, down);
    upDownManualWrist(up, down);
  }
  public void upDownManualArm(boolean up, boolean down)
  { 
    //This function only sets the target values. The processPIDs below drives the motors
    //---- process the arm ------------------
    if(up && ((wristPositionTarget + armPositionTarget) >= HAND_UNDER_EXTEND_LIMIT))
    {
      return;// dont allow hand to point further back than straight up
    }
    if(up && armPositionTarget < ARM_POT_FULL_UP)
    {
      armPositionTarget += FAST_MOTION_FACTOR_ARM;
    }
    else
    {
      if(down && armPositionTarget > ARM_POT_FULL_DOWN)
      {
         armPositionTarget -= FAST_MOTION_FACTOR_ARM;
      }
    }
  }

  public void upDownManualWrist(boolean up, boolean down)
  { 
    //System.out.println("in manual wrist method " + up + " " + down + " " + wristPositionTarget);
    //This function only sets the target values. The processPIDs below drives the motors
    //---- process the wrist ------------------ 
    if(up && ((wristPositionTarget + armPositionTarget) >= HAND_UNDER_EXTEND_LIMIT))
    {
      return;// dont allow hand to point further back than straight up
    }
    if(up && wristPositionTarget < WRIST_POT_FULL_UP)
    {
      wristPositionTarget += FAST_MOTION_FACTOR_WRIST;
    }
    else
    {
      if(down && wristPositionTarget > WRIST_POT_FULL_DOWN)
      {
        wristPositionTarget -= FAST_MOTION_FACTOR_WRIST;
      }
    }
  }

  public void upDownCycle(boolean up, boolean down)
  {
    //Buttons must be released for 1 sec before subsequant press takes effect.
    //(debounce the switches)
    if(!up && !down)
    {
      if(poseLockOutTimer > 0)
      {
        poseLockOutTimer--;
      }
      else
      {
        acceptButtonRequest = true;
      }
    }
    //---- process the pose selection-----------------
    if(up && acceptButtonRequest && poseSelection < POSE_HIGHEST_DEFINED)
    { 
      poseSelection++;
      resetPids(); //remove I built up during last pose
      poseLockOutTimer = POSE_LOCKOUT_TIME;
      acceptButtonRequest = false;
      System.out.printf("UP new pose %d\n", poseSelection);
    }
    else
    {
      if(down && acceptButtonRequest && poseSelection > 0)
      {
         poseSelection--;
         resetPids(); //remove I built up during last pose
         poseLockOutTimer = POSE_LOCKOUT_TIME;
         acceptButtonRequest=false;
         System.out.printf("DOWN new pose %d\n", poseSelection);
      }
    }
    
    //if no button press, return before changing targets so manual function can work.
    if(up == false && down == false)
    {
      return;
    }
    //strike a pose, come on vogue
    switch (poseSelection)
    {
      case 0:
        wristPositionTarget = WRIST_ARM_POSE_0;
        armPositionTarget = ARM_POSE_0;
        break;
      case 1:
        wristPositionTarget = WRIST_ARM_POSE_1;
        armPositionTarget = ARM_POSE_1;
        break;
      case 2:
        wristPositionTarget = WRIST_ARM_POSE_2;
        armPositionTarget = ARM_POSE_2;
        break;
      case 3:
        wristPositionTarget = WRIST_ARM_POSE_3;
        armPositionTarget   = ARM_POSE_3;
        break;
      default:
        System.out.printf("*** Logic Error *** bad pose bozo - please fix your code");
        break;
    }
  }
  
  public void resetPids()
  {
    pidArm.reset();
    pidWrist.reset();
  }

  public void processPIDsAndDriveMotors()
  {
    //----- Read the pots, cycle the PIDs and store the PID outputs  -----------------------------------------------------
    armPositionCurrent   = potArm.get()/ARM_DIGITAL_RANGE     - 1.0;  //map [0 to 2.0] to [-1.0 to 1.0]
    wristPositionCurrent = potWrist.get()/WRIST_DIGITAL_RANGE - 1.0; 
    //For each PID cycle, pass in the current and target positions. 
    //The needed drive to eliminate error is returned from the PID.
    //Simple as that :)
    double pidOutputArm   = pidArm.getOutput(armPositionCurrent, armPositionTarget/ARM_DIGITAL_RANGE); //output range is -1000 to +1000
    double pidOutputWrist = pidWrist.getOutput(wristPositionCurrent, wristPositionTarget/WRIST_DIGITAL_RANGE);
   
    //The variable torque caused by the weight of the arm and wrist makes for bad PID behavior so we need to add
    // a feed forward term which is an offset that is dependant of the angles of the joint.
    // 1st determine the joint angles 
    double armAngle   = calculateJointAngle(armPositionCurrent, 
                                            ARM_DIGITAL_RANGE, 
                                            ARM_ANGLE_FULL_UP, 
                                            ARM_ANGLE_FULL_DOWN, 
                                            ARM_POT_FULL_UP, 
                                            ARM_POT_FULL_DOWN,
                                            ARM_POT_STRAIGHT_OUT); 
  double wristAngle   = calculateJointAngle(wristPositionCurrent, 
                                            WRIST_DIGITAL_RANGE, 
                                            WRIST_ANGLE_FULL_UP, 
                                            WRIST_ANGLE_FULL_DOWN, 
                                            WRIST_POT_FULL_UP, 
                                            WRIST_POT_FULL_DOWN,
                                            WRIST_POT_STRAIGHT_OUT);
    double wristAngleRealtiveToGravity = wristAngle + armAngle;  
    // 2nd - Now that we know the angles, the feed forward term is pretty simple as follows:
    //       Consider the arm straight out at zero degrees: That would be full torque.
    //       Then consider the arm straight up at 90 degrees: That would be zero torque.
    //       This a cos function.     
    
    //C = A + M    See comments is top of this file to understand feed forward terms 
    //A = C - M    (solved for A)
    double arm_A   = ARM_DRIVE_C - ARM_DRIVE_M; 
    double armACosTheta = arm_A * Math.cos(Math.toRadians(armAngle));
    double wrist_A = WRIST_DRIVE_C - WRIST_DRIVE_M; 
    double wristACosTheta = wrist_A * Math.cos(Math.toRadians(wristAngleRealtiveToGravity));
    //----- Print the results ---------------------------------------------------------------------------------------------
    if(printCounter%10 == 0)//print every 20*10 = 200mS
    {
      System.out.printf("C:T:P:A:F Arm %.2f : %.0f : %.3f : %.2f : %.3f  Wrist %.2f : %.0f : %.3f : %.2f : %.3f\n", 
                                              armPositionCurrent,
                                              armPositionTarget,
                                              pidOutputArm,
                                              armAngle,
                                              armACosTheta,  
                                              wristPositionCurrent,
                                              wristPositionTarget,
                                              pidOutputWrist,
                                              wristAngle,
                                              wristACosTheta);
    }
    printCounter++;
   
    //-------------------------------------------------------------------------
    //Now that we have the drive levels, drive the motors.
    //Another case statement is needed as each bot is different.
    switch(selectedBot_local)
    {
      case PEANUT:
        setArmWithSafetyCheck(pidOutputArm, armPositionCurrent);
        //Lowly peanut has no wrist
        break;
      case WM2019_BAG:
      case WM2019_2ND:
        //See comments at top of this file to understand feed forward terms
        //When PidOut is positive to raise arm: F = PidOut + M + A cos(theta) 
        //When PidOut is negative to lower arm: F = PidOut - M + A cos(theta) 
        //---- determine final drive F for the arm ---------------------------
        double armFinalDrive = 0; 
        if(pidOutputArm > 0) 
        {
          armFinalDrive = pidOutputArm + ARM_DRIVE_M + armACosTheta;
          //System.out.printf("ArmFinal+M %.2f %.2f, %.2f %.2f\n",armFinalDrive, pidOutputArm, ARM_DRIVE_M, armACosTheta);
        }
        else
        {
          armFinalDrive = pidOutputArm - ARM_DRIVE_M + armACosTheta;
          //System.out.printf("ArmFinal-M %.2f %.2f, %.2f %.2f\n",armFinalDrive, pidOutputArm, ARM_DRIVE_M, armACosTheta);
        }
        //---- again for the wrist ----
        double wristFinalDrive = 0; 
        if(pidOutputWrist > 0)
        {
          wristFinalDrive = pidOutputWrist + WRIST_DRIVE_M + wristACosTheta;
        }
        else
        {
          wristFinalDrive = pidOutputWrist - WRIST_DRIVE_M + wristACosTheta;
        }
        setArmWithSafetyCheck  (armFinalDrive,   armPositionCurrent);
        setWristWithSafetyCheck(wristFinalDrive, wristPositionCurrent);
        break;
    }
  }

  /** This method checks to make sure the arm pot sensor wire is not broken then drives the motors */
  private void setArmWithSafetyCheck(double driveValue, double potValueSafetyCheckValue)
  {
    if(potValueSafetyCheckValue < ARM_SAFETY_UP && potValueSafetyCheckValue > ARM_SAFETY_DOWN)
    {
      armGroup.set(driveValue); 
      //System.out.printf("final arm drive is %.2f\n", driveValue);
    }
    else
    {
      armGroup.set(0);
      System.out.println("*Error* Check for broken Arm pot or wire or excessive electrical noise");
    }
  }

  /** This method checks to makes sure the arm pot sensor wire is not broken then drives the motors */
  private void setWristWithSafetyCheck(double driveValue, double potValueSafetyCheckValue)
  {
    if(potValueSafetyCheckValue > WRIST_SAFETY_DOWN && potValueSafetyCheckValue < WRIST_SAFETY_UP)
    {
     // System.out.printf("final wrist drive is %.2f\n", driveValue);
     wrist.set(driveValue); //negative drive rasies wrist so ***negate it here ONLY***
    }
    else
    {
      wrist.set(0);
      System.out.println("*Error* Check for broken Wrist pot or wire or excessive electrical noise");
    }
  }

  /** This method calculates the angle in degrees of the joint with 0 being horizontal.
   *  It is simply the full angle times a ratio of pot values.
   *  Positive angle returned when joint is up   from straight out.
   *  Negative angle returned when joint is down from straight out
   */
  private double calculateJointAngle( double potPosition, 
                                      double potDigitalRange, 
                                      double angleFullUp, 
                                      double angleFullDown, 
                                      double potFullUp,
                                      double potFullDown,
                                      double potStraightOut)
  {
    double returnAngle  = 0;
    //potDigitalRange scales from -1 to +1 back to -1000 to 1000
  /*  System.out.printf("A: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", 
                                                            potPosition, 
                                                            potDigitalRange, 
                                                            angleFullUp, 
                                                            angleFullDown, 
                                                            potFullUp,
                                                            potFullDown,
                                                            potStraightOut);
  
    
    */                                                        if(potDigitalRange * potPosition > potStraightOut)
    {
      //The positive angle above stright out 
      returnAngle = angleFullUp * (potDigitalRange * potPosition - potStraightOut) / (potFullUp - potStraightOut);
    }
    else
    {
      //The negative angle below stright out 
      returnAngle = -angleFullDown * (potStraightOut - potDigitalRange * potPosition) / (potStraightOut - potFullDown);
    }
    return returnAngle;
  }
}
