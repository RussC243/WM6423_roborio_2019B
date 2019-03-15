/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Robot.OurBots;
import edu.wpi.first.wpilibj.*;

/**
 * Add your docs here.
 */
public class ArmWrist {
  //at 20mS per update, changing target from zero to full up or full down would take 1000*0.020 = 20 sec
  //So to speed the motion we will change target by the factor every 20mS.
  final int FAST_MOTION_FACTOR  = 20;     //20 sec divided by 10 = 2 sec for full travel up or down from center position
  //define the range of digital counts of the pots
  final double ARM_FULL         = 1000.0; //range is -1000 to +1000
  final double WRIST_FULL       = 1000.0;
  
  //*************************************************************************************************************************
  //values with comments starting with @@@ must be determied empirically for each robot each time the pot set screw is tightned   
  //*************************************************************************************************************************
  //----- target limits so we dont over rotate ---------------------------
  //These are in units of analog to digital counts returned from the pot sensor
  final double ARM_POT_FULL_UP       =  150;  //@@@ - record from print output
  final double ARM_POT_FULL_DOWN     = -650;  //@@@ 
  final double ARM_POT_STRAIGHT_OUT  = -200;  //@@@ 
  
  final double WRIST_POT_FULL_UP     =  200;  //@@@ 
  final double WRIST_POT_FULL_DOWN   = -200;  //@@@ 
  final double WRIST_POT_STRAIGHT_OUT=   -0;  //@@@ 
  
  //------- values needed to calculated the PID feed forward value to compensate for torque caused by the weight of the arm 
  //We will ignore affects of the various wrist positions affecting the torque on the arm joint.
  //Once the angle of the arm is known and the relative to the arm is known, we can determine the angle of the wrist
  //  relative to gravity.
  final double ARM_ANGLE_FULL_UP    = 50; //@@@ degrees up from straight out  - measure with inclinometer
  final double ARM_ANGLE_FULL_DOWN  = 40; //@@@ degrees down from straight out
  final double ARM_NEEDED_COMPENSATION_STRAIGHT_OUT = 0.2;  //@@@ measure by looking at print of PID out value with no compensation
  final double WRIST_ANGLE_FULL_UP  = 90; //@@@ degrees up relative to arm    
  final double WRIST_ANGLE_FULL_DOWN= 40; //@@@ degrees down relative to arm  
  final double WRIST_NEEDED_COMPENSATION_STRAIGHT_OUT = 0.2;//@@@ measure by looking at print of PID out value with no compensation
  //------- poses (There are only a hadfull so an array would add more complication than the benifit.) --------
  final double ARM_POSE_0       = -500; //pick up ball from ground
  final double WRIST_ARM_POSE_0 =  200;
  final double ARM_POSE_1       = -300; //hatch level 1
  final double WRIST_ARM_POSE_1 =  100;
  final double ARM_POSE_2       = -100; //hatch level 2
  final double WRIST_ARM_POSE_2 =    0;
  final double ARM_POSE_3       =  100; //hatch level 3
  final double WRIST_ARM_POSE_3 = -100;
  private int poseSelection             = 1;    //initial pose
  final private int POSE_HIGHEST_DEFINED= 3;    //poses 0 to 3 are defined so far

  //Observed values for the pots for a given position 
  //Static Values (after I term settles)
  //Values for the WM2019_2nd bot
  //pot   : target: pid out  
  //0.2   : 160   : -0.2  full up
  //-0.73 : -659  : -0.2  full down
  
  //Declare all possible objects here and instantiate what is needed for each bot in constructor
  //Peanut Bot
  HardwareMap hMap;
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
  SpeedController armGroup;
  Spark           wrist;      
  AnalogPotentiometer potArm;
  AnalogPotentiometer potWrist;  

  double armPositionCurrent      = 0;
  double armPositionTarget       = 0;
  double wristPositionCurrent    = 0; 
  double wristPositionTarget     = 0;
    
  MiniPID pidArm;
  final double P_ARM = 0.99;
  final double I_ARM = 0.005;
  final double D_ARM = 0.0;
  MiniPID pidWrist;
  final double P_WRIST = 0.55;
  final double I_WRIST = 0.0;
  final double D_WRIST = 0.0;
  
  int printCounter = 0; //used to reduce the print frequency
  OurBots selectedBot_local; //copy so we can pass in one in constructor
  
  public ArmWrist(OurBots selectedBot)//constructor
  {
    hMap = new HardwareMap();
    selectedBot_local = selectedBot; //copy to be used by other methods of this class
    //arm PID
    pidArm = new MiniPID(P_ARM,I_ARM,D_ARM);
    pidArm.setSetpoint(0.0);            //center of travel
    //pidArm.setSetpointRange(ARM_FULL);  //MiniPID class sets range to +/- the value set here 
    //pidArm.setOutputLimits(ARM_DOWN_LIMIT,ARM_UP_LIMIT);
    pidArm.setDirection(true); //true is reversed
    pidArm.reset();            //remove any I term build up from last time we used the PID
    
    //wrist PID
    pidWrist = new MiniPID(P_WRIST,I_WRIST,D_WRIST);
    pidWrist.setSetpoint(0.0);      
    pidArm.setDirection(true); //true is reversed
    pidWrist.reset();  
    // pots
    potArm   = new AnalogPotentiometer(0, 2 * ARM_FULL, 0); //channel, range, offset; [0 to 2000] will map to [-1.0 to +1.0] when read
    potWrist = new AnalogPotentiometer(1, 2 * WRIST_FULL, 0); 

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
        wrist = new Spark(5); 
        armGroup = new SpeedControllerGroup(armLeft_2nd, armRight_2nd); 
        armGroup.set(0);
        wrist.set(0);     
        break;
      case WM2019_BAG:
      default:
        armLeft_bag = new WPI_TalonSRX(4);	
        armRight_bag = new WPI_TalonSRX(5);
        wrist = new Spark(5);      
        armGroup = new SpeedControllerGroup(armLeft_2nd, armRight_2nd); 
        armGroup.set(0);
        wrist.set(0);
        break;
    }
  }
  public void upDownManual(boolean up, boolean down)
  {
    //This function only sets the target values. The processPIDs below drives the motors
    //---- process the arm ------------------
    if(up && armPositionTarget < ARM_POT_FULL_UP)
    {
      armPositionTarget += FAST_MOTION_FACTOR;
    }
    else
    {
      if(down && armPositionTarget > ARM_POT_FULL_DOWN)
      {
         armPositionTarget -= FAST_MOTION_FACTOR;
      }
    }
    //---- process the wrist ------------------
    if(up && wristPositionTarget < WRIST_POT_FULL_UP)
    {
      wristPositionTarget += FAST_MOTION_FACTOR;
    }
    else
    {
      if(down && wristPositionTarget > WRIST_POT_FULL_DOWN)
      {
        wristPositionTarget -= FAST_MOTION_FACTOR;
      }
    }
  }

  public void upDownCycle(boolean up, boolean down)
  {
    //---- process the pose selection-----------------
    if(up && poseSelection < POSE_HIGHEST_DEFINED)
    {
      poseSelection++;
    }
    else
    {
      if(down && poseSelection > 0)
      {
         poseSelection--;
      }
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
        System.out.printf("*** Logic Error *** bad pose selection - please fix your code");
        break;
    }

    //---- process the wrist ------------------
    if(up && wristPositionTarget < WRIST_POT_FULL_UP)
    {
      wristPositionTarget += FAST_MOTION_FACTOR;
    }
    else
    {
      if(down && wristPositionTarget > WRIST_POT_FULL_DOWN)
      {
        wristPositionTarget -= FAST_MOTION_FACTOR;
      }
    }
  }
  
  public void processPIDs()
  {
    //----- read the pots, cycle the PIDs and store the PID outputs  -----------------------------------------------------
    armPositionCurrent   = potArm.get()/ARM_FULL - 1.0;  //map [0 to 2.0] to [-1.0 to 1.0]
    wristPositionCurrent = potWrist.get()/WRIST_FULL - 1.0; 
    //For each PID cycle, pass in the current and target positions. 
    //The needed drive to eliminate error is returned from the PID.
    //Simple as that :)
    //sensor target
    double pidOutputArm   = -pidArm.getOutput(armPositionCurrent, armPositionTarget/ARM_FULL); //output range is -1000 to +1000
    double pidOutputWrist = -pidWrist.getOutput(wristPositionCurrent, wristPositionTarget/WRIST_FULL);
    //----------------------------------------------------------
    //calcualte the feed forward terms to compensate for torques caused by weight of arm and wrist------------------------
    double feedForwardArm   = 0; 
    double feedForwardWrist = 0;
    double armAngle         = 0;
    double wristAngle       = 0; 
    //The ARM_FULL term is needed to scale from -1 to +1 back to -1000 to 1000
    if(ARM_FULL * armPositionCurrent > ARM_POT_STRAIGHT_OUT)
    {
      //The angle above stright out  = full up angle * (measured pot value - straight out pot value) /(full up pot value - straight out pot value) 
      armAngle = ARM_ANGLE_FULL_UP * (ARM_FULL * armPositionCurrent - ARM_POT_STRAIGHT_OUT) / (ARM_POT_FULL_UP - ARM_POT_STRAIGHT_OUT);
    }
    else
    {
      //The angle below stright out  = full down angle * (measured pot value- full down pot value) /(straight out pot value - full down pot value) 
      armAngle = ARM_ANGLE_FULL_DOWN * (ARM_FULL * armPositionCurrent - ARM_POT_FULL_DOWN) / (ARM_POT_STRAIGHT_OUT - ARM_POT_FULL_DOWN);
    }
    //The ARM_FULL term is needed to scale from -1 to +1 back to -1000 to 1000
    if(WRIST_FULL * wristPositionCurrent > WRIST_POT_STRAIGHT_OUT)
    {
      //The angle above stright out  = full up angle * (measured pot value - straight out pot value) /(full up pot value - straight out pot value) 
      //The WRIST_FULL term is needed to scale from -1 to +1 back to -1000 to 1000
      wristAngle = WRIST_ANGLE_FULL_UP * (WRIST_FULL * wristPositionCurrent - WRIST_POT_STRAIGHT_OUT) / (WRIST_POT_FULL_UP - WRIST_POT_STRAIGHT_OUT);
    }
    else
    {
      //The angle below stright out  = full down angle * (measured pot value- full down pot value) /(straight out pot value - full down pot value) 
      wristAngle = WRIST_ANGLE_FULL_DOWN * (WRIST_FULL * armPositionCurrent - WRIST_POT_FULL_DOWN) / (WRIST_POT_STRAIGHT_OUT - WRIST_POT_FULL_DOWN);
    }
    //------- now that we know the arm and wrist angles, calcualte the wrist angle relative to gravity
    double wristAngleRealtiveToGravity = 0;
    //The ARM_FULL term is needed to scale from -1 to +1 back to -1000 to 1000
    if(ARM_FULL * armPositionCurrent > ARM_POT_STRAIGHT_OUT)
    {
      wristAngleRealtiveToGravity  = wristAngle + armAngle;
    }
    else
    {
      wristAngleRealtiveToGravity = wristAngle - armAngle;
    }
    //------ Now that we know the arma dn wrist angles relative to gravity, determine the feed forweard terms ---------    
    feedForwardArm   = ARM_NEEDED_COMPENSATION_STRAIGHT_OUT * Math.cos(Math.toRadians(armAngle));
    feedForwardWrist = WRIST_NEEDED_COMPENSATION_STRAIGHT_OUT * Math.cos(Math.toRadians(wristAngleRealtiveToGravity));

    //----- print the results ---------------------------------------------------------------------------------------------
    if(printCounter%10 == 0)//print every 20*10 = 200mS
    {
      System.out.printf("C:T:P:A:F %.2f : %.2f : %.3f %.2f %.3f :  %.2f : %.2f : %.3f %.2f %.3f\n", 
                                              armPositionCurrent,
                                              armPositionTarget,
                                              pidOutputArm,
                                              armAngle,
                                              feedForwardArm,  
                                              wristPositionCurrent,
                                              wristPositionTarget,
                                              pidOutputWrist,
                                              wristAngle,
                                              feedForwardWrist);
    }
    printCounter++;
   
    //-------------------------------------------------------------------------
    //Now that we have the drive levels, drive the motors.
    //Another case statement is needed as each bot is different.
    switch(selectedBot_local)
    {
      case PEANUT:
        armGroup.set(pidOutputArm);//TODO add feedForwardArm term aftr it is printing correctly
        //Lowly peanut has no wrist
        break;
      case WM2019_BAG:
      case WM2019_2ND:
        armGroup.set(pidOutputArm);//TODO add feedForwardArm term aftr it is printing correctly
        wrist.set(pidOutputWrist);     
        break;
    }
  }
}
