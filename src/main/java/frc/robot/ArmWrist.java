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
  //todo: move magic numbers to HardwareMap class
  //HardwareMap hMap = new HardwareMap();
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

  //at 20mS per update, changing target from zero to full up or full down would take 1000*0.020 = 20 sec
  //So to speed the motion we will change target by the factor every 20mS.
  int FAST_MOTION_FACTOR      = 20; //20 sec divided by 10 = 2 sec for full travel up or down from center position
  double ARM_FULL             = 1000.0; //range is -1000 to +1000
  double WRIST_FULL           = 1000.0;
  double armPositionCurrent   = 0;
  double armPositionTarget    = 0;
  double wristPositionCurrent = 0; 
  double wristPositionTarget  = 0;
  double ARM_UP_LIMIT        = 150;//for now stay near center so we don't break the new pots
  double ARM_DOWN_LIMIT      = -650;
  double WRIST_UP_LIMIT      = 300;
  double WRIST_DOWN_LIMIT    = -300;
  //Observed values on WM2019_2nd bot
  //Static Values (after I term settles)
  //pot   : target: pid out  
  //0.2   : 160   : -0.2  full up???
  //-0.73 : -659  : -0.2  full down???
  //  
  
  MiniPID pidArm;
  double P_ARM = 0.99;
  double I_ARM = 0.005;
  double D_ARM = 0.0;
  
  MiniPID pidWrist;
  double P_WRIST = 0.55;
  double I_WRIST = 0.0;
  double D_WRIST = 0.0;
  
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
    if(up && armPositionTarget < ARM_UP_LIMIT)
    {
      armPositionTarget += FAST_MOTION_FACTOR;
    }
    else
    {
      if(down && armPositionTarget > ARM_DOWN_LIMIT)
      {
         armPositionTarget -= FAST_MOTION_FACTOR;
      }
    }
    //---- process the wrist ------------------
    if(up && wristPositionTarget < WRIST_UP_LIMIT)
    {
      wristPositionTarget += FAST_MOTION_FACTOR;
    }
    else
    {
      if(down && wristPositionTarget > WRIST_DOWN_LIMIT)
      {
        wristPositionTarget -= FAST_MOTION_FACTOR;
      }
    }
  }
  public void processPIDs()
  {
    //----------------------------------------------------------
    armPositionCurrent   = potArm.get()/ARM_FULL - 1.0;  //map [0 to 2.0] to [-1.0 to 1.0]
    wristPositionCurrent = potWrist.get()/WRIST_FULL - 1.0; 
    //For each PID cycle, pass in the current and target positions. 
    //The needed drive to eliminate error is returned from the PID.
    //Simple as that :)
    //sensor target
    double pidOutputArm   = -pidArm.getOutput(armPositionCurrent, armPositionTarget/ARM_FULL); //output range is -1000 to +1000
    double pidOutputWrist = -pidWrist.getOutput(wristPositionCurrent, wristPositionTarget/WRIST_FULL);
    if(printCounter%10 == 0)//print every 20*10 = 200mS
    {
      System.out.printf("C:T:P %.2f : %.2f : %.3f  :  %.2f : %.2f : %.4f\n", 
                                              armPositionCurrent,
                                              armPositionTarget,
                                              pidOutputArm,  
                                              wristPositionCurrent,
                                              wristPositionTarget,
                                              pidOutputWrist);
    }
    printCounter++;
   
    //-------------------------------------------------------------------------
    //Now that we have the drive levels, drive the motors.
    //Another case statement is needed as each bot is different.
    switch(selectedBot_local)
    {
      case PEANUT:
        armGroup.set(pidOutputArm);
        //Lowly peanut has no wrist
        break;
      case WM2019_BAG:
      case WM2019_2ND:
        armGroup.set(pidOutputArm);
        wrist.set(pidOutputWrist);     
        break;
    }
  }
}
