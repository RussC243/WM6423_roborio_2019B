/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class HardwareMap {
    //Drive motors are mixed: Sparks, Talons on CAN bus and one talon on PWM
  public  final int driveMotorLeftFront 	= 0 ; //can ID 0 on bag bot, PWM 0 on peanut and 2nd bots
  public  final int driveMotorLeftRear 	 = 1;  //can ID 1 on bag and 2nd, PWM 1 on peanut and
  public  final int driveMotorRightFront 	= 2;  //can ID 2 on bag and 2nd bots, PWM 2 on peanut
  public  final int driveMotorRightRear 	= 3;  //can ID 3 on bag and 2nd bots, PWM 3 on peanut
  //Arm motors: Talon SRX  
  public  final int armMotorLeft          = 4;  //can ID 4
  public  final int armMotorRight         = 5;  //can ID 5
  //Intake motor: Spark PWM  
  public  final int intakeMotor           = 4; //next PWM is at 4 to be common with peanut 
  //Wrist, Spark PWM. 
  public  final int wristMotor            = 5; //PWM
  //Climb wheels, Spark PWM
  public  final int climbWheelLeft        = 6; //PWM
  public  final int climbWheelRight       = 7; //PWM
  //Pneumatic Solonoids. connected to pneumatic controller
  public  final int pnuematic_front_down  = 0; 
  public  final int pnuematic_front_up    = 1;
  public  final int pnuematic_rear_down   = 2;
  public  final int pnuematic_rear_up     = 3;
  public  final int pnuematic_hatch_pull  = 4;
  public  final int pnuematic_hatch_push  = 5;

  // PCM and PDP
  public  int pcm_can_ID = 6;
  public  int pdp_can_ID = 7; 
  
  // compressors
  public  int pneumaticCompressor = 0 ;

// sensors
  public  int armPot = 0;
  public  int wristPot = 1;
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

}
