/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot.OurBots;
import edu.wpi.first.wpilibj.*;

/**
 * Add your docs here.
*/
public class DriveTrain 
{
  //todo: move magic numbers to HardwareMap class
  //HardwareMap hMap = new HardwareMap();
  //Declare all possible objects here and instantiate what is needed for each bot in constructor
  //Peanut Bot
  Spark           driveLeft_peanut;
  Spark           driveRight_peanut;
  //Bag Bot 
  WPI_VictorSPX   driveLeftFront_bag;	
  WPI_VictorSPX   driveLeftRear_bag;
  WPI_VictorSPX   driveRightFront_bag;
  WPI_VictorSPX   driveRightRear_bag;
  // 2nd Bot  
  VictorSP      driveLeftFront_2nd; //this one speed controller is PWM. Code works for drivetrain. 
  WPI_TalonSRX	driveLeftRear_2nd;   
  WPI_TalonSRX	driveRightFront_2nd;
  WPI_TalonSRX	driveRightRear_2nd; 
  // general
  HardwareMap hMap;
  SpeedController leftSpeedGroup; 
  SpeedController rightSpeedGroup;
  DifferentialDrive diffDrive;
  double DRIVE_SCALE = 0.99;
  
  public DriveTrain(OurBots selectedBot)//constructor
  {
    hMap = new HardwareMap();
    switch(selectedBot)
    {
      case PEANUT:
        driveLeft_peanut	= new Spark(0);
        driveRight_peanut	= new Spark(1);
        diffDrive = new DifferentialDrive(driveLeft_peanut,driveRight_peanut);
        break;
      case WM2019_2ND:
        driveLeftFront_2nd  = new VictorSP(hMap.canID_driveMotorLeftFront);
        driveLeftRear_2nd   = new WPI_TalonSRX(hMap.canID_driveMotorLeftRear);
        driveRightFront_2nd = new WPI_TalonSRX(hMap.canID_driveMotorRightFront);
        driveRightRear_2nd  = new WPI_TalonSRX(hMap.canID_driveMotorRightRear);
        leftSpeedGroup = new SpeedControllerGroup(driveLeftFront_2nd, driveLeftRear_2nd);; 
        rightSpeedGroup = new SpeedControllerGroup(driveRightFront_2nd, driveRightRear_2nd);;
        diffDrive = new DifferentialDrive(leftSpeedGroup,rightSpeedGroup);
        break;
      case WM2019_BAG:
      default:
        driveLeftFront_bag	= new WPI_VictorSPX(hMap.canID_driveMotorLeftFront);	
        driveLeftRear_bag 	= new WPI_VictorSPX(hMap.canID_driveMotorLeftRear);
        driveRightFront_bag	= new WPI_VictorSPX(hMap.canID_driveMotorRightFront);
        driveRightRear_bag	= new WPI_VictorSPX(hMap.PDP_driveMotorRightRear);
        leftSpeedGroup = new SpeedControllerGroup(driveLeftFront_bag, driveLeftRear_bag);; 
        rightSpeedGroup = new SpeedControllerGroup(driveRightFront_bag, driveRightRear_bag);;
        diffDrive = new DifferentialDrive(leftSpeedGroup,rightSpeedGroup);
        break;
    }
    diffDrive.setSafetyEnabled(false);
  }
  public void drive(double left, double right)
  {
     if(Math.abs(left)>0.1 || Math.abs(right)>0.1) //don't creep
     {
        diffDrive.tankDrive(left*DRIVE_SCALE,right*DRIVE_SCALE);
     }
     else
     {
        diffDrive.tankDrive(0,0);
     }
  }
}
