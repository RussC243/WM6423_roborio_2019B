/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Spark;
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
  WPI_VictorSPX   driveLeft_peanut;
  WPI_TalonSRX    driveRight_peanut;
  //Bag Bot 
  WPI_VictorSPX   driveLeftFront_bag;	
  WPI_VictorSPX   driveLeftRear_bag;
  WPI_VictorSPX   driveRightFront_bag;
  WPI_VictorSPX   driveRightRear_bag;
  // 2nd Bot  
  Spark         driveLeftFront_2nd; //Use Spark for Victor SP as generic PWM object
  WPI_TalonSRX	driveLeftRear_2nd;
  WPI_TalonSRX	driveRightFront_2nd;
  WPI_TalonSRX	driveRightRear_2nd;
  // general
  SpeedController leftSpeedGroup; 
  SpeedController rightSpeedGroup;
  DifferentialDrive diffDrive;
  
  public DriveTrain(OurBots selectedBot)
  {
    switch(selectedBot)
    {
      case PEANUT:
        driveLeft_peanut	= new WPI_VictorSPX(0); //retain bag bot ID
        driveRight_peanut	= new WPI_TalonSRX(4);  //retain bag bot ID
        diffDrive = new DifferentialDrive(driveLeft_peanut,driveRight_peanut);
        break;
      case WM2019_2ND:
        driveLeftFront_2nd  = new Spark(0);//Used Spark for Victor SP as generic PWM object
        driveLeftRear_2nd   = new WPI_TalonSRX(1);
        driveRightFront_2nd = new WPI_TalonSRX(2);
        driveRightRear_2nd  = new WPI_TalonSRX(3);
        leftSpeedGroup = new SpeedControllerGroup(driveLeftFront_2nd, driveLeftRear_2nd);; 
        rightSpeedGroup = new SpeedControllerGroup(driveRightFront_2nd, driveRightRear_2nd);;
        diffDrive = new DifferentialDrive(leftSpeedGroup,rightSpeedGroup);
        break;
      case WM2019_BAG:
      default:
        driveLeftFront_bag	= new WPI_VictorSPX(0);	
        driveLeftRear_bag 	= new WPI_VictorSPX(1);
        driveRightFront_bag	= new WPI_VictorSPX(2);
        driveRightRear_bag	= new WPI_VictorSPX(3);
        leftSpeedGroup = new SpeedControllerGroup(driveLeftFront_bag, driveLeftRear_bag);; 
        rightSpeedGroup = new SpeedControllerGroup(driveRightFront_bag, driveRightRear_bag);;
        diffDrive = new DifferentialDrive(leftSpeedGroup,rightSpeedGroup);
        break;
    }
    diffDrive.setSafetyEnabled(false);
  }
  public void drive(double left, double right)
  {
     if(Math.abs(left)>0.1 || Math.abs(right)>0.1)
     {
        diffDrive.tankDrive(left,right);
     }
     else
     {
        diffDrive.tankDrive(0,0);
     }
  }
}
