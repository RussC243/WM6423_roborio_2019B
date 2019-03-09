/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot.OurBots;
import edu.wpi.first.wpilibj.*;

/**
 * Add your docs here.
*/
public class DriveTrain 
{
  //peanut bot uses spark speed controlers
  Spark           left_front_2nd_and_peanut = new Spark(0);	
  Spark           left_rear_peanut 	= new Spark(1);
  Spark           right_front_peanut= new Spark(2);
  Spark           right_rear_peanut	= new Spark(3);
  HardwareMap hMap = new HardwareMap();

  // bag bot uses victor speed controlers on can bus
  WPI_VictorSPX   left_front_bag	= new WPI_VictorSPX(hMap.driveMotorLeftFront);	
  WPI_VictorSPX   left_rear_bag 	= new WPI_VictorSPX(hMap.driveMotorLeftRear);
  WPI_VictorSPX   right_front_bag	= new WPI_VictorSPX(hMap.driveMotorRightFront);
  WPI_VictorSPX   right_rear_bag	= new WPI_VictorSPX(hMap.driveMotorRightRear);
  
  // 2nd bot uses victor speed controlers on can bus except left fron is on PWM
  WPI_VictorSPX   left_rear_2nd 	= new WPI_VictorSPX(hMap.driveMotorLeftRear);
  WPI_VictorSPX   right_front_2nd	= new WPI_VictorSPX(hMap.driveMotorRightFront);
  WPI_VictorSPX   right_rear_2nd	= new WPI_VictorSPX(hMap.driveMotorRightRear);
  
  SpeedController leftGroup   = new SpeedControllerGroup(left_front_2nd_and_peanut, left_rear_2nd);
  SpeedController rightGroup = new SpeedControllerGroup(right_front_2nd, right_rear_2nd);
   
  DifferentialDrive r_drive = new DifferentialDrive(leftGroup,rightGroup);
  public DriveTrain(OurBots selectedBot)
  {
    // switch(selectedBot)
    // {
    //   case PEANUT:
    //     leftGroup   = new SpeedControllerGroup(left_front_2nd_and_peanut, left_rear_peanut);
    //     righttGroup = new SpeedControllerGroup(right_front_peanut, right_rear_peanut);
    //     break;
    //   case WM2019_2ND:
    //     leftGroup   = new SpeedControllerGroup(left_front_2nd_and_peanut, left_rear_2nd);
    //     righttGroup = new SpeedControllerGroup(right_front_2nd, right_rear_2nd);
    //     break;
    //   case WM2019_BAG:
    //   default:
    //     leftGroup   = new SpeedControllerGroup(left_front_bag, left_rear_bag);
    //     righttGroup = new SpeedControllerGroup(right_front_bag, right_rear_bag);
    //   break;
    // }
  //  r_drive = new DifferentialDrive(leftGroup,righttGroup);
  //  r_drive.setSafetyEnabled(false);
  }
  public void drive(double left, double right)
  {
     if(Math.abs(left)>0.1 || Math.abs(right)>0.1)
     {
       r_drive.tankDrive(left,right);
     }
     else
     {
       r_drive.tankDrive(0,0);
     }
  }
}
