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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
    //no arm wrist/arm connected
  //Bag Bot 
  WPI_TalonSRX    armLeft_bag;	
  WPI_TalonSRX    armRight_bag;
  Spark           wrist_bag;      
  // 2nd Bot  
  WPI_TalonSRX    armLeft_2nd;	
  WPI_TalonSRX    armRight_2nd;
  Spark           wrist_2nd;      
  // general
  SpeedController armGroup;
 
  public ArmWrist(OurBots selectedBot)//constructor
  {
    switch(selectedBot)
    {
      case PEANUT:
        //no arm wrist
        break;
      case WM2019_2ND:
        armLeft_2nd = new WPI_TalonSRX(4);	
        armRight_2nd = new WPI_TalonSRX(5);
        wrist_2nd = new Spark(5); 
        armGroup = new SpeedControllerGroup(armLeft_2nd, armRight_2nd); 
        armGroup.set(0);
        wrist_2nd.set(0);     
        break;
      case WM2019_BAG:
      default:
        armLeft_bag = new WPI_TalonSRX(4);	
        armRight_bag = new WPI_TalonSRX(5);
        wrist_bag = new Spark(5);      
        armGroup = new SpeedControllerGroup(armLeft_2nd, armRight_2nd); 
        armGroup.set(0);
        wrist_bag.set(0);
        break;
    }
  }
}

