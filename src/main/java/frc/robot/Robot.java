/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  enum OurBots
  {
    PEANUT, WM2019_BAG, WM2019_2ND //The peanut is Russ' test bot. The bag bot is the one in the bag. The 2nd is the spare bot
  }
  final OurBots selectedBot = OurBots.WM2019_2ND; //set the bot to the one you are working with

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  Joystick        joy      = new Joystick(0);     //popular and generic, IZT brand joystick
  HardwareMap     hMap     = new HardwareMap();   //This defines what inputs and outputs are connectedd to roborio
  DriveTrain      dTrain   = new DriveTrain(selectedBot);
  ArmWrist        armWrist = new ArmWrist(selectedBot);
  Pneumatics      air      = new Pneumatics(selectedBot);
   
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    System.out.printf("teleopInit\n"); 
    armWrist.armPositionTarget = 0.1234;
    armWrist.wristPositionTarget = 0.1234; 
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    linkJoyStickToDrive();
    linkJoyStickToArmWrist();
    linkJoyStickToPneumatics();
    armWrist.processPIDs();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void linkJoyStickToDrive()
  {
    dTrain.drive(joy.getRawAxis(hMap.axisTankDriveLeft),joy.getRawAxis(hMap.axisTankDriveRight)); //left and right veritcal axis
  }
  public void linkJoyStickToArmWrist()
  {
    armWrist.upDownManual(joy.getRawButton(hMap.buttonArmManualUp), joy.getRawButton(hMap.buttonArmManualDown));
    armWrist.upDownCycle( joy.getRawButton(hMap.buttonArmCycleUp), joy.getRawButton(hMap.buttonArmCycleDown)); 
    // armWrist.upDownWristOnly(joy.getPOV());
  
  }
  public void linkJoyStickToPneumatics()
  {
    //---- Control the climb pistons. ------------------
    if(joy.getRawButton(hMap.buttonClimbDown))
    {
      air.frontRetract();
      air.rearRetract();
    }
    if(joy.getRawButton(hMap.buttonClimbUp))
    {
      air.frontExtend();
      air.rearExtend();
    }
    
    //---- Control the hatch pistons. ------------------
    /*TODO: Find unused buttons on controller, add them to map and un-comment this code
    
    if(joy.getRawButton(hMap.buttonHatchPull))
    //{
      air.hatchRetract();
    }
    if(joy.getRawButton(hMap.buttonHatchPush))
    {
      air.hatchExtend();
    }
    */
    //---- Control the drop wheels. ------------------
    /* TODO: find out the trigger axis ID using the driver station and set in the hardare map. 
    Then un-comment this code
    
    if(joy.getRawAxis(hMap.axisLeftTrigger) > 0.2)
    {
      air.driveDropWheels(joy.getRawAxis(hMap.axisLeftTrigger));
    }
    else if(joy.getRawAxis(hMap.axisRightTrigger) > 0.2)
    {
      air.driveDropWheels(joy.getRawAxis(hMap.axisRightTrigger));
    }
    else
    {
      air.driveDropWheels(0);
    }
    */
  }
}
