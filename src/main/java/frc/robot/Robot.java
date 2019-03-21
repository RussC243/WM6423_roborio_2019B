package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
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
  
  Joystick        joy       = new Joystick(0);     //popular and generic, IZT brand joystick
  HardwareMap     hMap      = new HardwareMap();   //This defines what inputs and outputs are connectedd to roborio
  DriveTrain      dTrain    = new DriveTrain(selectedBot);
  ArmWrist        armWrist  = new ArmWrist(selectedBot);
  Pneumatics      air       = new Pneumatics(selectedBot);
  Intake          intake    = new Intake();
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
    
    System.out.printf("autonomousInit\n"); 
    armWrist.armPositionTarget = armWrist.ARM_ANGLE_FULL_DOWN;//starting position 
    armWrist.wristPositionTarget = armWrist.ARM_ANGLE_FULL_UP;
    linkPack(); //This year driver can drive during autonomous.
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
    armWrist.armPositionTarget = armWrist.ARM_POT_FULL_DOWN;//starting position 
    armWrist.wristPositionTarget = armWrist.ARM_POT_FULL_UP; 
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    linkPack();//link the joystick to the hardware
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /** a handfull of methods that are easier to read when separate but usually called together  */
  private void linkPack()
  {
    linkJoyStickToIntake();
    linkJoyStickToDrive();
    linkJoyStickToPneumatics();
    linkJoyStickToArmWrist();             //sets the target 
    armWrist.processPIDsAndDriveMotors(); //drives the motors to match the targets using PIDs
  }
  
  public void linkJoyStickToIntake()
  {
    if(joy.getRawAxis(hMap.axisTriggerIntakeIn) > 0.1)
    {
      intake.driveMotorIn(0.7);
    }
    else
    {
      if(joy.getRawAxis(hMap.axisTriggerIntakeOut) > 0.1)
      {
        intake.driveMotorIn(-0.7);
      }
      else
      {
        intake.driveMotorIn(0);
      }
    }
  }
  public void linkJoyStickToDrive()
  {
    dTrain.drive(joy.getRawAxis(hMap.axisTankDriveLeft),joy.getRawAxis(hMap.axisTankDriveRight)); //left and right veritcal axis
  }
  public void linkJoyStickToArmWrist()
  {
    armWrist.upDownManualArm  (joy.getRawButton(hMap.buttonArmWristManualUp), joy.getRawButton(hMap.buttonArmWristManualDown));
    armWrist.upDownManualWrist(joy.getRawButton(hMap.buttonWristManualUp),    joy.getRawButton(hMap.buttonWristManualDown));
    armWrist.upDownCycle      (joy.getRawButton(hMap.buttonArmWristCycleUp),  joy.getRawButton(hMap.buttonArmWristCycleDown)); 
  }
  public void linkJoyStickToPneumatics()
  {
    //---- Control the hatch pistons. ------------------
    if(joy.getPOV() == (hMap.povHatchPush))
    {
      air.hatchPush();
    }
    if(joy.getPOV() == (hMap.povHatchPull))
    {
      air.hatchPull();
    } 
  }
}
