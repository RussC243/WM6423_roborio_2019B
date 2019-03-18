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
<<<<<<< HEAD

  HardwareMap hMap = new HardwareMap();   //This defines what inputs and outputs are connectedd to roborio
  DriveTrain driveTrain = new DriveTrain(selectedBot); //Subsystem for the robot's drivebase. 
  public static final int xboxPort = 0;
  public XboxController xCon = new XboxController(xboxPort);
  Pneumatics pneumatics = new Pneumatics(selectedBot); //Pneumatics subsystem.

=======
  
  Joystick        joy       = new Joystick(0);     //popular and generic, IZT brand joystick
  HardwareMap     hMap      = new HardwareMap();   //This defines what inputs and outputs are connectedd to roborio
  DriveTrain      dTrain    = new DriveTrain(selectedBot);
  ArmWrist        armWrist  = new ArmWrist(selectedBot);
  Pneumatics      air       = new Pneumatics(selectedBot);
  Intake          intake    = new Intake();
>>>>>>> e99fd21d91b28bdffb47a5b744b680e0d2d58351
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
    armWrist.armPositionTarget = armWrist.ARM_ANGLE_FULL_DOWN;//starting position 
    armWrist.wristPositionTarget = armWrist.ARM_ANGLE_FULL_UP; 
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

<<<<<<< HEAD
  //Following functions are commands linking a subsystem to joysticks/main robot class. 
=======
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
      intake.driveMotorIn(1);
    }
    else
    {
      if(joy.getRawAxis(hMap.axisTriggerIntakeOut) > 0.1)
      {
        intake.driveMotorIn(-1);
      }
      else
      {
        intake.driveMotorIn(0);
      }
    }
  }
 

>>>>>>> e99fd21d91b28bdffb47a5b744b680e0d2d58351
  public void linkJoyStickToDrive()
  {
    dTrain.drive(joy.getRawAxis(hMap.axisTankDriveLeft),joy.getRawAxis(hMap.axisTankDriveRight)); //left and right veritcal axis
  }
  public void linkJoyStickToArmWrist()
  {
    armWrist.upDownManual(joy.getRawButton(hMap.buttonArmManualUp), joy.getRawButton(hMap.buttonArmManualDown));
    armWrist.upDownCycle( joy.getRawButton(hMap.buttonArmCycleUp), joy.getRawButton(hMap.buttonArmCycleDown)); 
    //armWrist.upDownWristOnly(joy.getPOV());
  
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
    
    /*TODO: Find unused buttons on controller for the hatch and drop wheels
       add them to the hardware map then un-comment this code.
    //---- Control the hatch pistons. ------------------
    if(joy.getRawButton(hMap.buttonHatchPull))
    {
      air.hatchRetract();
    }
    if(joy.getRawButton(hMap.buttonHatchPush))
    {
      air.hatchExtend();
    }
    //---- Control the drop wheels. ------------------
    if(joy.getRawButton(hMap.buttonDropWheelsDriveForward))
    {
      air.driveDropWheels(1.0);
    }
    else if(joy.getRawButton(hMap.buttonDropWheelsDriveReverse))
    {
      air.driveDropWheels(-1.0);
    }
    else
    {
      air.driveDropWheels(0);
    }
    */ 
  }

  public void linkJoyStickButtonsToArmWrist() 
  {
    //left_button - manual arm down. Right_button - manual arm up. 

    
  }

  public void linkJoyStickButtonsToPnuematics() 
  {
    /*** Method for linking pneumatic commmands to joystick buttons.
     * TODO: Resolve which buttons are for which command. 
     * Also, should we change xCon to the IZT (generic HID)? 
     * Formula: 
     * Button condition - pnuematic command performed. 
    */

    if(xCon.getAButtonPressed() )
    {
      pneumatics.frontExtend(); //Extends front pneumatic system.
    }

    if(xCon.getBButtonPressed()) 
    {
      pneumatics.frontRetract(); // Retracts front pneumatic system. 
    }
      //More examples need to be made. 
      
    


  }
}
