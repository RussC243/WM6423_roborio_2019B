package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.vision.VisionPipeline;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
import frc.robot.LPipeline;
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
  final OurBots selectedBot = OurBots.WM2019_BAG; //set the bot to the one you are working with

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean initDone = false; //allows not to reset to starting position after auto init
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
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);//
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  CameraServer.getInstance().startAutomaticCapture();
    armWrist.armPositionTarget = armWrist.ARM_POT_INITIAL;//starting position 
    armWrist.wristPositionTarget = armWrist.WRIST_POT_INITIAL; 
    armWrist.resetPids();
    air.retract();
    /*
    VisionThread vThread;
    UsbCamera camera = new UsbCamera("Camera 2", 0);
    //VisionPipeline l = new LPipeline();
   
    //MjpegServer server = new MjpegServer("new server",6423);
    CvSource source = CameraServer.getInstance().putVideo("new source", 300, 300);
    vThread = new VisionThread(camera,new LPipeline(), pipeline -> {
   //CvSink s = CameraServer.getInstance().getVideo();
        
          System.out.println("hello vision");
          source.putFrame(pipeline.desaturateOutput());
        
  });
  vThread.start();
  */
  
  
  
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
    armWrist.armPositionTarget = armWrist.ARM_POSE_1;//starting position 
    armWrist.wristPositionTarget = armWrist.WRIST_ARM_POSE_1;
     
    //** All the needed init was done in robotInit */
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        linkPack(); //Driver can control this year during autonomous.
                    //Link the joystick to the hardware.
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        linkPack(); //Driver can control this year during autonomous.
                    //Link the joystick to the hardware.
        break;
    }
  }

  @Override
  public void teleopInit() {
    System.out.printf("teleopInit\n"); 
    /**   All the init we needed was done in roboInit. */
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
    //One of team's joystick controllers changed from 0.5 to 1.0 not 0.0 to 1.0 when trigger was pressed
    //Russ' controller changed from 0.0 to 1.0
    //So... 0.7 works for both cases
    if(joy.getRawAxis(hMap.axisTriggerIntakeIn) > 0.7)
    {                               
      intake.driveMotorIn();//scale in intake class
    }
    else
    {
      if(joy.getRawAxis(hMap.axisTriggerIntakeOut) > 0.7)
      {
        intake.driveMotorOut();//scale in intake class
      }
      else
      {
        intake.driveMotorOff();
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
    //armWrist.upDownCycle      (joy.getRawButton(hMap.buttonArmWristCycleUp),  joy.getRawButton(hMap.buttonArmWristCycleDown)); 
  }
  public void linkJoyStickToPneumatics()
  {
    //---- Control the hatch pistons. ------------------
    if(joy.getPOV() == (hMap.povHatchPush) )
    {
      air.hatchPush(); //if button pushed, push hatch
      armWrist.upDownManualWrist(joy.getPOV() == 270,false);
    }
    else
    {
      air.hatchPull(); //else pull it in for 1 second
    }
    
    if(joy.getPOV() == (hMap.povClimbExtend))
    {
      air.climb();
    }
    else
    {
      if(joy.getPOV() == (hMap.povClimbRetract))
        air.retract();
      else
        air.climbExtendOff();
        air.climbRetractOff();

    }
    
  }
}
