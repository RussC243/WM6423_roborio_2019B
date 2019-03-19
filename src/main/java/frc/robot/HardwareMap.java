package frc.robot;
/**
 This class defines how the roborio for each robot is wired to its peripheral hardware
  and what that hardware does. This abstracts the details so all numbers are in one place 
 */
public class HardwareMap {
  //Drive motors are mixed: Sparks, Talons on CAN bus and one talon on PWM
  public  final int canID_driveMotorLeftFront 	= 0 ; //can ID 0 on bag bot, PWM 0 on peanut and 2nd bots
  public  final int canID_driveMotorLeftRear 	  = 1;  //can ID 1 on bag and 2nd, PWM 1 on peanut and
  public  final int canID_driveMotorRightFront 	= 2;  //can ID 2 on bag and 2nd bots, PWM 2 on peanut
  public  final int canID_driveMotorRightRear 	= 3;  //can ID 3 on bag and 2nd bots, PWM 3 on peanut
  //Arm motors: Talon SRX  
  public  final int canID_armMotorLeft          = 4;  //can ID 4
  public  final int canID_armMotorRight         = 5;  //can ID 5
  // Pneumatics control module and Power Distribution Panel
  public  final int canID_PCM                   = 6;  //can ID 6
  public  final int canID_PDP                   = 7;  //can ID 7
  //Intake motor: Spark PWM  
  public  final int motorIntake           = 4; //next PWM is at 4 to be common with peanut 
  //Wrist, Spark PWM. 
  public  final int wristMotor            = 5; //PWM 
  //Climb wheels, Spark PWM
  public  final int climbWheelLeft        = 6; //PWM
  public  final int climbWheelRight       = 7; //PWM
  //Pneumatic Solonoids value for solenoid values. connected to pneumatic controller, PCM 
  public  final int pnuematic_front_down  = 0; 
  public  final int pnuematic_front_up    = 1;
  public  final int pnuematic_rear_down   = 2;
  public  final int pnuematic_rear_up     = 3;
  public  final int pnuematic_hatch_pull  = 4;
  public  final int pnuematic_hatch_push  = 5;
  // compressors
  public  final int pneumaticCompressor = 0 ;
  // sensors
  public  final int potArm    = 0;
  public  final int potWrist  = 1;
  //Joystick - This is the IZT brand,  generic USB joystick
  private final int buttonA             = 1; 
  public  final int buttonArmCycleDown  = buttonA;
  private final int buttonB             = 2; 
  public  final int buttonClimbDown     = buttonB;
  private final int buttonX             = 3; 
  public  final int buttonArmCycleUp    = buttonX; 
  private final int buttonY             = 4; 
  public  final int buttonClimbUp       = buttonY; 
  private final int buttonBumperLeft    = 5; 
  public  final int buttonArmManualDown = buttonBumperLeft; 
  private final int buttonBumperRight   = 6; 
  public  final int buttonArmManualUp   = buttonBumperRight; 
  private final int axisLeftX           = 0; 
  public  final int axisLeftX_NOT_USED  = axisLeftX; 
  private final int axisLeftY         = 1; 
  public  final int axisTankDriveLeft   = axisLeftY;
  private final int axisTriggerLeft     = 2; 
  public  final int axisTriggerIntakeOut= axisTriggerLeft;
  private final int axisTriggerRight    = 3; 
  public  final int axisTriggerIntakeIn = axisTriggerRight;
  private final int axisRightX          = 4; 
  public  final int axisRightX_NOT_USED = axisRightX; 
  private final int axisRightY          = 5; 
  public  final int axisTankDriveRight  = axisRightY;
  
  public final int buttonHatchPush              = 77777;//TODO: map these place holders
  public final int buttonHatchPull              = 77777;
  public final int buttonDropWheelsDriveForward = 77777;
  public final int buttonDropWheelsDriveReverse = 77777;
  
  //Power Distribution Panel
  public  final int PDP_driveMotorLeftFront 	= 3;//TODO: replace these place holders 
  public  final int PDP_driveMotorLeftRear 	  = 3;  
  public  final int PDP_driveMotorRightFront 	= 3;  
  public  final int PDP_driveMotorRightRear 	= 3;  
  public  final int PDP_armMotorLeft          = 3;  
  public  final int PDP_armMotorRight         = 3;  
  public  final int PDP_intakeMotor           = 3; 
  public  final int PDP_wristMotor            = 3; 
  public  final int PDP_climbWheelLeft        = 3; 
  public  final int PDP_climbWheelRight       = 3; 
}
