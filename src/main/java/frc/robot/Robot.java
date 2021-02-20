
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Robot 1-25, http://10.51.41.11:5801/ 

/* to do list
  organize imports
*/
package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kCenter = "Center";
  private static final String kLeft = "Left";
  private static final String kRight = "Right";
  private static final String kOff = "Off";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private String m_controlSelected;
  private final SendableChooser<String> m_control = new SendableChooser<>();

  private static final String kManual = "Manual";
  private static final String kSensor = "Sensor";

  private String m_challengeSelected;
  private final SendableChooser<String> m_challange = new SendableChooser<>();

  private static final String kComp = "Competition";
  private static final String kTask1 = "Task1";

  String Mode = "manual";// manual v. sensor

  Joystick gamePad0 = new Joystick(0);
  /*
   * Button mapping 1 - AutoFace 2 - Color 3 - Rotate 4 - Conveyer input ? 5 -
   * Intake toggle 6 - Shooter 7 - 8 - Auto Kill
   */
  VictorSP right0 = new VictorSP(0);
  VictorSP right1 = new VictorSP(1);
  VictorSP left0 = new VictorSP(2);
  VictorSP left1 = new VictorSP(3);
  SpeedControllerGroup leftDrive = new SpeedControllerGroup(left0, left1);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(right0, right1);
  DifferentialDrive driveTrain = new DifferentialDrive(leftDrive, rightDrive);

  AHRS navx;
  String navDrive = "null";
  int setAngle;
  double angledYaw;
  double AOC = 85; //Area of correction

  Timer challengeTimer = new Timer();
  int challengeTimerCheckpoint;
  double route;
  // NetworkTable table;
  /** Limelight Modes
   *  0 - use the LED Mode set in the current pipeline
   *  1 - force off
   *  2 - force blink
   *  3 - force on
   */
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  double x;
  double y;
  double a;
  double v;

  double minCorrectNavX = .34;
  double maxCorrectNavX = .65;

/*  enum ROUTES {
    BLUE,
    RED,
    GREEN
  }

  if(ROUTES.BLUE) {}*/
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // m_choosers on bottom
    SmartDashboard.putData("Auto positions", m_chooser);
    m_chooser.addOption("Center", kCenter);
    m_chooser.addOption("Left", kLeft);
    m_chooser.addOption("Right", kRight);
    m_chooser.setDefaultOption("Off", kOff);

    SmartDashboard.putData("Control Mode", m_control);
    m_control.setDefaultOption("Manual", kManual);
    m_control.addOption("Sensor", kSensor);

    SmartDashboard.putData("Game Mode", m_challange);
    m_challange.setDefaultOption("Competition", kComp);
    m_challange.addOption("Task1", kTask1);

    // navx = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData,
    // (byte)50);
    right0.setInverted(true); //false for flash
    right1.setInverted(true); // false for flash
    left0.setInverted(true); //true for flash
    left1.setInverted(true); //true for flash

    navx = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
    navx.zeroYaw();
    navx.reset();

  //  CameraServer.getInstance().startAutomaticCapture(); //non-li\melight camera declaration????? why is it here.

    // navx.reset();
    // navx.zeroYaw();

    table.getEntry("ledMode").setNumber(3);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    double yaw = navx.getYaw();
    SmartDashboard.putNumber("Yaw",navx.getYaw());

    final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    final NetworkTableEntry tx = table.getEntry("tx");
    final NetworkTableEntry ty = table.getEntry("ty");
    final NetworkTableEntry ta = table.getEntry("ta");
    final NetworkTableEntry tv = table.getEntry("tv");

    final double x = (tx.getDouble(0.0)); // x & y is negative because limelight is upsidedown
    final double y = (ty.getDouble(0.0));
    final double a = ta.getDouble(0.0);
    final double v = tv.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightA", a);
    
    double ratioNavX;
    setAngle = 0;

    if (Math.abs(yaw - setAngle) <= 180){
      angledYaw = yaw - setAngle;
    }else{
      angledYaw = -Math.signum(yaw - setAngle)*(360-Math.abs(yaw - setAngle));
    }

    if(Math.abs(angledYaw) < (AOC)) {
      ratioNavX =angledYaw/AOC;
    } else if (angledYaw > 0){
      ratioNavX = 1;
    } else { ratioNavX = -1;}

    //double sineWithSignum = Math.signum(ratioNavX)*(1-min)*Math.sin(ratioNavX*Math.PI/2)+(1+min)/2;
    double sineNavX = Math.signum(ratioNavX)*((maxCorrectNavX - minCorrectNavX)/2)*Math.sin(Math.PI*(ratioNavX-.5))+Math.signum(ratioNavX)*((maxCorrectNavX + minCorrectNavX)/2);
    
  if(navDrive.length() > 0) {
    switch (navDrive.charAt(0)) {
      case 'T' :
        minCorrectNavX = .34;
        maxCorrectNavX = .65;
        AOC = 85;

        driveTrain.tankDrive(-sineNavX,sineNavX);
        break;
      case 'D' :
        minCorrectNavX = 0.5;//.4
        maxCorrectNavX = 1;//.75
        AOC = 15;//15

        double cCorrection = (-sineNavX > 0) ? -sineNavX : minCorrectNavX;
        double ccCorrection  = (sineNavX > 0) ? sineNavX : minCorrectNavX;

        driveTrain.tankDrive(cCorrection,ccCorrection);
       /*
        if (angledYaw > 0) {
          driveTrain.tankDrive(
            (maxCorrectNavX-minCorrectNavX) * (angledYaw/180) + minCorrectNavX,
            (maxCorrectNavX-minCorrectNavX) - (maxCorrectNavX-minCorrectNavX) * (angledYaw/180) + minCorrectNavX
        );  
        }
        else {
          driveTrain.tankDrive(
          (maxCorrectNavX-minCorrectNavX) - (maxCorrectNavX-minCorrectNavX) * (angledYaw/180) + minCorrectNavX,
          (maxCorrectNavX-minCorrectNavX) * (angledYaw/180) + minCorrectNavX
        );

        }*/
        
    
        break;
      default :
        minCorrectNavX = .34;
        maxCorrectNavX = .65;
        AOC = 85;
        break;
  }
  }

}
  @Override
  public void autonomousInit() {

    table.getEntry("ledMode").setNumber(3);

    navx.zeroYaw();
    navx.reset();

    challengeTimer.stop();
    challengeTimer.reset();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    final NetworkTableEntry tx = table.getEntry("tx");
    final NetworkTableEntry ty = table.getEntry("ty");
    final NetworkTableEntry ta = table.getEntry("ta");
    final NetworkTableEntry tv = table.getEntry("tv");

    m_autoSelected = m_chooser.getSelected();
    m_challengeSelected = m_challange.getSelected();


    switch (m_autoSelected) {
    case kLeft:
      // Put left auto code here
      driveTrain.tankDrive(-.5, .5);
      break;
    case kRight:
      driveTrain.tankDrive(.5, -.5);
      // Put right auto code here
      break;
    case kCenter:
      break;
    case kOff:
      break;
    default:
      break;
    }

    SmartDashboard.putNumber("challengeTimer", challengeTimer.get());
    SmartDashboard.putNumber("route number", challengeTimer.get());
  
      switch (m_challengeSelected) {
        case kComp:
          // Put left auto targetting and shooting code here
          //driveTrain.tankDrive(-.5, .5);
          break;

        //Use Yellow Limelight Snapshot setting
        case kTask1:
          if (challengeTimer.get() == 0) {
            route = y;
            challengeTimer.start();
          } 

          if( route < 0 ) {
              //
            }
          
           else  { //red config
              //
            }


          break;
        default:
          break;
    }



  }

  @Override
  public void teleopInit() {

    table.getEntry("ledMode").setNumber(3);

    navx.zeroYaw();
    navx.reset();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    m_controlSelected = m_control.getSelected(); 

    SmartDashboard.putNumber("leftStick", gamePad0.getRawAxis(1));
    SmartDashboard.putNumber("rightStick", gamePad0.getRawAxis(5));
    
    final double leftStick = (-gamePad0.getRawAxis(1) * .6) * (gamePad0.getRawAxis(3) + 1);
    // double leftStick =
    // (-gamePad0.getRawAxis(1)*((gamePad0.getRawAxis(3)==1)?.6:.8));
    final double rightStick = (-gamePad0.getRawAxis(5) * .6) * (gamePad0.getRawAxis(3) + 1);
    // double rightStick =
    // (-gamePad0.getRawAxis(5)*((gamePad0.getRawAxis(3)==1)?.6:.8));
    driveTrain.tankDrive(leftStick, rightStick);// 12/13 is motor ratio for simon none for flash


    if (gamePad0.getRawButton(1)){
      navDrive = "Turn";
    } else if (gamePad0.getRawButton(2)) {
      navDrive = "Drive";
    } else {
      navDrive = "null";
    }

    if (gamePad0.getRawButton(5)){
      navx.zeroYaw();
      navx.reset();
    }


  }

  /**
   * This function is called periodically during test mode.
   */

  @Override
  public void testPeriodic() {
  }
}
