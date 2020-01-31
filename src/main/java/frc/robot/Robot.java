
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Robot 1-25

package frc.robot;

import javax.swing.event.ChangeEvent;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
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
  
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  


  Joystick gamePad0 = new Joystick (0);
  VictorSP left0 = new VictorSP(2);
  VictorSP left1 = new VictorSP(3);

  VictorSP right0 = new VictorSP(0);
  VictorSP right1 = new VictorSP(1);
  SpeedControllerGroup leftDrive = new SpeedControllerGroup(left0,left1);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(right0,right1);
  DifferentialDrive driveTrain = new DifferentialDrive(leftDrive, rightDrive);
  VictorSP colMotor = new VictorSP(5);
  AnalogInput m_ultrasonic = new AnalogInput(0);
  AHRS navx;
  
  Timer autoPilotTimer = new Timer();

  

  int autoPilotStep = 0;
  int navxStep = 0;
  int rotatenum = 0;
  double targetAngle;
  boolean autoFace;
  double autoFaceTimeNeeded;
  boolean doAutoPilotNow = false;
  boolean b = true;
  boolean doubleAuto=false;
  double naenae; //it got less funny
  boolean Ball;
  String colorString;
  String gameData;
  String nextColor = "Purple Baby";
  String gameSadFace = "Mehh";
  
  private final I2C.Port cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 cSensor = new ColorSensorV3(cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color BlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color GreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color RedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color YellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  double yaw; 
    
   
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    double a;
    double v;

    int seenColor;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    navx = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
    rightDrive.setInverted(true);// Set true for Flash, set false for Simon/Tank
    leftDrive.setInverted(true);//false for Simon and Dave, true for Flash
    CameraServer.getInstance().startAutomaticCapture();

    m_colorMatcher.addColorMatch(BlueTarget);
    m_colorMatcher.addColorMatch(GreenTarget);
    m_colorMatcher.addColorMatch(RedTarget);
    m_colorMatcher.addColorMatch(YellowTarget);

    autoPilotTimer.start();
    navx.reset();
    navx.zeroYaw();
    colMotor.set(0);
    rotatenum = 0;
    seenColor = 0;
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
    
    int proximity = cSensor.getProximity();
    Color detectedColor = cSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    gameData = DriverStation.getInstance().getGameSpecificMessage();

    if (match.color == BlueTarget) {
      colorString = "B" ;
    } else if (match.color == RedTarget) {
      colorString = "R";
    } else if (match.color == GreenTarget) {
      colorString = "G";
    } else if (match.color == YellowTarget) {
      colorString = "Y";
    } else {
      colorString = "Unknown";
    }
    
    double IR = cSensor.getIR();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  

  double yaw = navx.getYaw(); //could not instantiate robot issue is here
    
   
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double a = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);

    double ratioX = (x-6.45)/27;  //was (x-6)/27 on 3/9
    double ratioY = y/40.5;
    double ratioA = .8*(1-(a/20));//changed <--- thank you very cool 1/25

    double min = .34;
    //double sineWithSignum = Math.signum(ratioX)*(1-min)*Math.sin(ratioX*Math.PI/2)+(1+min)/2;
    double sineLeft = ((ratioA+min)-((ratioA+min)-min)/2)*Math.sin(ratioX*Math.PI/2)+((ratioA+min)+min)/2;
    double sineRight = (min-((ratioA+min)-min)/2)*Math.sin(ratioX*Math.PI/2)+((ratioA+min)+min)/2;

    double correctionX = (Math.signum(ratioX) == 1) ? Math.sin(1.2*ratioX + .2): Math.sin(1.2*ratioX - .2);

    if ((gamePad0.getRawButtonPressed(1)) || doAutoPilotNow && v==1) { //a button
      autoPilotStep = 1;
    }
    else{doAutoPilotNow=false;}

    switch(autoPilotStep) {
      case 1:
      if(v==1){driveTrain.tankDrive((sineLeft/1.1/*(ratioX*.5)+ratioA*.8*/)+0.1,((sineRight/1.1/*(-(ratioX*.5))*12/13+ratioA*.8*/))+0.1);}//.4
      if (a > 6.6){autoPilotStep = 0;doAutoPilotNow = false;b=true;}
      break;
    }

    SmartDashboard.putNumber("LimelightX", x);
  SmartDashboard.putNumber("LimelightY", y);
  SmartDashboard.putNumber("LimelightA", a);
  SmartDashboard.putNumber("correctionX", correctionX);  
  SmartDashboard.putNumber("ratioX",ratioX);
  SmartDashboard.putNumber("D-Pad",gamePad0.getPOV());
  SmartDashboard.putNumber("Yaw",navx.getYaw());
  SmartDashboard.putNumber("NavxStep",navxStep);
  SmartDashboard.putBoolean("Auto",doAutoPilotNow);
  SmartDashboard.putBoolean("isBall", Ball);
  
  }

  @Override
  public void autonomousInit() {
    teleopInit();
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    
    teleopPeriodic();
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
    
    autoPilotTimer.start();
    navx.reset();
    navx.zeroYaw();
    colMotor.set(0);
    rotatenum = 0;
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
SmartDashboard.putString("gameData",gameData);
//SmartDashboard.putString("DetectedColor",colorString); just so  Iremember
SmartDashboard.putString("nextColor",nextColor);
SmartDashboard.putString("gameSadFace",gameSadFace);
SmartDashboard.putNumber("rotatenum",rotatenum);
/*
Implement logic
if 
*/
if(gameData.length() > 0)
{
  switch (gameData.charAt(0))
  {
    case 'B' :
    gameSadFace = "B";
      break;
    case 'G' :
    gameSadFace = "G";
      break;
    case 'R' :
    gameSadFace = "R";
      break;
    case 'Y' :
    gameSadFace = "Y";
      break;
    default :
    gameSadFace = "N/A";
      break;
  }
}
SmartDashboard.putNumber("seenColor",seenColor);
if (gamePad0.getRawButtonPressed(3)){rotatenum += 1;}
if (rotatenum > 0){
 colMotor.set(.5);
  if (nextColor == "B" && colorString == "Y" && seenColor == 0){
    rotatenum -= 1;
    seenColor = 1;
    }
    if (nextColor == "G" && colorString == "B"){
    rotatenum -= 1;
    }
    if (nextColor == "R" && colorString == "G"){
    rotatenum -= 1;
    }
    if (nextColor == "Y" && colorString == "R"){
    rotatenum -=1;
    }
    else{seenColor = 0;}
} else if (gamePad0.getRawButton(2)&& gameSadFace != nextColor){
colMotor.set(.25);
     if (nextColor == "B" && colorString == "G"){
     nextColor = "G";
     }
     if (nextColor == "G" && colorString == "R"){
     nextColor = "R";
    
     }
     if (nextColor == "R" && colorString == "Y"){
     nextColor = "Y";
    
     }
     if (nextColor == "Y" && colorString == "B"){
     nextColor = "B";
     
     }

}else{
 nextColor = colorString;
colMotor.stopMotor();
}
 
//}
    if(m_ultrasonic.getValue() > 245){Ball = false;}
    else{ Ball = true; }
    
    if(autoPilotStep==0){
      SmartDashboard.putNumber("leftStick",gamePad0.getRawAxis(1));
      SmartDashboard.putNumber("rightStick",gamePad0.getRawAxis(5));
      double leftStick = (-gamePad0.getRawAxis(1)*.6)*(gamePad0.getRawAxis(3)+1);
      //double leftStick = (-gamePad0.getRawAxis(1)*((gamePad0.getRawAxis(3)==1)?.6:.8));
      double rightStick = (-gamePad0.getRawAxis(5)*.6)*(gamePad0.getRawAxis(3)+1);
      //double rightStick = (-gamePad0.getRawAxis(5)*((gamePad0.getRawAxis(3)==1)?.6:.8));
      driveTrain.tankDrive(leftStick,rightStick);//12/13 is motor ratio for simon none for flash
     }

    if(autoPilotStep != 0 || autoFace){
      gamePad0.setRumble(RumbleType.kRightRumble, 1);
      gamePad0.setRumble(RumbleType.kLeftRumble, 1);
    }
    else{
      gamePad0.setRumble(RumbleType.kRightRumble, 0);
      gamePad0.setRumble(RumbleType.kLeftRumble, 0); 
    }
    
    SmartDashboard.putNumber("autopilot",autoPilotStep);

    if(gamePad0.getRawButtonPressed(8)){ //start button is kill switch for autoPilot, autoTurnLeft, and autoTurnRight
      autoPilotStep=0;
      doAutoPilotNow=false;
      seenColor = 0;
    }



   }
  /**
   * This function is called periodically during test mode.
   */
  
  @Override
  public void testPeriodic() {
  }
}
