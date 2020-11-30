
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Robot 1-25, http://10.51.41.11:5801/ 

package frc.robot;

import javax.swing.event.ChangeEvent;

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
  
  private static final String kCenter = "Center";
  private static final String kLeft = "Left";
  private static final String kRight = "Right";
  private static final String kOff = "Off";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  
  String autoPosition ="B-GAMERRRRRRRR";

  Joystick gamePad0 = new Joystick (0);
  /*
  Button mapping
  1 - AutoFace
  2 - Color 
  3 - Rotate
  4 - Conveyer input ?
  5 - Intake toggle
  6 - Shooter
  7 - 
  8 - Auto Kill
  */
  VictorSP left0 = new VictorSP(2);
  VictorSP left1 = new VictorSP(3);
  VictorSP right0 = new VictorSP(0);
  VictorSP right1 = new VictorSP(1);
  SpeedControllerGroup leftDrive = new SpeedControllerGroup(left0,left1);
  SpeedControllerGroup rightDrive = new SpeedControllerGroup(right0,right1);
  DifferentialDrive driveTrain = new DifferentialDrive(leftDrive, rightDrive);
  VictorSP colMotor = new VictorSP(5);
  VictorSP conveyor = new VictorSP(6);
  VictorSP shooter = new VictorSP(7);
  VictorSP intake = new VictorSP(8);
  DigitalOutput ultrasonicPing1 = new DigitalOutput(0);
  DigitalInput ultrasonicEcho1 = new DigitalInput(1);
  DigitalOutput ultrasonicPing2 = new DigitalOutput(2);
  DigitalInput ultrasonicEcho2 = new DigitalInput(3);
  DigitalOutput ultrasonicPing3 = new DigitalOutput(4);
  DigitalInput ultrasonicEcho3 = new DigitalInput(5);
  Ultrasonic ultrasonic1 = new Ultrasonic(ultrasonicPing1,ultrasonicEcho1);
  Ultrasonic ultrasonic2 = new Ultrasonic(ultrasonicPing2,ultrasonicEcho2);
  Ultrasonic ultrasonic3 = new Ultrasonic(ultrasonicPing3,ultrasonicEcho3);


  AHRS navx;
  
  Timer autoPilotTimer = new Timer();

  double h2 = 83; //height of target "inches"
  double h1 = 32; //height of camera
  double a1 = 20; //angle of camera 20 on test bot
  double disXnum;
  double aimnum;
  double difYnum = h2 - h1;
  double airtim;
  double fixedAngle = (45)*Math.PI/180; //angle of shooter 45-47 degrees
  double veloFwoosh; //angular velocity variable
  double velocityToMotorRatio = 2; //conversion rate

  int autoPilotStep = 0;
  int navxStep = 0;
  int rotatenum = 0;
  double targetAngle;
  boolean autoFace;
  double autoFaceTimeNeeded;
  boolean doAutoPilotNow = false;
  boolean doubleAuto=false;
  double naenae; //it got less funny
  boolean Ball1;
  boolean Ball2;
  boolean Ball3;
  boolean intakeOn = true;
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

  double range1;
  double range2;

  double yaw; 
    
   
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;
  double x;
  double y;
  double a;
  double v;

  int seenColor;

  Timer warmUp = new Timer();
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Center", kCenter);
    m_chooser.addOption("Left", kLeft);
    m_chooser.addOption("Right", kRight);
    m_chooser.addOption("Off", kOff);
    SmartDashboard.putData("Auto positions", m_chooser);
    
    navx = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
    rightDrive.setInverted(true);// Set true for Flash, set false for Simon/Tank
    leftDrive.setInverted(true);//false for Simon and Dave, true for Flash
    CameraServer.getInstance().startAutomaticCapture();

    m_colorMatcher.addColorMatch(BlueTarget);
    m_colorMatcher.addColorMatch(GreenTarget);
    m_colorMatcher.addColorMatch(RedTarget);
    m_colorMatcher.addColorMatch(YellowTarget);

    autoPilotTimer.reset();
    autoPilotTimer.stop();
    navx.reset();
    navx.zeroYaw();
    colMotor.set(0);
    rotatenum = 0;
    seenColor = 0;
    warmUp.reset();
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

    double yaw = navx.getYaw();
    
   
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double a = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);

    double marginXerror = 93;
    double ratioX = x/27;  //was (x-6)/27 on 3/9
    //double ratioY = (1.81-y)/20;  // was y/20 Based of angle target is seen at
    double ratioY = (disXnum-marginXerror)/25;  // Based of distance of target from dsXnum 108, 35
    double ratioA = (2.68 - a);//changed <--- thank you very cool 1/25
    double minCorrectX = .2;
    double maxCorrectX = .33;
    double minCorrectY = .22;
    double maxCorrectY = .6;

    //double sineWithSignum = Math.signum(ratioX)*(1-min)*Math.sin(ratioX*Math.PI/2)+(1+min)/2;
    double sineX = Math.signum(ratioX)*((maxCorrectX - minCorrectX)/2)*Math.sin(Math.PI*(ratioX-.5))+Math.signum(ratioX)*((maxCorrectX + minCorrectX)/2);
    double sineY = Math.signum(ratioY)*((maxCorrectY - minCorrectY)/2)*Math.sin(Math.PI*(ratioY-.5))+Math.signum(ratioY)*((maxCorrectY + minCorrectY)/2);
    
    if  (doAutoPilotNow && v==1) { //a button
      autoPilotStep = 1;
    }
    else {
      doAutoPilotNow=false;
      autoPilotStep = 0;
    }
    switch(autoPilotStep) {
      case 1:
      if(v==1){driveTrain.tankDrive(sineX+sineY,-(sineX)+sineY);}//.4
      //if (x > -1 && x < 1 && disXnum > (marginXerror-.8) && disXnum < (marginXerror+.8)){autoPilotStep = 0;doAutoPilotNow = false;}
      if (x > -1 && x < 1 && disXnum > (marginXerror-1.5) && disXnum < (marginXerror+1.5)) {
        if(autoPilotTimer.get() == 0){autoPilotTimer.start();}
      }
      else {
        autoPilotTimer.reset();
        autoPilotTimer.stop();
      }
      if (autoPilotTimer.hasPeriodPassed(1)) {
        autoPilotStep = 0;
        doAutoPilotNow = false;
      }
      /*if (x > -.8 && x < .8 && ratioY > -.8 && ratioY < .8 && ratioA > -.8 && ratioA < .8){
        autoPilotTimer.start();
      }else{
        autoPilotTimer.reset();
        autoPilotTimer.stop();
      }
      if (autoPilotTimer.hasPeriodPassed(1)){
        autoPilotStep = 0;
        doAutoPilotNow = false;
      }*/
      break;
    }
    // Potential Goldilocks, original dood distance y = 7.39
    //((y < -1.48)?.25:-.25)
    //(7.39-y)/20*.9
    aimnum = Math.sqrt((disXnum*Math.tan(fixedAngle)*(disXnum)*(disXnum))/2*Math.pow(Math.cos(fixedAngle),2)*difYnum);

    //disXnum = ((h2-h1)/Math.tan((a1-y)*Math.PI/180));
    disXnum = (h2-h1)/(Math.tan((a1+y)*Math.PI/180));
    airtim = Math.sqrt((2*Math.sin(fixedAngle)/3786.22)*((disXnum/Math.cos(fixedAngle))-(difYnum/Math.sin(fixedAngle))));
    veloFwoosh = disXnum/(Math.cos(fixedAngle)*airtim);
    //(h2-h1)*.1/Math.tan((a1-y))
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightA", a);
    SmartDashboard.putNumber("ratioX",ratioX);
    SmartDashboard.putNumber("D-Pad",gamePad0.getPOV());
    SmartDashboard.putNumber("Yaw",navx.getYaw());
    SmartDashboard.putNumber("NavxStep",navxStep);

    SmartDashboard.putBoolean("Auto",doAutoPilotNow);
    SmartDashboard.putBoolean("isBall1", Ball1);
    SmartDashboard.putBoolean("isBall2", Ball2);
    SmartDashboard.putBoolean("isBall3", Ball3);

    SmartDashboard.putNumber("DisXNum", disXnum);
    SmartDashboard.putNumber("DifYNum", difYnum);
    SmartDashboard.putNumber("aimnum",aimnum);
    SmartDashboard.putNumber("Velocity",veloFwoosh);
    SmartDashboard.putNumber("airtime",airtim);
  }

  @Override
  public void autonomousInit() {
    //teleopInit();
    m_autoSelected = m_chooser.getSelected();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    //teleopPeriodic();
    /*
    if( true ) {
      //
    }*/
    switch (m_autoSelected) {
      case kLeft:
        // Put left auto code here
        driveTrain.tankDrive(-.5,.5);
        break;
      case kRight:
        driveTrain.tankDrive(.5,-.5);
        // Put right auto code here
        break;
      case kCenter:
        // Put center auto code here
        driveTrain.tankDrive(.5,.5);
        break;
      default:
        // Put center auto code here
        driveTrain.tankDrive(0,0);
        break;
    }
    /*
    switch (autoPosition.charAt(0)) {
      case 'L':
        driveTrain.tankDrive(-.5,.5);
        break;
      case 'C':
        driveTrain.tankDrive(.5,.5);
        break;
      case 'R':
        driveTrain.tankDrive(.5,-.5);
        break;
      default:
        driveTrain.tankDrive(0,0);
        break;
    }
*/
  }
  @Override
  public void teleopInit() {
    
    autoPilotTimer.start();
    navx.reset();
    navx.zeroYaw();
    colMotor.set(0);
    rotatenum = 0;
    ultrasonic1.setEnabled(true);
    ultrasonic2.setEnabled(true);
    ultrasonic3.setEnabled(true);
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
  SmartDashboard.putNumber("ultrasonicRange1",ultrasonic1.getRangeMM());
  SmartDashboard.putNumber("ultrasonicRange2",ultrasonic2.getRangeMM());
  SmartDashboard.putNumber("ultrasonicRange3",ultrasonic3.getRangeMM());
  SmartDashboard.putNumber("Range1",range1);
  SmartDashboard.putNumber("Range2",range2);
  SmartDashboard.putBoolean("ultrasonic",ultrasonic1.isEnabled());


  if(gameData.length() > 0) {
    switch (gameData.charAt(0)) {
      case 'B' :
        gameSadFace = "R";
        break;
      case 'G' :
        gameSadFace = "Y";
        break;
      case 'R' :
        gameSadFace = "B";
        break;
      case 'Y' :
        gameSadFace = "G";
        break;
      default :
        gameSadFace = "N/A";
        break;
    }
  }
  SmartDashboard.putNumber("seenColor",seenColor);
  //Color Wheel Moter Must Turn Clockwise
  if (gamePad0.getRawButtonPressed(3)){rotatenum += 8;} //rotatenum is number of rotations asked for x2; e.g. setting for 8 makes 4 full rotations, setting for 7 gives 3.5 wheel rotations.
  if (rotatenum > 0){
    colMotor.set(.5);
    if (nextColor == "B" && colorString == "G" && seenColor == 0){
      rotatenum -= 1;
      seenColor = 1;
    }
    if (nextColor == "B" && colorString != "G" && seenColor == 1){
      seenColor = 0;
    }
    if (nextColor == "G" && colorString == "R" && seenColor == 0){
      rotatenum -= 1;
      seenColor = 1;
    }
    if (nextColor == "G" && colorString != "R" && seenColor == 1){
      seenColor = 0;
    }
    if (nextColor == "R" && colorString == "Y" && seenColor == 0){
      rotatenum -= 1;
      seenColor = 1;
    }
    if (nextColor == "R" && colorString != "Y" && seenColor == 1){
      seenColor = 0;
    }
    if (nextColor == "Y" && colorString == "B" && seenColor == 0){
      rotatenum -=1;
      seenColor = 1;
    }
    if (nextColor == "Y" && colorString != "B" && seenColor == 1){
      seenColor = 0;
    }
  } else if (gamePad0.getRawButton(2)&& gameSadFace != nextColor){
    colMotor.set(.25);
    if (nextColor == "B" && colorString == "Y"){
      nextColor = "Y";
    }
    if (nextColor == "Y" && colorString == "R"){
      nextColor = "R";
    }
    if (nextColor == "R" && colorString == "G"){
      nextColor = "G";
    }
    if (nextColor == "G" && colorString == "B"){
      nextColor = "B";
    }
  } else {
    nextColor = colorString;
    colMotor.stopMotor();
  }

    if(ultrasonic1.getRangeMM() > 200){Ball1 = false;}
    else{ Ball1 = true; }

    if(ultrasonic2.getRangeMM() > 200){Ball2 = false;}
    else{ Ball2 = true; }

    if(ultrasonic3.getRangeMM() > 200){Ball3 = false;}
    else{ Ball3 = true; }

    ultrasonic1.ping();
    ultrasonic2.ping();
    ultrasonic3.ping();

    if(gamePad0.getRawButtonPressed(4)){intakeOn = !intakeOn;}
    
    if(gamePad0.getRawButton(6)){//shooter Righth bumper
      warmUp.start();
      shooter.set(veloFwoosh*velocityToMotorRatio);//shooter value depending on target distance x and y
      if(warmUp.hasPeriodPassed(10)){conveyor.set(.5);}
    }
    else if(intakeOn && !Ball3 && !gamePad0.getRawButton(6) && !(gamePad0.getPOV() == 270)){//auto intake
      intake.set(.5);
      if(Ball1 || Ball2){conveyor.set(.5);} //button 4 questionable, propose we do it autonomous
      else{conveyor.set(0);}
    }else if(gamePad0.getPOV() == 270){
      intake.set(-.5);
      conveyor.set(-.5);
    }else{//default
      intake.set(0);
      conveyor.set(0);
    }
   if(!gamePad0.getRawButton(6)) {//not shooting
      warmUp.stop();
      warmUp.reset();
      shooter.set(0); //coolDown
    }

    if(gamePad0.getPOV() == 0){
      // Climb UP
    }
    if(gamePad0.getPOV() == 180){
      // Climb DOWN
    }

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

    if(gamePad0.getRawButtonPressed(1)){doAutoPilotNow = !doAutoPilotNow;}
    if(gamePad0.getRawButtonPressed(6) || gamePad0.getRawButtonPressed(10) || Math.abs(gamePad0.getRawAxis(1)) >= .2 || Math.abs(gamePad0.getRawAxis(5)) >= .2) { //a button
      autoPilotStep = 0;
      doAutoPilotNow=false;
    }

    if(gamePad0.getRawButtonPressed(8)){ //start button is kill switch for autoPilot, autoTurnLeft, and autoTurnRight
      autoPilotStep=0;
      doAutoPilotNow=false;
      seenColor = 0;
      rotatenum = 0;
      warmUp.reset();
      warmUp.stop();
      shooter.set(0);
    }



  }
  /**
   * This function is called periodically during test mode.
   */
  
  @Override
  public void testPeriodic() {
  }
}
