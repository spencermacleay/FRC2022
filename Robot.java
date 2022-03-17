// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */



  public class Robot extends TimedRobot {




XboxController controller = new XboxController(0);



//Encoder encoder = new Encoder(4, 5, false);
Timer timerauton = new Timer();
Timer timerangler = new Timer();
WPI_VictorSPX carriage1 = new WPI_VictorSPX(1);
WPI_VictorSPX carriage2 = new WPI_VictorSPX(2);

WPI_VictorSPX angler1 = new WPI_VictorSPX(5);
WPI_VictorSPX angler2 = new WPI_VictorSPX(6);

WPI_TalonSRX left1 = new WPI_TalonSRX(9);
WPI_TalonSRX left2 = new WPI_TalonSRX(8);
WPI_TalonSRX right1 = new WPI_TalonSRX(7);
WPI_TalonSRX right2 = new WPI_TalonSRX(10);

WPI_VictorSPX climber1 = new WPI_VictorSPX(3);
WPI_VictorSPX climber2 = new WPI_VictorSPX(4);

WPI_VictorSPX intake = new WPI_VictorSPX(11);

PWMSparkMax Rdrive1 = new PWMSparkMax(0);
PWMSparkMax Rdrive2 = new PWMSparkMax(1);
PWMSparkMax Ldrive1 = new PWMSparkMax(2);
PWMSparkMax Ldrive2 = new PWMSparkMax(3);

MotorControllerGroup CARRIAGE = new MotorControllerGroup(carriage1, carriage2);
MotorControllerGroup CLIMBER = new MotorControllerGroup(climber1, climber2);

MotorControllerGroup RIGHT = new MotorControllerGroup(Rdrive1, Rdrive2);
MotorControllerGroup LEFT = new MotorControllerGroup(Ldrive1, Ldrive2);

DifferentialDrive drivebase = new DifferentialDrive(RIGHT, LEFT);

double left_command = 0;
double right_command = 0;


  @Override
  public void robotInit() {
  
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);




    RIGHT.setInverted(true);
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    
  
  
  
  
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);



  double targetOffsetAngle_Vertical = ty.getDouble(0.0);

  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 25.0;
  
  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 35.5;
  
  // distance from the target to the floor
  double goalHeightInches = 104.0;
  
  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  
  //calculate distance
  double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

 
    



  }





  @Override
  public void autonomousInit() {
    
    timerauton.reset();
    timerauton.start();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  
    if (timerauton.get() < 1.6) {
      carriage1.set(0.57);
      carriage2.set(-0.57);
      right1.set(-0.57);
      right2.set(-0.57);
      left1.set(0.5);
      left2.set(0.5);
      
      //angler1.set(0.25);
      //angler2.set(-0.25);
      
    }
      else if(timerauton.get() < 3){
      
      drivebase.tankDrive(0.5, 0.5); 
    }
   else {
      drivebase.stopMotor(); // stop robot
      angler1.stopMotor();
      angler2.stopMotor();
      carriage1.stopMotor();
      carriage2.stopMotor();
      right1.stopMotor();
      right2.stopMotor();
      left1.stopMotor();
      left2.stopMotor();
    }


  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);
   
    double KpAim = -0.05;
    double KpDistance = -0.05;
    double min_aim_command = 0.05;

      
   
   
   if(controller.getStartButton()){
     CLIMBER.set(0.65);
     //servo.setAngle(30);
    }
    else if(controller.getBackButton()){
      CLIMBER.set(-0.65);
      
    }
    else if(controller.getRightStickButton()){

    //  servo.setAngle(90);
    }
    
    
    else{

      CLIMBER.set(0);
      

    }

    if(controller.getLeftTriggerAxis() > 0.3){
      right1.set(0.45);
      right2.set(0.45);
      left1.set(-0.45);
      left2.set(-0.45);
      intake.set(-0.45);
      carriage1.set(-0.5);
      carriage2.set(0.5);
    }
      else if(controller.getRightTriggerAxis() > 0.3 ){

      right1.set(-0.65);
      right2.set(-0.65);
      left1.set(0.65);
      left2.set(0.65);
      carriage1.set(0.5);
      carriage2.set(-0.5);
      }
      else if(controller.getAButton()){
        right1.set(-0.9);
        right2.set(-0.9);
        left1.set(0.9);
        left2.set(0.9);
        carriage1.set(0.5);
        carriage2.set(-0.5);
      } else {    
      right1.set(0);
      right2.set(0);
      left1.set(0);
      left2.set(0);
      carriage1.set(0);
      carriage2.set(0);
      intake.set(0);
      }
      
      if(controller.getBButton()){
         
        double heading_error = -x;
        double distance_error = -y;
        double steering_adjust = 0.0;

        if (x>1.0){

          steering_adjust = KpAim*heading_error - min_aim_command;
       }
       else if (x < 1.0){

        steering_adjust = KpAim*heading_error + min_aim_command;

       }
      
       double distance_adjust = KpDistance * distance_error; 

       left_command = steering_adjust;
       right_command = -steering_adjust;

       //System.out.println(left_command);
      
       drivebase.tankDrive(left_command, right_command);
    
    } 
      else{ 
      drivebase.tankDrive(controller.getRightY(), controller.getLeftY());
    }
 


      if(controller.getRightBumper()){
        angler1.set(0.4);
        angler2.set(-0.4);
      //  timerangler.reset();
        //timerangler.start();
      }
       
      else if(controller.getLeftBumper()){
          angler1.set(-0.25);
          angler2.set(0.25);
        }
      // else if(timerangler.get() < 1.5){
        //angler1.set(-0.1);
        //angler2.set(0.1);
       //}
        else{
          angler1.set(0);
          angler2.set(0);
        }


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
 