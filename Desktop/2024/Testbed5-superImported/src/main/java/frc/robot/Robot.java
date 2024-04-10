// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.FlatElevator;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Oi m_Oi;
  public static DriveTrain driveTrain = new DriveTrain();
  // public static FlatElevator flatElevator = new FlatElevator();
  
  private static double speed = .25;
  private Command m_autonomousCommand;
  // private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();
    m_Oi = new Oi();
    // CameraServer.startAutomaticCapture();
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  
  public void autonomousInit() {
    //autonomous mode if we're in the right corner
    // Robot.driveTrain.setRightMotorsForwardOverdrive(-speed);
    // Robot.driveTrain.setLeftMotorsForwardOverdrive(speed);
    // Timer.delay(.2);
    // Robot.driveTrain.stopRobot();
    // Timer.delay(.25);
    // Robot.driveTrain.setLeftMotorsForwardOverdrive( speed);
    //when its in the left, set it to positive speed with setLeftMotors. when its in the right set it to negative speed with the setRightMotors
    // Timer.delay(.6);
    //comment out above code if you're in the center position
    Robot.driveTrain.stopRobot();
    for (int i = -1; i <= 1; i += 2) {
      Robot.driveTrain.setRightMotorsForwardOverdrive(i*-speed);
      Robot.driveTrain.setLeftMotorsForwardOverdrive(i*speed);
      Timer.delay(2.25);
      Robot.driveTrain.stopRobot();
      // Timer.delay(1);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // System.out.println("testing if this is called");
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      //this is really weird and probably wrong i think i fucked up writing it earlier but im not going to touch it cause i dont want to fuck it up more
      // Robot.driveTrain.motorsInvert();

    }
    Robot.driveTrain.initDefaultCommand();
    // Robot.flatElevator.initDefaultCommand();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
