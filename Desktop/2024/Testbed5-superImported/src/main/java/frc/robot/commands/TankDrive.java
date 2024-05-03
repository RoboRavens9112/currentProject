// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;


public class TankDrive extends Command {
  /** Creates a new TankDrive. */
  private double speed = .25;
  public TankDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  // Robot.driveTrain.motorsInvert();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // eli arcade drive mode
    double leftStickY = -Robot.m_Oi.GetDriverRawAxis(4)*.7;
    double leftStickX = Robot.m_Oi.GetDriverRawAxis(1);

    if (Robot.m_Oi.XButtonPushed()) {
      var result = Robot.camera.getLatestResult();
      if (result.hasTargets()) {
        leftStickY = Robot.turnController.calculate(result.getBestTarget().getYaw(), 0);
      }
    }
    Robot.driveTrain.setLeftMotorsForward((leftStickY-leftStickX)*speed);
    Robot.driveTrain.setRightMotorsForward((leftStickY+leftStickX)*speed);
    
    //regular tank drive
    // double leftStickY = -Robot.m_Oi.GetDriverRawAxis(5);
    // double rightStickY = Robot.m_Oi.GetDriverRawAxis(1);
    // // boolean bButtonPressed = Robot.m_Oi.bButtonPushed();

    // Robot.driveTrain.setLeftMotorsForward(leftStickY*speed);
    // Robot.driveTrain.setRightMotorsForward(rightStickY*speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.stopRobot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
