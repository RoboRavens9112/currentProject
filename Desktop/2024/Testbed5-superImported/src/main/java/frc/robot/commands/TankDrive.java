// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;


public class TankDrive extends Command {
  /** Creates a new TankDrive. */
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
    double leftStickY = Robot.m_Oi.GetDriverRawAxis(0)*.18;
    double rightStickY = Robot.m_Oi.GetDriverRawAxis(5)*.18;
    // boolean bButtonPressed = Robot.m_Oi.bButtonPushed();
    // double leftStickY = Robot.m_Oi.GetDriverRawAxis(1)*.18;
  
    // Robot.driveTrain.setLeftMotorsForward(leftStickY-leftStickX);
    // Robot.driveTrain.setRightMotorsForward(leftStickY+leftStickX);
      Robot.driveTrain.setLeftMotorsForward(leftStickY);
      Robot.driveTrain.setRightMotorsForward(rightStickY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setLeftMotorsForward(0);
    Robot.driveTrain.setRightMotorsForward(0);
    Robot.driveTrain.setLeftMotorsTurn(0);
    Robot.driveTrain.setRightMotorsTurn(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
