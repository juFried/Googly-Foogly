// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.EncoderPID;

public class MoveArmLow extends CommandBase {
  /** Creates a new MoveArmLow. */
  public MoveArmLow() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_EncoderPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_EncoderPID.setSetpoint(ArmConstants.kLowArm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = RobotContainer.m_EncoderPID.getMeasurement();
    RobotContainer.m_EncoderPID.useOutput(output, ArmConstants.kLowArm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
