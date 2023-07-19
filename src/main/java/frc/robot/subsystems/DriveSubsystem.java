// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class DriveSubsystem extends SubsystemBase {
  
  //Initialize motors
  TalonSRX leftMotorMain = new TalonSRX(MotorConstants.kLeftMotorMain);
  TalonSRX leftMotorFollow = new TalonSRX(MotorConstants.kLeftMotorFollow);
  TalonSRX rightMotorMain = new TalonSRX(MotorConstants.kRightMotorMain);
  TalonSRX rightMotorFollow = new TalonSRX(MotorConstants.kRightMotorFollow);
  



  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configMotors();
  }

  private void configMotors() {
    leftMotorFollow.set(ControlMode.Follower, MotorConstants.kLeftMotorMain);
    rightMotorFollow.set(ControlMode.Follower, MotorConstants.kRightMotorMain);

    frontLeft.set(ControlMode.Follower, MotorConstants.kBackLeft);



    frontLeft.setInverted(false);
    backLeft.setInverted(false);
    frontRight.setInverted(true);
    backRight.setInverted(true);
     
    frontLeft.configClosedloopRamp(0.5);
    frontRight.configClosedloopRamp(0.5);
    backLeft.configClosedloopRamp(0.5);
    backRight.configClosedloopRamp(0.5);

    System.out.println("Motors Configured!"); 
  }


  public void DriveArcade(double xSpeed, double ySpeed) {
    drive.arcadeDrive(xSpeed, ySpeed);
  }

  public void DriveTank(double left, double right) {
    drive.tankDrive(left, right);
  }

  public void stopDrive() {
    leftDriveMotors.set(0);
    rightDriveMotors.set(0);
  }

  public void setCoastMode() {
    frontLeft.setIdleMode(IdleMode.kCoast);
    backLeft.setIdleMode(IdleMode.kCoast);
    frontRight.setIdleMode(IdleMode.kCoast);
    backRight.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode() {
    frontLeft.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
