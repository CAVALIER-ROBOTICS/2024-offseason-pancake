// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.CycloidLibrary.NeoKrakenModule;
import frc.robot.CycloidLibrary.NeoSteveModule;

public class DriveSubsystem extends SubsystemBase {
  NeoKrakenModule fleft, fright, bleft, bright;

  Pigeon2 pigeon = new Pigeon2(Constants.PIGEON_ID);

  public DriveSubsystem() {
    fleft = new NeoKrakenModule(Constants.FLEFT_DRIVE_ID, Constants.FLEFT_STEER_ID, Constants.FLEFT_CANCODER_ID, SwerveConstants.FLEFT_OFFSET);
    fright = new NeoKrakenModule(Constants.FRIGHT_DRIVE_ID, Constants.FRIGHT_STEER_ID, Constants.FRIGHT_CANCODER_ID, SwerveConstants.FRIGHT_OFFSET);
    bleft = new NeoKrakenModule(Constants.BLEFT_DRIVE_ID, Constants.BLEFT_STEER_ID, Constants.BLEFT_CANCODER_ID, SwerveConstants.BLEFT_OFFSET);
    bright = new NeoKrakenModule(Constants.BRIGHT_DRIVE_ID, Constants.BRIGHT_STEER_ID, Constants.BRIGHT_CANCODER_ID, SwerveConstants.BRIGHT_OFFSET);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    fleft.setModuleState(states[0]);
    fright.setModuleState(states[1]);
    bleft.setModuleState(states[2]);
    bright.setModuleState(states[3]);
    // fleft.setPercentOutput(.5);
    // fright.setPercentOutput(.5);
    // bleft.setPercentOutput(.5);
    // bright.setPercentOutput(.5);
  }

  private double clamp(double x, double min, double max) {
    return (x > max) ? max: (x < min) ? min: x;
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = SwerveConstants.m_kinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public void autonDrive(ChassisSpeeds speeds) { //Man, pathplanner is weird
    ChassisSpeeds speeds2 = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
    drive(speeds2);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      fleft.getSwerveModulePosition(), 
      fright.getSwerveModulePosition(), 
      bleft.getSwerveModulePosition(), 
      bright.getSwerveModulePosition()};
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      fleft.getSwerveModuleState(),
      fright.getSwerveModuleState(),
      bleft.getSwerveModuleState(),
      fright.getSwerveModuleState()
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    SwerveModuleState[] currentStates = getSwerveModuleStates();
    return SwerveConstants.m_kinematics.toChassisSpeeds(currentStates);
  }

  public double getRobotVelocityMagnitude() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
  }

  public double getPScalingFactor() {
    double percent = getRobotVelocityMagnitude() / 4.2;
    return percent;
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-pigeon.getAngle());
  }

  public void zeroGyro() {
    pigeon.reset();
  }

  public void setYaw(double yaw) {
    pigeon.setYaw(yaw);
  }

  public void pushMeasurementAndSetpoint(double setpoint) {
    SmartDashboard.putNumber("CurrentTheta", getAngle().getRadians());
    SmartDashboard.putNumber("SetpointTheta", setpoint);
    SmartDashboard.putNumber("ThetaError", (getAngle().getRadians() - setpoint) % Math.PI * 2);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FLEFT", fleft.getEncoderPosition());
    SmartDashboard.putNumber("FRIGHT", fright.getEncoderPosition());
    SmartDashboard.putNumber("BLEFT", bleft.getEncoderPosition());
    SmartDashboard.putNumber("BRIGHT", bright.getEncoderPosition());
    SmartDashboard.putNumber("Gyro angle rads", getAngle().getRadians());
  }
}
