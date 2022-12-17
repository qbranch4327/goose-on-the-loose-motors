// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDriveSubsystem extends SubsystemBase {

  // these are limits you can change!!!
  public static final double kMaxSpeed = Units.feetToMeters(3.4); // 5 feet per second
  public static final double kMaxAngularSpeed = 0.25 * Math.PI; // 1/2 rotation per second
  public static double feildCalibration = 0;

  // this is where you put the angle offsets you got from the smart dashboard

  public static double frontLeftOffset = 15;
  public static double frontRightOffset = 250;
  public static double backLeftOffset = 176;
  public static double backRightOffset = 277;

  // put your can Id's here!
  public static final int frontLeftDriveId = 6;
  public static final int frontLeftCANCoderId = 3;
  public static final int frontLeftSteerId = 3;
  // put your can Id's here!
  public static final int frontRightDriveId = 5;
  public static final int frontRightCANCoderId = 0;
  public static final int frontRightSteerId = 0;
  // put your can Id's here!
  public static final int backLeftDriveId = 7;
  public static final int backLeftCANCoderId = 1;
  public static final int backLeftSteerId = 2;
  // put your can Id's here!

  public static final int backRightDriveId = 1;
  public static final int backRightCANCoderId = 2;
  public static final int backRightSteerId = 4;
  public static AHRS gyro = new AHRS(I2C.Port.kOnboard);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(10)),
      new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(-10)),
      new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(10)),
      new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-10)));

  private SwerveModuleMK4i[] modules = new SwerveModuleMK4i[] {
      new SwerveModuleMK4i(new TalonFX(frontLeftDriveId), new TalonFX(frontLeftSteerId),
          new CANCoder(frontLeftCANCoderId), Rotation2d.fromDegrees(frontLeftOffset)), // Front Left
      new SwerveModuleMK4i(new TalonFX(frontRightDriveId), new TalonFX(frontRightSteerId),
          new CANCoder(frontRightCANCoderId), Rotation2d.fromDegrees(frontRightOffset)), // Front Right
      new SwerveModuleMK4i(new TalonFX(backLeftDriveId), new TalonFX(backLeftSteerId), new CANCoder(backLeftCANCoderId),
          Rotation2d.fromDegrees(backLeftOffset)), // Back Left
      new SwerveModuleMK4i(new TalonFX(backRightDriveId), new TalonFX(backRightSteerId),
          new CANCoder(backRightCANCoderId), Rotation2d.fromDegrees(backRightOffset)) // Back Right

  };

  public SwerveDriveSubsystem() {
    // gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param calibrateGyro button to recalibrate the gyro offset
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean calibrateGyro) {

    if (calibrateGyro) {
      gyro.reset(); // recalibrates gyro offset
    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-gyro.getAngle()))
        : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);

    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK4i module = modules[i];
      SwerveModuleState state = states[i];
      SmartDashboard.putNumber(String.valueOf(i), module.getRawAngle());
      // below is a line to comment out from step 5
      module.setDesiredState(state);
      SmartDashboard.putNumber("gyro Angle", gyro.getAngle());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
