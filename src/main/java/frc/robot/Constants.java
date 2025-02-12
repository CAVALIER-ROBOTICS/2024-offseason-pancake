// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.I2C;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class ShooterConstants {
    public static final int SHOOTER_LIMIT_SWITCH_ID = 1;

    public static final int SPEED_SHOOTER = 1;
    public static final double MAX_FLYWHEEL_PERCENT_OUTPUT = .7; // 1.863636363636364
    public static final double MAX_RPM_FLYWHEEL = 3350;
    public static final double MAX_POSITION_SHOOTER = 0.822;
    public static final double MIN_POSITITON_SHOOTER = 0.572;
    public static final double SHOOTER_HORIZONTAL = 0.7175;
    public static final double SHOOTER_45_DEGREE = 0.703; 
    public static final double SHOOTER_VERTICAL = 0.822;
    public static final double SHOOTER_LINEUP_POSITION = 0.15;
  }

  public final class JetsonConstants {
    public static final String table = "targeting";
    public static final String vSlamX = "vslamdata_poseX";
    public static final String vSlamY = "vslamdata_poseY";
    public static final String tagX = "";
    public static final String tagY = "";
  }

  public final class IntakeConstants {
    public static final I2C.Port INTAKE_SENSOR_PORT = I2C.Port.kOnboard;
    public static final int MINIMUM_PROXIMITY_TRIGGER = 9;

    public static final double RETRACTED_POS = 0.77851161951279;
    public static final double EXTENDED_POS = 0.079076751851919;
    public static final double INTAKE_LINEUP_POSITION = RETRACTED_POS;
  }

  public final class AmpBarConstants {
    public static final int AMP_MOTOR_ID = 37;
    public static final double AMPBAR_RETRACTED = .136;
    public static final double AMPBAR_EXTENDED = .583;
  }

  public final class SwerveConstants {

    // public static final double FLEFT_OFFSET = -2.876213977285577;
    // public static final double FRIGHT_OFFSET = -2.311709047343661;
    // public static final double BLEFT_OFFSET = -2.16444689170664;
    // public static final double BRIGHT_OFFSET = -0.193281579273591 - Math.toRadians(2);

    public static final double FLEFT_OFFSET = -0.148796136424907;
    public static final double FRIGHT_OFFSET = 2.621573166496561;
    public static final double BLEFT_OFFSET = 0.743980682124536;
    public static final double BRIGHT_OFFSET = 2.785709110800324 + Math.PI;

    public static final double BOT_LENGTH = .6858;
    public static double L = BOT_LENGTH;
    public static double W = BOT_LENGTH;

    public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(L/2, W/2),
    new Translation2d(L/2, -W/2),
    new Translation2d(-L/2, W/2),
    new Translation2d(-L/2, -W/2)
    );

    public static final Pose2d KATY_TEST_FIELD_INIT_POSE = new Pose2d(15.3, 5.53, Rotation2d.fromRadians(0));
  }

  public final class LEDConstants {
    public static final int BLINKIN_ID = 3;
    public static final double enabled = 0.75; // Green
    public static final double disabled = 0.61; // Red
    public static final double autonomous = -0.95; // Rainbow Ocean
    public static final double error = 0.69; // Yellow
  }

  public final class PiConstants {
    public static final String table = "targeting";
    public static final String robotX = "targetdata_robotX";
    public static final String robotY = "targetdata_robotX";
    public static final String noteX = "targetdata_noteX";
    public static final String noteY = "targetdata_noteY";
    public static String ourBotY = "";
    public static String ourBotX = "";
  }

  public static final int LEFT_INTAKE_ID = 16;
  public static final int RIGHT_INTAKE_ID = 17;
  public static final int SPIN_INTAKE_ID = 18;

  public static final int LEFT_CLIMB_ID = 24;
  public static final int RIGHT_CLIMB_ID = 25;
  
  public static final int FLEFT_DRIVE_ID = 1;
  public static final int FLEFT_STEER_ID = 2;

  public static final int FRIGHT_DRIVE_ID = 3;
  public static final int FRIGHT_STEER_ID = 4;

  public static final int BLEFT_DRIVE_ID = 5;
  public static final int BLEFT_STEER_ID = 6;

  public static final int BRIGHT_DRIVE_ID = 7;
  public static final int BRIGHT_STEER_ID = 8;

  public static final int PIGEON_ID = 13;
  public static final int FLEFT_CANCODER_ID = 20;
  public static final int FRIGHT_CANCODER_ID = 21;
  public static final int BLEFT_CANCODER_ID = 22;
  public static final int BRIGHT_CANCODER_ID = 23;

  public static final int RIGHT_SHOOTER_PIVOT_ID = 11;
  public static final int LEFT_SHOOTER_PIVOT_ID = 12;
  public static final int KICKER_ID = 13;
  public static final int BOTTOM_SHOOTER_ID = 14;
  public static final int TOP_SHOOTER_ID = 15;

  public static final double NOMINAL_VOLTAGE = 12.2;

  public static final double MAX_DISTANCE_TO_APRILTAG = 5; //Meters
  public static final double MAX_DISTANCE_TO_SINGLETAG = 3;
  
  public static final String CANIVORE = "OTHERCANIVORE";
  public static final double BOT_LENGTH = .6858;

  private static double L = Constants.BOT_LENGTH;
  private static double W = Constants.BOT_LENGTH;


  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(L/2, W/2),
    new Translation2d(L/2, -W/2),
    new Translation2d(-L/2, W/2),
    new Translation2d(-L/2, -W/2)
  );

  public static final String P_thetaSmartdashboard = "ThetaP";
  public static final String I_thetaSmartdashboard = "ThetaI";
  public static final String D_thetaSmartdashboard = "ThetaD";

  public static final String P_phiSmartdashboard = "PhiP";
  public static final String I_phiSmartdashboard = "PhiI";
  public static final String D_phiSmartdashboard = "PhiD";

  public static final String ShooterAngleAmpSD = "AmpSetpointAngle";
  public static final String FlywheelSpeedAmpSD = "FlywheelSetpointAngle";

  public static final int blinkinID = 0;

}
