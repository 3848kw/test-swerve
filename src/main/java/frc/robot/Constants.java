// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 8;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.02;
    public static final double LEFT_Y_DEADBAND  = 0.02;
    public static final double RIGHT_X_DEADBAND = 0.02;
    public static final double TURN_CONSTANT    = 2;
    


  }

  public static class Arm {
    public static final int Port_Neo_Arm = 14;
    public static final boolean Invert_Neo_Arm = true;
    public static final MotorType N_MOTOR_TYPE = MotorType.kBrushless;
    public static final double Arm_Neo_Rise_Speed = .35;
    public static final double Arm_Neo_Fall_Speed = -.35;

    public static final double SET_FORWARD = -3;
    public static final double SET_BACKWARD = -55;

    public static final double CLAW_PROPORTIONAL_CONSTANT = .01;
    public static final double CLAW_INTEGRAL_CONSTANT = .1;
    public static final double CLAW_DERIVATIVE_CONSTANT = 0;
    public static final double CLAW_POSITION_TOLERANCE = 3; // revolutions
    public static final double CLAW_VELOCITY_TOLERANCE =  800; // RPM
    
  }
  public static class Intake {
    public static final int Port_Neo_Intake = 13;
    public static final boolean Invert_Neo_Intake = true;
    public static final MotorType NI_MOTOR_TYPE = MotorType.kBrushless;
    public static final double Intake_Neo_out_Speed = 2;
    public static final double Intake_Neo_in_Speed = -.8;
    public static final double Secret_Intake_Speed = 1.5;
   }
  public static class Shooter {
    public static final int Port_Redline1 = 15;
    public static final int Port_Redline2 = 16;
    public static final boolean Invert_Redline1_Shooter = false;
    public static final boolean Invert_Redline2_Shooter = true;
    public static final MotorType Nr1_MOTOR_TYPE = MotorType.kBrushed;
    public static final MotorType Nr2_MOTOR_TYPE = MotorType.kBrushed;
    public static final double Shooter_out_R1_Speed = 1;
    public static final double Shooter_in_R1_Speed = -.4;
    public static final double Shooter_out_R2_Speed = 1;
    public static final double Shooter_in_R2_Speed = -.6;
    public static final double Shooter_out_R1L_Speed = .17677;
    public static final double Shooter_out_R2L_Speed = .17677;

  }

    public static class Climber {
    public static final int Port_Climber1 = 18;
    public static final int Port_Climber2 = 19;
    public static final boolean Invert_Climber1 = false;
    public static final boolean Invert_Climber2 = false;
    public static final MotorType Nc1_MOTOR_TYPE = MotorType.kBrushless;
    public static final MotorType Nc2MOTOR_TYPE = MotorType.kBrushless;
    public static final double Climber1_up_Speed = 1;
    public static final double Climber1_down_Speed = -1;
    public static final double Climber2_up_Speed = -1;
    public static final double Climber2_down_Speed = 1;
    }
  public static final class Control {
    public static final int Port_Joystick = 2;
}

}
