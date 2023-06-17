package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";
  public static final String kArmIKey = "ArmI";
  public static final String kArmDKey = "ArmD";

  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 50.0;
  public static final double kDefaultArmKi = 0.0;
  public static final double kDefaultArmKd = 0.0;
  public static final double kDefaultArmSetpointDegrees = 75.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kArmReduction = 10;
  public static final double kArmMass = 3.0; 
  public static final double kArmLength = Units.inchesToMeters(10);
  public static final double kMinAngleRads = Units.degreesToRadians(-90);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);
}
