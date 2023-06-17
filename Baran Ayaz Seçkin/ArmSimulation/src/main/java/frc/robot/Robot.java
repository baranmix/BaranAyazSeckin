package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
  private final Arm m_arm = new Arm();

  @Override
  public void robotInit() {}

  @Override
  public void simulationPeriodic() {
    m_arm.simulationPeriodic();
  }

  @Override
  public void teleopInit() {
    m_arm.loadPreferences();
    //m_arm.setPreferences(0, 0.5, 0.8, 0.001);
  }

  @Override
  public void teleopPeriodic() {
    m_arm.reachSetpoint();
  }

  @Override
  public void close() {
    m_arm.close();
    super.close();
  }

  @Override
  public void disabledInit() {
    m_arm.stop();
  }
}
