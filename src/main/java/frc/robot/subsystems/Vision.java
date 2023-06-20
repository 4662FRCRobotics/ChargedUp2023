// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera camera = new PhotonCamera(VisionConstants.kCAMERA_NAME);
  private double m_dyaw;
  private double m_dpitch;
  private double m_darea;
  private double m_dskew;
  private boolean m_bhaveTarget;
  int m_sampleCount;

  public boolean gethaveTarget() {
    return m_bhaveTarget;
  }

  public Vision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getTarget();
    SmartDashboard.putBoolean("targetfound", m_bhaveTarget);
    SmartDashboard.putNumber("targetyaw", getYaw());
    SmartDashboard.putNumber("targetpitch", getpitch());
    SmartDashboard.putNumber("targetskew", m_dskew);
    SmartDashboard.putNumber("targetarea", m_darea);

  }

  public void enableHubTargeting() {
    m_sampleCount = 0;
    m_dyaw = 0;
    m_dskew = 0;
    m_dpitch = 0;
    m_darea = 0;
    camera.setDriverMode(false);
    camera.setPipelineIndex(0);
    camera.setLED(VisionLEDMode.kOn);
    m_bhaveTarget = false;
  }

  public void disableHubTargeting() {
    camera.setDriverMode(true);
    camera.setLED(VisionLEDMode.kOff);
  }

  public void getTarget() {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    m_sampleCount = 0;
    if (hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();
      // double latencySeconds = result.getLatencyMillis() / 1000.0;
      m_dyaw = target.getYaw();
      m_dpitch = target.getPitch();
      m_darea = target.getArea();
      m_dskew = target.getSkew();
      // SmartDashboard.putBoolean("targetfound", m_bhaveTarget);
      // SmartDashboard.putNumber("targetyaw", getYaw());
      // SmartDashboard.putNumber("targetpitch", target.getPitch());
      m_bhaveTarget = true;
      // Transform2d pose = target.getCameraToTarget();
      // List<TargetCorner> corners = target.getCorners();
      m_sampleCount++;
    } else {
      m_bhaveTarget = false;
    }

  }

  public double getYaw() {
    return m_dyaw;
  }

  public double getpitch() {
    return m_dpitch;
  }

  public boolean haveTarget() {
    return m_bhaveTarget;
  }

}
