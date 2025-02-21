package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.Vision.LimelightHelpers.*;

public class VisionSubsystem extends SubsystemBase {
  private RawFiducial[] fiducials;

  public VisionSubsystem() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {

    // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
    LimelightHelpers.setCameraPose_RobotSpace(
        "",
        Meters.convertFrom(30. / 2., Inches), // forward location wrt robot center
        0., // assume perfect alignment with robor center
        Meters.convertFrom(VisionConstants.limelightLensHeightInches, Inches), // height of camera from the floor
        0,
        0,
        0);
    // Overrides the valid AprilTag IDs that will be used for localization. 
    // Tags not in this list will be ignored for robot pose estimation.
    Optional<Alliance> ally = DriverStation.getAlliance();
    int[] tagsToConsider = new int[] {};
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          tagsToConsider = new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        }
        if (ally.get() == Alliance.Blue) {
          tagsToConsider = new int[] {12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
        }
    }
    else {
      System.out.println("No Alliance info is available");
    }

    LimelightHelpers.SetFiducialIDFiltersOverride("", tagsToConsider);
  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("");

  }

  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;
    SmartDashboard.putNumber("minDistance", minDistance);

    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }

    return closest;
  }

  public RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

public RawFiducial getFiducialWithId(int id, boolean verbose) {
  StringBuilder availableIds = new StringBuilder();

  for (RawFiducial fiducial : fiducials) {
      if (availableIds.length() > 0) {
          availableIds.append(", ");
      } //Error reporting
      availableIds.append(fiducial.id);
      
      if (fiducial.id == id) {
          return fiducial;
      }
  }
  throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
  }

  public double getTX(){
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTY(){
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTA(){
    return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME);
  }
  public boolean getTV(){
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
  }

  public double getClosestTX(){
    return getClosestFiducial().txnc;
  }
  public double getClosestTY(){
    return getClosestFiducial().tync;
  }
  public double getClosestTA(){
    return getClosestFiducial().ta;
  }
}