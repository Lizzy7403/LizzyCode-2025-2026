// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Camera extends SubsystemBase {


  private final PhotonCamera camera = new PhotonCamera("photonvision");

  public Camera(){
    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  


  @Override
  public void periodic() {


    
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();
    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> targets = result.getTargets();
    // Get the current best target.
    PhotonTrackedTarget target = result.getBestTarget();


    // Get information from target.
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();
    //Transform2d pose = target.getCameraToTarget();
    //List<TargetCorner> corners = target.getCorners();

    // Get information from target.
    int targetID = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();


    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(null, null, alternateCameraToTarget);
    
    
    AprilTagFieldLayout x;


    Pose3d pose3d = new Pose3d(bestCameraToTarget.getTranslation(), bestCameraToTarget.getRotation());

    AprilTag aprilTag = new AprilTag(targetID, pose3d);

    
    ArrayList<AprilTag> sintaxisTest = new ArrayList<AprilTag>();
    sintaxisTest.add(aprilTag);
    AprilTagFieldLayout y = new AprilTagFieldLayout(sintaxisTest, 57.58333333333, 26.41666666666);
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(y, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
    


  }
}



