// package frc.robot.Components;

// import frc.robot.LimelightHelpers;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;

// /* Summary of needed changes
//  * Talk to eli to understand how to implement the april tag calculation using the built in functions
//  * Remove all the unnecessary code
//  * Skeleton is correct, just need to implement
//  */

// public class LimelightComponent {
//     public static Pose2d calcAprilTag() {
//         // Basic targeting data
//         double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
//         double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
//         double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
//         boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

//         double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
//         double tync = LimelightHelpers.getTYNC("");  // Vertical  offset from principal pixel/point to target in degrees
        
//         // Switch to pipeline 0
//         LimelightHelpers.setPipelineIndex("", 0);
        
//         // Let the current pipeline control the LEDs
//         LimelightHelpers.setLEDMode_PipelineControl("");

//         // Force LEDs on/off/blink
//         LimelightHelpers.setLEDMode_ForceOn("");
//         LimelightHelpers.setLEDMode_ForceOff("");
//         LimelightHelpers.setLEDMode_ForceBlink("");

//         // In your periodic function:
//         LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
//         if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
//             m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
//             m_poseEstimator.addVisionMeasurement(
//                 limelightMeasurement.pose,
//                 limelightMeasurement.timestampSeconds
//             );
//         }

//         // First, tell Limelight your robot's current orientation
//         double robotYaw = m_gyro.getYaw();  
//         LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

//         // Get the pose estimate
//         LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

//         // Add it to your pose estimator
//         m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
//         m_poseEstimator.addVisionMeasurement(
//             limelightMeasurement.pose,
//             limelightMeasurement.timestampSeconds
//         );

//         // Set a custom crop window for improved performance (-1 to 1 for each value)
//         LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);


//         // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
//         LimelightHelpers.setCameraPose_RobotSpace("", 
//             0.5,    // Forward offset (meters)
//             0.0,    // Side offset (meters)
//             0.5,    // Height offset (meters)
//             0.0,    // Roll (degrees)
//             30.0,   // Pitch (degrees)
//             0.0     // Yaw (degrees)
//         );

//         // Set AprilTag offset tracking point (meters)
//         LimelightHelpers.setFiducial3DOffset("", 
//             0.0,    // Forward offset
//             0.0,    // Side offset  
//             0.5     // Height offset
//         );

//         // Configure AprilTag detection
//         LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 2, 3, 4}); // Only track these tag IDs
//         LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half resolution for improved framerate and reduced range

//         if(hasTarget){
//             return new Pose2d(); // Fix to pose2d based on april tag positioning
//         }else{
//             return null; // No valid target, return null
//         }
//     }
// }