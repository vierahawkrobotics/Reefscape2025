// package frc.robot.Drivetrain.old;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Components.PositionTools.PositionTools;

// public class ScoreDriveCommand extends Command{
//     private enum DriveState{
//         goingToEntry,
//         goingToScore,
//         finished;
//     }
//     private Drivetrain drivetrain;
//     private DriveState driveState;
//     private DrivePoseBased driveCommand;
//     private boolean interrupt;
//     private boolean isRotated;
//     private double offset;
//     public ScoreDriveCommand(Drivetrain drivetrain, boolean isRotated, double limitSwitchOffset){
//         this.interrupt = false;
//         this.drivetrain = drivetrain;
//         this.driveState = DriveState.goingToEntry;
//         this.isRotated = isRotated;
//         this.offset = limitSwitchOffset;
//         addRequirements(drivetrain);
//     }

//     @Override
//     public void initialize(){
//         driveCommand = new DrivePoseBased(PositionTools.closestScorePoseEntry(isRotated),()->{return interrupt;});
//     }

//     @Override
//     public void execute(){
//         if(driveCommand.isFinished() && driveState == DriveState.goingToEntry){
//             driveState = DriveState.goingToScore;
//             driveCommand = new DrivePoseBased(PositionTools.closestScorePose(isRotated, offset),()->{return interrupt;});
//             driveCommand.schedule();
//         } else if(driveCommand.isFinished() && driveState == DriveState.goingToScore){
//             driveState = DriveState.finished;
//         }
//     }

//     @Override
//     public void end(boolean interrupted){
//         interrupt = true;
//     }

//     @Override
//     public boolean isFinished(){
//         return driveState == DriveState.finished;
//     }
// }