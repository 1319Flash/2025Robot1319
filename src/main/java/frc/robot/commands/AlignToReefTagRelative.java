package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReefTagRelative extends Command {
    private PIDController xController, yController, rotController;

    private Timer dontSeeTagTimer, stopTimer;
    private final CommandSwerveDrivetrain drivebase;

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    public AlignToReefTagRelative(CommandSwerveDrivetrain drivebase) {
        this.drivebase = drivebase;

        xController = new PIDController(1.5, 0, 0);
        yController = new PIDController(1.5, 0, 0);
        rotController = new PIDController(0.05, 0, 0);

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        stopTimer = new Timer();
        stopTimer.start();

        dontSeeTagTimer = new Timer();
        dontSeeTagTimer.start();

        xController.setSetpoint(0);
        xController.setTolerance(0.2);

        yController.setSetpoint(0);
        yController.setTolerance(0.2);

        rotController.setSetpoint(0);
        rotController.setTolerance(1);
    }

    @Override
    public void execute() {
      if (LimelightHelpers.getTV("limelight-flash")) {
        dontSeeTagTimer.reset();
    
        // Use RobotSpace instead of BotPose if not using field-space localization
        double[] tagPose = LimelightHelpers.getTargetPose_RobotSpace("limelight-flash");
    
        // DEBUG: Show Limelight readings on SmartDashboard
        SmartDashboard.putNumber("TargetPose X (forward)", tagPose[0]);
        SmartDashboard.putNumber("TargetPose Y (left)", tagPose[1]);
        SmartDashboard.putNumber("TargetPose Yaw", tagPose[5]);
    

            double xError = tagPose[0];
            double yError = LimelightHelpers.getTX("limelight-flash") * -0.02;
            double rotError = tagPose[5];

            double maxSpeed = 0.4;
            double maxRotSpeed = 0.2;

            double xSpeed = Math.abs(xError) > 0.1 ? xController.calculate(xError) : 0;
            double ySpeed = Math.abs(yError) > 0.1 ? yController.calculate(yError) : 0;
            double rotValue = Math.abs(rotError) > 1 ? rotController.calculate(rotError) : 0;

            // Clamp speeds
            xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), maxSpeed), xSpeed);
            ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), maxSpeed), ySpeed);
            rotValue = Math.copySign(Math.min(Math.abs(rotValue), maxRotSpeed), rotValue);

            // Apply rate limiters
            xSpeed = xLimiter.calculate(xSpeed);
            ySpeed = yLimiter.calculate(ySpeed);
            rotValue = rotLimiter.calculate(rotValue);

            drivebase.setControl(fieldCentric
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotValue));

            // Reset stop timer if we aren't at setpoints
            if (!xController.atSetpoint() || !yController.atSetpoint() || !rotController.atSetpoint()) {
                stopTimer.reset();
            }

            SmartDashboard.putNumber("xError", xError);
            SmartDashboard.putNumber("yError", yError);
            SmartDashboard.putNumber("rotError", rotError);

        } else if (dontSeeTagTimer.hasElapsed(0.5)) {
            // Stop if we lose the tag for too long
            drivebase.setControl(fieldCentric
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        }

        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setControl(fieldCentric
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return dontSeeTagTimer.hasElapsed(1) || stopTimer.hasElapsed(0.3);
    }
}