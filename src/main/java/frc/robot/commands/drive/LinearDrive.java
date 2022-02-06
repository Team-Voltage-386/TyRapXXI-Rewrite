package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.pidConstants.*;

public class LinearDrive extends CommandBase {

    private final DriveSubsystem _dss;
    private final PIDController pidt = new PIDController(TP,TI,TD);
    private final PIDController pidd = new PIDController(DP, DI, DD);
    private Pose2d startPose = new Pose2d();
    private double headingHold = 0;
    private double distanceFromStart = 0;
    private final double targetDistance;
    

    public LinearDrive(DriveSubsystem DSS,double distance) {
        targetDistance = distance;
        _dss = DSS;
        addRequirements(_dss);
    }

    @Override
    public void initialize() {
        pidt.reset();
        pidd.reset();
        startPose = _dss.getPose();
        headingHold = startPose.getRotation().getDegrees();
    } 

    @Override
    public void execute() {
        distanceFromStart = startPose.getTranslation().getDistance(_dss.getPose().getTranslation());
        _dss.arcadeDrive(MathUtil.clamp(pidd.calculate(distanceFromStart,targetDistance),-1*DC,DC), MathUtil.clamp(pidt.calculate(_dss.getHeadingError(headingHold),0), -1*TC,TC));
    }

    @Override
    public void end(boolean interuppted) {
        _dss.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(distanceFromStart-targetDistance) < 0.05;
    }
}