package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.pidConstants.*;

public class StationaryTurn extends CommandBase {

    private final DriveSubsystem _dss;
    private final PIDController pidt = new PIDController(TP,TI,TD);
    private Pose2d startPose = new Pose2d();
    private double angle;
    private final Boolean relTurn;

    public StationaryTurn(DriveSubsystem DSS,double value,Boolean relativeTurn) {
        relTurn = relativeTurn;
        angle = value;
        _dss = DSS;
        addRequirements(_dss);
    }

    @Override
    public void initialize() {
        pidt.reset();
        startPose = _dss.getPose();
        if (relTurn) {
            angle += startPose.getRotation().getDegrees();
            while (angle > 360) angle -= 360;
            while (angle < 0) angle += 360;
        }
    } 

    @Override
    public void execute() {
        _dss.arcadeDrive(0.0, MathUtil.clamp(pidt.calculate(_dss.getHeadingError(angle),0), -1*TC,TC));
    }

    @Override
    public void end(boolean interuppted) {
        _dss.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(_dss.getHeadingError(angle)) < 0.5;
    }
}