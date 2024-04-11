package frc.robot.utilities;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShuffleboardDisplay {
    private final ShuffleboardTab armTab;
    private final ShuffleboardTab intakeTab;

    private final GenericEntry intakePosition;
    private final GenericEntry intakeVelocity;
    private final GenericEntry armPosition;
    private final GenericEntry armVelocity;
    private static ShuffleboardDisplay instance = null;

    private ShuffleboardDisplay() {
        this.armTab = Shuffleboard.getTab("Arm");
        this.armPosition = this.armTab.add("Arm Position (Reveloutions)", 0.0)
        .withPosition(0, 2)
        .withSize(2,1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
        this.armVelocity = this.armTab.add("Arm Velocity (RPM)", 0.0)
        .withPosition(0, 2)
        .withSize(2,1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
        this.intakeTab = Shuffleboard.getTab("Intake");
        this.intakePosition = this.intakeTab.add("Intake Position (Reveloutions)", 0.0)
        .withPosition(0, 2)
        .withSize(2,1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
        this.intakeVelocity = this.intakeTab.add("Intake Velocity (RPM)", 0.0)
        .withPosition(0, 2)
        .withSize(2,1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    }
    public static ShuffleboardDisplay getInstance() {
        if (instance == null) {
            instance = new ShuffleboardDisplay();
        }
        return instance;
    }
    public GenericEntry getArmPosition() {
        return this.armPosition;
    }
    public GenericEntry getArmVelocity() {
        return this.armVelocity;
    }
    public GenericEntry getIntkePosition() {
        return this.intakePosition;
    }
    public GenericEntry getIntakeVelocity() {
        return this.intakeVelocity;
    }



} 