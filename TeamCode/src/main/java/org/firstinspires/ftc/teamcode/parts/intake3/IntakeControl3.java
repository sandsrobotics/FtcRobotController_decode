package org.firstinspires.ftc.teamcode.parts.intake3;

public class IntakeControl3 {
    public double launchSpeed;
    public boolean intakeState;
    public int launchFlipPosition;
    public boolean robotEStop;

    public IntakeControl3(double launchSpeed, boolean intakeState, int launchFlipPosition, boolean robotEStop) {
        this.launchSpeed = launchSpeed;
        this.intakeState = intakeState;
        this.launchFlipPosition = launchFlipPosition;
        this.robotEStop = robotEStop;
    }
}