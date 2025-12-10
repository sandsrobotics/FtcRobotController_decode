package org.firstinspires.ftc.teamcode.parts.Team1.Intake1;

public class IntakeControl1 {
    public double launchSpeed;
    public boolean intakeState;
    public int launchFlipPosition;
    public boolean robotEStop;

    public IntakeControl1(double launchSpeed, boolean intakeState, int launchFlipPosition, boolean robotEStop) {
        this.launchSpeed = launchSpeed;
        this.intakeState = intakeState;
        this.launchFlipPosition = launchFlipPosition;
        this.robotEStop = robotEStop;
    }
}