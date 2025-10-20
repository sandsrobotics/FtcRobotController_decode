package org.firstinspires.ftc.teamcode.parts.intake;

public class IntakeControl {
    public int sweeperPower;
    public int sweepLiftPosition;
    public int sweepSlidePosition;
    public int bucketLiftPosition;
    public int pinchPosition;
    public int v_SlidePosition;
    public int intakeAngleSupplier;

    // todo: eliminate this like the others

    public IntakeControl(int sweeperPower, int sweeperLiftPosition, int sweepSlidePosition, int bucketLiftPosition, //int intakePosition,
                         int pinchPosition, int v_SlidePosition, int intakeAngleSupplier){

        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
        this.sweepSlidePosition = sweepSlidePosition;
        this.bucketLiftPosition = bucketLiftPosition;
        this.pinchPosition = pinchPosition;
        this.v_SlidePosition = v_SlidePosition;
        this.intakeAngleSupplier = intakeAngleSupplier;
        }
}