package org.firstinspires.ftc.teamcode.parts.intake3.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.lib.ServoSSR;
import om.self.ezftc.utils.hardware.motor.MotorSettings;

public class IntakeHardware3 {
    public final DcMotorEx intakeMotor;
    public final DcMotorEx launchMotor;
    public final ServoSSR launchServo;
    public final ServoSSR pixel;

    public IntakeHardware3(DcMotorEx intakeMotor, DcMotorEx launchMotor, ServoSSR launchServo, ServoSSR pixel) {
        this.intakeMotor = intakeMotor;
        this.launchMotor = launchMotor;
        this.launchServo = launchServo;
        this.pixel = pixel;

        DcMotorEx[] motors = {this.intakeMotor, this.launchMotor};
        for(DcMotorEx motor : motors){
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }
    //beans
    public static IntakeHardware3 makeDefault(HardwareMap hardwareMap)  {
        MotorSettings intakeMotorSettings =new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 1);
        MotorSettings launchMotorSettings = new MotorSettings(MotorSettings.Number.THREE_B, DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 1);

        return new IntakeHardware3(
            intakeMotorSettings.makeExMotor(hardwareMap),
            launchMotorSettings.makeExMotor(hardwareMap),
            new ServoSSR(hardwareMap.get(Servo.class,"servo0")),
            new ServoSSR(hardwareMap.get(Servo.class,"servo5"))
        );
    }
}