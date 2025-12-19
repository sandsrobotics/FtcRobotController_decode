package org.firstinspires.ftc.teamcode.parts.decode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware {
    public final DcMotorEx            intakeMotor;
    public final ServoSSR             sorterServo;
    public final ServoSSR             pinkServo;
    public final ServoSSR             blueServo;
    public final ServoSSR             greenServo;
    public final DcMotorEx            launchMotorLeft;
    public final DcMotorEx            launchMotorRight;






    public IntakeHardware(DcMotorEx intakeMotor, ServoSSR sorterServo, ServoSSR pinkServo, ServoSSR blueServo,
                          ServoSSR greenServo, DcMotorEx launchMotorLeft, DcMotorEx launchMotorRight) {
        this.intakeMotor = intakeMotor;
        this.sorterServo = sorterServo;
        this.pinkServo   = pinkServo;
        this.blueServo   = blueServo;
        this.greenServo  = greenServo;
        this.launchMotorLeft = launchMotorLeft;
        this.launchMotorRight = launchMotorRight;





        DcMotorEx[] motors = {this.intakeMotor};
        for(DcMotorEx motor : motors){
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }
//beans
    public static IntakeHardware makeDefault(HardwareMap hardwareMap)  {
        MotorSettings intakeMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER);
        MotorSettings launchMotorLeftSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorEx.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.FLOAT, DcMotorEx.RunMode.RUN_USING_ENCODER);
        MotorSettings launchMotorRightSettings = new MotorSettings(MotorSettings.Number.TWO_B, DcMotorEx.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.FLOAT, DcMotorEx.RunMode.RUN_USING_ENCODER);

        return new IntakeHardware(
                intakeMotorSettings.makeExMotor(hardwareMap),
                new ServoSSR(hardwareMap.get(Servo.class,"servo0B")),
                new ServoSSR(hardwareMap.get(Servo.class,"servo0")),
                new ServoSSR(hardwareMap.get(Servo.class,"servo1")),
                new ServoSSR(hardwareMap.get(Servo.class,"servo2")),
                launchMotorLeftSettings.makeExMotor(hardwareMap),
                launchMotorRightSettings.makeExMotor(hardwareMap)
                );
    }
}