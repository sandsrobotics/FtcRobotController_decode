package org.firstinspires.ftc.teamcode.parts.decode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware {
    public final DcMotorEx            intakeMotor;
    public final ServoSSR             intakeServo;
    public IntakeHardware(DcMotorEx intakeMotor, ServoSSR intakeServo) {
        this.intakeMotor = intakeMotor;
        this.intakeServo = intakeServo;

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

        return new IntakeHardware(
                intakeMotorSettings.makeExMotor(hardwareMap),
                new ServoSSR(hardwareMap.get(Servo.class,"servo0"))
        );
    }
}