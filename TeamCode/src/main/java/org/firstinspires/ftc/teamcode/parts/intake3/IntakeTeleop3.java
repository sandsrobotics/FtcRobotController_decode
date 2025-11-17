package org.firstinspires.ftc.teamcode.parts.intake3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeTeleopSettings3;
import om.self.ezftc.core.part.LoopedPartImpl;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class IntakeTeleop3 extends LoopedPartImpl<Intake3, IntakeTeleopSettings3, ObjectUtils.Null> {
    private IntakeTeleopSettings3 settings;
    ButtonMgr buttonMgr;

    public IntakeTeleop3(Intake3 parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeTeleopSettings3.makeDefault(parent.parent));
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleop3(Intake3 parent, IntakeTeleopSettings3 settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleopSettings3 getSettings() {
        return settings;
    }

    public void setSettings(IntakeTeleopSettings3 settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {}

    @Override
    public void onStart() {
        parent.setBaseController(() -> new IntakeControl3(0, false, 0, false), true);
        parent.getHardware().launchMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, IntakeTeleopSettings3.spinnerPID);
    }

    @Override
    public void onRun() {
        driverControls();
    }

    @Override
    public void onStop() {
        parent.setBaseControllerToDefault(parent.isControlActive());
    }

    public void driverControls() {
        // e-stop, either driver
        if (buttonMgr.getState(2, Buttons.back, State.wasPressed) ||
                buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }
        if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
            double targetTPS = (IntakeTeleopSettings3.intakeRPM / 60.0) * IntakeTeleopSettings3.ticksPerRev;
            parent.getHardware().intakeMotor.setVelocity(targetTPS);
        }
        if (buttonMgr.getState(1, Buttons.x, State.wasDoubleTapped)){
            parent.getHardware().intakeMotor.setVelocity(0);;
        }
        if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
            double targetTPS = (IntakeTeleopSettings3.launchRPM / 60.0) * IntakeTeleopSettings3.ticksPerRev;
            parent.getHardware().launchMotor.setVelocity(targetTPS);
        }
        if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
            parent.getHardware().launchMotor.setVelocity(0);;
        }
//        if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
//            parent.getHardware().launchServo.setPosition(IntakeTeleopSettings3.servoPosition);
//        }
//        if (buttonMgr.getState(1, Buttons.b, State.wasDoubleTapped)) {
//            parent.getHardware().launchServo.setPosition(0);
//        }
        parent.getHardware().launchServo.setPosition(parent.parent.opMode.gamepad1.right_trigger);
    }
}
