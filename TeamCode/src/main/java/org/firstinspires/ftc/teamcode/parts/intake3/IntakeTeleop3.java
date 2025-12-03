package org.firstinspires.ftc.teamcode.parts.intake3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeTeleopSettings3;
import om.self.ezftc.core.part.LoopedPartImpl;

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
            parent.setIntakeRPM(IntakeTeleopSettings3.intakeRPM);
        }
        if (buttonMgr.getState(1, Buttons.x, State.wasDoubleTapped)){
            parent.setIntakeRPM(0);
        }
        if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
            parent.setLaunchRPM(IntakeTeleopSettings3.launchRPM);
        }
        if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
            parent.setLaunchRPM(0);
        }
        parent.getHardware().launchServo.setPosition(parent.parent.opMode.gamepad1.right_trigger);
    }
}
