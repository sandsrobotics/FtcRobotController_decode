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

    // NEW: Servo selector
    private int selectedServo = 0;   // 0, 1, or 2

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

        // e-stop
        if (buttonMgr.getState(2, Buttons.back, State.wasPressed) ||
                buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

        // intake/launch stuff
        if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
            parent.setIntakeRPM(IntakeTeleopSettings3.intakeRPM);
        }
        if (buttonMgr.getState(1, Buttons.x, State.wasDoubleTapped)) {
            parent.setIntakeRPM(0);
        }
        if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
            parent.setLaunchRPM(IntakeTeleopSettings3.launchRPM);
        }
        if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
            parent.setLaunchRPM(0);
        }


        // left bumper : cycle through servos 0 → 1 → 2 → 0
        if (buttonMgr.getState(1, Buttons.left_bumper, State.wasTapped)) {
            selectedServo = (selectedServo + 1) % 3;  // wrap
        }

        // right bumper : activate selected servo
        if (buttonMgr.getState(1, Buttons.right_bumper, State.wasTapped)) {
            switch (selectedServo) {
                case 0:
                    parent.getHardware().launchServo0.setPosition(IntakeTeleopSettings3.servoPosition);
                    break;

                case 1:
                    parent.getHardware().launchServo1.setPosition(IntakeTeleopSettings3.servoPosition);
                    break;

                case 2:
                    parent.getHardware().launchServo2.setPosition(IntakeTeleopSettings3.servoPosition);
                    break;
            }
        }
        // right bumper : activate selected servo
        if (buttonMgr.getState(1, Buttons.right_bumper, State.wasDoubleTapped)) {
            switch (selectedServo) {
                case 0:
                    parent.getHardware().launchServo0.setPosition(0);
                    break;

                case 1:
                    parent.getHardware().launchServo1.setPosition(0);
                    break;

                case 2:
                    parent.getHardware().launchServo2.setPosition(0);
                    break;
            }
        }
    }
}