package org.firstinspires.ftc.teamcode.parts.intake3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import om.self.ezftc.core.part.LoopedPartImpl;

//@Config
public class IntakeTeleop3 extends LoopedPartImpl<Intake3, IntakeSettings3, ObjectUtils.Null> {
    private IntakeSettings3 settings;
    ButtonMgr buttonMgr;

    public IntakeTeleop3(Intake3 parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeSettings3.makeDefault());
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleop3(Intake3 parent, IntakeSettings3 settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeSettings3 getSettings() {
        return settings;
    }

    public void setSettings(IntakeSettings3 settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {}

    @Override
    public void onStart() {
        parent.setBaseController(() -> new IntakeControl3(false), true);
        parent.getHardware().launchMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, IntakeSettings3.spinnerPID);
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

        // ============================================
        // CONTROLLER 1 (DRIVER) CONTROLS
        // ============================================

        // BACK BUTTON - Emergency stop
        if (buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

        // D-PAD DOWN - Start intake (collect artifacts)
        if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
            parent.setIntakeRPM(IntakeSettings3.intakeRPM);
        }

        // D-PAD UP - Reverse intake (eject artifacts)
        if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
            parent.setIntakeRPM(-IntakeSettings3.intakeRPM);
        }

        // D-PAD DOWN/UP DOUBLE TAP - Stop intake
        if (buttonMgr.getState(1, Buttons.dpad_down, State.wasDoubleTapped) ||
                buttonMgr.getState(1, Buttons.dpad_up, State.wasDoubleTapped)) {
            parent.setIntakeRPM(0);
        }

        // B BUTTON - test and find out.
        if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
            IntakeSettings3.launchArmed = false;
            parent.setLaunchRPM(0);
        }


//
//        if (buttonMgr.getState(1, Buttons.left_trigger, State.wasPressed)) {
//            parent.getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Rest);
//            parent.getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Rest);
//            parent.getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Rest);
//        }
//
//        if (buttonMgr.getState(1, Buttons.right_trigger, State.wasPressed)) {
//            parent.getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Launch);
//            parent.getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Launch);
//            parent.getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Launch);
//        }

        // ============================================
        // CONTROLLER 2 (OPERATOR) CONTROLS
        // ============================================

        // BACK BUTTON - Emergency stop
        if (buttonMgr.getState(2, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

        // LEFT BUMPER - Auto-align to April tag
        if (buttonMgr.getState(2, Buttons.left_bumper, State.isHeld)) {
            IntakeSettings3.alignTarget = true;
        } else {
            IntakeSettings3.alignTarget = false;
        }

        // RIGHT TRIGGER - Color-ordered launch (auto-starts launcher)
        if (buttonMgr.getState(2, Buttons.right_trigger, State.wasPressed)) {
            parent.computeLaunchOrderAndLaunchBlocking(
                    parent.limeLight.getClassificationPattern()
            );
            parent.getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Lock);
        }

        // Y BUTTON - Start/Stop launcher
        if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
            parent.setLaunchRPM(IntakeSettings3.launchRPM);
            IntakeSettings3.launchArmed = true;  // use true for interpolated
        }

        if (buttonMgr.getState(2, Buttons.y, State.wasDoubleTapped)) {
            IntakeSettings3.launchArmed = false;
            parent.setLaunchRPM(0);
        }

        // X BUTTON - Move to blueshoot1 and launch (auto-starts launcher)
        if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
            if (parent.getTargetLaunchRPM() < 500) {
                parent.setLaunchRPM(IntakeSettings3.launchRPM);
            }
            parent.launchData = IntakeSettings3.launchPosiMap.get("blueshoot1");
            parent.tasks.moveAndLaunch.restart();
        }

        // B BUTTON - Simultaneous launch (auto-starts launcher)
        if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
            if (parent.getTargetLaunchRPM() < 500) {
                parent.setLaunchRPM(IntakeSettings3.launchAllRPM);
                IntakeSettings3.launchArmed = true;
            }
            parent.stopAllIntakeTasks();
            parent.tasks.sameTimeBallLaunchTask.restart();
        }

        // A BUTTON - Sequential launch (auto-starts launcher)
        if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
            if (parent.getTargetLaunchRPM() < 500) {
                parent.setLaunchRPM(IntakeSettings3.launchRPM);
                IntakeSettings3.launchArmed = true;

            }
            parent.stopAllIntakeTasks();
            parent.tasks.ballLaunchTask.restart();
        }

        // X BUTTON - Reverse shooter
        if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
            parent.setLaunchRPM(-IntakeSettings3.launchRPM);
        }

        // B BUTTON - Lock servo (tap to lock, double tap to unlock)
        if (buttonMgr.getState(2, Buttons.right_stick_button, State.wasTapped)) {
            parent.getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Lock);
        }

        if (buttonMgr.getState(2, Buttons.right_stick_button, State.wasDoubleTapped)) {
            parent.getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Unlock);
        }
        // LEFT STICK BUTTON - Move to bluefartriangle and launch (auto-starts launcher)
//        if (buttonMgr.getState(2, Buttons.left_stick_button, State.wasTapped)) {
//            if (parent.getTargetLaunchRPM() < 500) {
//                parent.setLaunchRPM(IntakeSettings3.launchRPM);
//                IntakeSettings3.launchArmed = true;
//            }
//            parent.launchData = IntakeSettings3.launchPosiMap.get("bluefartriangle");
//            parent.tasks.moveAndLaunch.restart();
//        }
    }
}