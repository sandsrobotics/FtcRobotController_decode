package org.firstinspires.ftc.teamcode.parts.intake1;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.depricated.intake.FlipbotSettings;
import org.firstinspires.ftc.teamcode.parts.intake1.settings.Intake1TeleopSettings;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;

import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.Vector3;

public class Intake1Teleop extends LoopedPartImpl<Intake1, Intake1TeleopSettings, ObjectUtils.Null> {
    private Intake1TeleopSettings settings;
    ButtonMgr buttonMgr;

    private boolean enableDemoDriver = false;
    private boolean enableLaunch = false;


    public Intake1Teleop(Intake1 parent) {
        super(parent, "Intake teleop");
        setSettings(Intake1TeleopSettings.makeDefault(parent.parent));
        buttonMgr = parent.parent.buttonMgr;
    }

    public Intake1Teleop(Intake1 parent, Intake1TeleopSettings settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
        buttonMgr = parent.parent.buttonMgr;
    }

    public Intake1TeleopSettings getSettings() {
        return settings;
    }

    public void setSettings(Intake1TeleopSettings settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStart() {
        parent.setBaseController(() -> new Intake1Control(
                0
        ), true);
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

        if (buttonMgr.getState(1, Buttons.left_bumper, State.isPressed)) {
            enableDemoDriver = true;
            DecodeSettings.demoDriverMultiplier = DecodeSettings.demoDriverDefaultMultiplier;
        } else {
            enableDemoDriver = false;
            DecodeSettings.demoDriverMultiplier = 0;
        }

        if (buttonMgr.getState(1, Buttons.right_bumper, State.isPressed)) {
            enableDemoDriver = true;
            DecodeSettings.demoDriverMultiplier = DecodeSettings.demoDriverDefaultMultiplier;
            enableLaunch = true;
        } else {
            enableDemoDriver = false;
            DecodeSettings.demoDriverMultiplier = 0;
            enableLaunch = false;
        }
        //in telelop, disable positiontracker if Driver1 uses controls
        if (org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings.isTeleOp()) {
            if (parent.parent.opMode.gamepad1.right_trigger +
                    parent.parent.opMode.gamepad1.left_trigger +
                    parent.parent.opMode.gamepad1.left_stick_x +
                    parent.parent.opMode.gamepad1.left_stick_y +
                    parent.parent.opMode.gamepad1.right_stick_x +
                    parent.parent.opMode.gamepad1.right_stick_y != 0) {
//                parent.positionSolver.stopSolver();
            }
        }

        //todo: Read this note and fix things!
        /* **********************************************************
         *
         * IMPORTANT NOTE FROM MR. KN:
         *
         * State.isPressed should not be used when controlling tasks (or triggering anything).
         * It returns the current state of the button, not a rising or falling edge event.
         * That means it triggers the task repeatedly for the entire duration it is pressed.
         * Not good!
         *
         * Better choices would be:
         *    wasPressed;       // Rise
         *    wasReleased;      // Fall
         *    wasHeld;          // Rise + Long Hold + Fall
         *    wasTapped;        // Rise + Short Hold + Fall
         *    wasSingleTapped;  // Rise + Short Hold + Fall + Gap  [note the gap/delay at end!]
         *    wasDoubleTapped;  // Rise + Short Hold + Fall + Short Gap x2
         *
         * ***********************************************************/

        // *** DRIVER 2 CONTROLS ***
        // Driver 2 - slide control
//        parent.setUserSlidePower(-parent.parent.opMode.gamepad2.left_stick_y);
        // Driver 2 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(2, Buttons.start, State.isPressed)) {
            if (enableDemoDriver) {
                if (buttonMgr.getState(2,Buttons.x, State.wasPressed)) {
                    if (enableLaunch) {
                        parent.tasks.greenServoLaunch.restart();
                    }
                }
                if (buttonMgr.getState(2,Buttons.y, State.wasPressed)) {
                    if (enableLaunch) {
                        parent.tasks.blueServoLaunch.restart();
                    }
                }
                if (buttonMgr.getState(2,Buttons.b, State.wasPressed)) {
                    if (enableLaunch) {
                        parent.tasks.pinkServoLaunch.restart();
                    }
                }
                if (buttonMgr.getState(2,Buttons.a, State.wasPressed)) {
                    if (enableLaunch) {
                        parent.tasks.computeAndLaunchInOrder.restart();
                    }
                }
            }
        }

        // Driver 2 - start button is a "shift" key; anything below is when start is held first
        else {


        }

        // *** DRIVER 1 CONTROLS ***
        // Driver 1 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(1, Buttons.start, State.isPressed)) {
            //           DRIVER 1
            if (buttonMgr.getState(1,Buttons.dpad_up, State.wasPressed)) {
                parent.tasks.stopLaunch.restart();
            }

            if (buttonMgr.getState(1,Buttons.dpad_right, State.wasPressed)) {
                parent.tasks.startFarLaunch.restart();
            }

            if (buttonMgr.getState(1,Buttons.dpad_down, State.wasPressed)) {
                parent.autoRPM = true;
            }

            if (buttonMgr.getState(1,Buttons.dpad_left, State.wasPressed)) {
                parent.tasks.startGoalLaunch.restart();
            }

//            TODO:AUTORPM ON/OFF
//            if (eitherOrGT(Buttons.a, State.wasPressed)) {
//                parent.tasks..restart();
//            }
            if (buttonMgr.getState(1,Buttons.b, State.wasPressed)) {
                parent.tasks.outtakeTask.restart();
            }

            if (buttonMgr.getState(1,Buttons.x, State.wasPressed)) {
                parent.tasks.intakeTask.restart();
            }

            if (buttonMgr.getState(1,Buttons.y, State.wasPressed)) {
                parent.tasks.artifactIntakeStopTask.restart();
            }
        }
    }

//    public boolean eitherOrGT(Buttons button, State state) {
//        // if either was enabled and activated the control, return true
//        boolean team = buttonMgr.getState(1, button, state);
//        boolean guest = enableDemoDriver && buttonMgr.getState(2, button, state);
//        return team || guest;
//    }
}

