package org.firstinspires.ftc.teamcode.parts.intake1;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.intake1.settings.Intake1TeleopSettings;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;

import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.Vector3;

public class Intake1Teleop extends LoopedPartImpl<Intake1, Intake1TeleopSettings, ObjectUtils.Null> {
    private Intake1TeleopSettings settings;
    ButtonMgr buttonMgr;

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

            // Driver 2
            // Manual launchSpeed Tasks.
            if (buttonMgr.getState(2, Buttons.dpad_down, State.wasTapped)) {
                parent.autoRPM = false;
                parent.tasks.startFarLaunch.restart();
            }
            if (buttonMgr.getState(2, Buttons.dpad_right, State.wasTapped)) {
                parent.autoRPM = false;
                parent.tasks.startThreeLaunch.restart();
            }
            if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
                parent.autoRPM = false;
                parent.tasks.startGoalLaunch.restart();
            }
            if (buttonMgr.getState(2, Buttons.dpad_left, State.wasTapped)) {
                parent.autoRPM = false;
                parent.tasks.stopLaunch.restart();
            }

            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
//                parent.tasks.pinkServoLaunch.restart();
                parent.tasks.pinkServoLaunchInTolerance.restart();
            }
            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
//                parent.tasks.blueServoLaunch.restart();
                parent.tasks.blueServoLaunchInTolerance.restart();
            }
            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
//                parent.tasks.greenServoLaunch.restart();
                parent.tasks.greenServoLaunchInTolerance.restart();
            }
            /*      Launch Tasks          */
            if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
//                parent.tasks.pinkBlueGreenServoLaunch.restart();
//                parent.tasks.allServoLaunch.restart();
                parent.tasks.allServoLaunchInTolerance.restart();
            }
            /*      Launch ALL Servos Tasks          */
            if (buttonMgr.getState(2, Buttons.a, State.wasHeld)) {
                parent.tasks.computeAndLaunchInOrder.restart();
            }

//            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasTapped)) {
//                parent.toggleAutoRPM();
//            }
//            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasPressed)) {
//                parent.autoRPM = true;
//                parent.launchOff = false;
//            }
//            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasReleased)) {
//                parent.autoRPM = false;
//                parent.setLaunchMotors(0);
//            }
            // todo: does driver2 prefer press/release or tap/hold for autoRPM?
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasTapped)) {
                parent.autoRPM = true;
                parent.launchOff = false;
            }
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasHeld)) {
                parent.autoRPM = false;
                parent.setLaunchMotors(0);
            }
            if (buttonMgr.getState(2, Buttons.left_bumper, State.wasPressed)) {
                parent.headingSolver.startSolver();
            }
            if (buttonMgr.getState(2, Buttons.left_bumper, State.wasReleased)) {
                parent.headingSolver.stopSolver();
            }
        }

        // Driver 2 - start button is a "shift" key; anything below is when start is held first
        else {


        }

        // *** DRIVER 1 CONTROLS ***
        // Driver 1 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(1, Buttons.start, State.isPressed)) {
            //           DRIVER 1
            //             intake Task
            if (buttonMgr.getState(1, Buttons.right_bumper, State.wasTapped)) {
                parent.tasks.intakeTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.left_bumper, State.wasTapped)) {
                parent.tasks.artifactIntakeStopTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
                parent.tasks.outtakeTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
                parent.tasks.viewObelisk.restart();
            }
            if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
                parent.tasks.allServoDock.restart();
            }
            if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
                parent.tasks.allServoStore.restart();
            }

            // Pre-Selected moveToLoadingZone.
            if (buttonMgr.getState(1, Buttons.left_stick_button, State.wasTapped)) {
                if(DecodeSettings.isAllianceRed()) {
                    parent.tasks.teleopMoveToRedLoadingZone.restart();
                } else {
                    parent.tasks.teleopMoveToBlueLoadingZone.restart();
                }
            }

            // Pre-Selected teleopFarLaunch
            if (buttonMgr.getState(1, Buttons.right_stick_button, State.wasTapped)) {
                if(DecodeSettings.isAllianceRed()) {
                    parent.tasks.teleopFarRedLaunch.restart();
                } else {
                    parent.tasks.teleopFarBlueLaunch.restart();
                }
            }

                // Driver 1
        }
            // Driver 1 - start button is a "shift" key; anything below is when start is held first
        else {
            // add shifted controls here

            // LK new test
            if (DecodeSettings.lkTestMode1) {
                if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
                    parent.headingSolver.startSolver();
                }
                if (buttonMgr.getState(1, Buttons.dpad_left, State.wasTapped)) {
                    parent.headingSolver.setNewTarget(DecodeSettings.targetBlue, true);
                    parent.storedTarget = DecodeSettings.targetBlue;
                }
                if (buttonMgr.getState(1, Buttons.dpad_right, State.wasTapped)) {
                    parent.headingSolver.setNewTarget(DecodeSettings.targetRed, true);
                    parent.storedTarget = DecodeSettings.targetRed;
                }
                if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
                    parent.headingSolver.stopSolver();
                }
            }
            if (DecodeSettings.lkTestMode2) {
                if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
                    parent.headerAimer.setAutoAim(true);
                }
                if (buttonMgr.getState(1, Buttons.dpad_left, State.wasTapped)) {
                    parent.headerAimer.setTarget(DecodeSettings.targetBlue);
                    parent.storedTarget = DecodeSettings.targetBlue;
                }
                if (buttonMgr.getState(1, Buttons.dpad_right, State.wasTapped)) {
                    parent.headerAimer.setTarget(DecodeSettings.targetRed);
                    parent.storedTarget = DecodeSettings.targetRed;
                }
                if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
                    parent.headerAimer.setAutoAim(false);
                }
            }
            if (DecodeSettings.lkTestMode1 || DecodeSettings.lkTestMode2) {
                if (buttonMgr.getState(1, Buttons.y, State.wasDoubleTapped)) {
                    parent.limeLight.applyTransform();
//                    parent.positionTracker.setOverrideTransform(parent.limeLight.llSavedTransform);
                }
                if (buttonMgr.getState(1, Buttons.x, State.wasDoubleTapped)) {
                    parent.limeLight.toggleAuto();
                }
                if (buttonMgr.getState(1, Buttons.left_bumper, State.wasDoubleTapped)) {
                    parent.limeLight.acceptableStdDev = new Vector3(1,1,1);
                    parent.limeLight.setSizeOfBuffer(50);
                }
                if (buttonMgr.getState(1, Buttons.right_bumper, State.wasDoubleTapped)) {
                    parent.limeLight.acceptableStdDev = new Vector3(2,2,2);
                    parent.limeLight.setSizeOfBuffer(25);
                }
                if (buttonMgr.getState(1, Buttons.right_stick_button, State.wasTapped)) {
                    parent.toggleAutoRPM();
                }
            }
        }
    }
}