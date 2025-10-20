package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class IntakeTasks {
    public final Group intakeTasksGroup;
    public final TimedTask autonomousSampleTask;
    public final TimedTask specAutoIntakePickupTask;
    public final TimedTask specAutoIntakeDepositTask;
    public final TimedTask moveToPickupSpecimenTask;
    public final TimedTask moveToHangSpecimenTask;
    public final TimedTask autoHomeTask;
    public final TimedTask autoHomeTaskLift;
    public final TimedTask autoHomeTaskSlide;
    public final TimedTask autoDepositTask;
    public final TimedTask autoDepositSample3;
    public final TimedTask prepareToIntakeTask;
    public final TimedTask dockTask;
    public final TimedTask transferTask;
    public final TimedTask lowDumpIntakeTask;
    public final TimedTask prepareToGetSpecimenTask;
    public final TimedTask getSpecimenTask;
    public final TimedTask hangSpecimenTask;
    public final TimedTask prepareToHangSpecimenTask;
    public final TimedTask prepareToDepositTask;
    public final TimedTask depositTask;
    public final TimedTask autoIntakeTask;
    public final TimedTask prepareToTransferTask;
    public final TimedTask checkSampleTask;
    public final TimedTask ejectBadSampleTask;
    public final TimedTask prepareToHangRobotTask;
    public final TimedTask hangRobotTask;
    private final Intake intake;
    private final Robot robot;

    public IntakeTasks(Intake intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        intakeTasksGroup = new Group("intake", intake.getTaskManager());
        autonomousSampleTask = new TimedTask(TaskNames.autonomousSample, intakeTasksGroup);
        specAutoIntakePickupTask = new TimedTask(TaskNames.specAutoIntakePickup, intakeTasksGroup);
        specAutoIntakeDepositTask = new TimedTask(TaskNames.specAutoIntakeDeposit, intakeTasksGroup);
        moveToPickupSpecimenTask = new TimedTask(TaskNames.moveToPickupSpecimen, intakeTasksGroup);
        moveToHangSpecimenTask = new TimedTask(TaskNames.moveToHangSpecimen, intakeTasksGroup);
        autoHomeTask = new TimedTask(TaskNames.autoHome, intakeTasksGroup);
        autoHomeTaskLift = new TimedTask("autoHomeLift", intakeTasksGroup);
        autoHomeTaskSlide = new TimedTask("autoHomeSlide", intakeTasksGroup);
        autoDepositTask = new TimedTask("autoDepositTask", intakeTasksGroup);
        autoDepositSample3 = new TimedTask("autoDepositSample3", intakeTasksGroup);
        prepareToIntakeTask = new TimedTask(TaskNames.prepareToIntake, intakeTasksGroup);
        dockTask = new TimedTask(TaskNames.safe, intakeTasksGroup);
        transferTask = new TimedTask(TaskNames.transfer, intakeTasksGroup);
        lowDumpIntakeTask = new TimedTask(TaskNames.lowDumpIntake, intakeTasksGroup);
        prepareToGetSpecimenTask = new TimedTask(TaskNames.prepareToGetSpecimen, intakeTasksGroup);
        getSpecimenTask = new TimedTask(TaskNames.getSpecimen, intakeTasksGroup);
        hangSpecimenTask = new TimedTask(TaskNames.hangSpecimen, intakeTasksGroup);
        prepareToHangSpecimenTask = new TimedTask(TaskNames.prepareToHangSpecimen, intakeTasksGroup);
        prepareToDepositTask = new TimedTask(TaskNames.prepareToDeposit, intakeTasksGroup);
        depositTask = new TimedTask(TaskNames.deposit, intakeTasksGroup);
        autoIntakeTask = new TimedTask(TaskNames.autoIntake, intakeTasksGroup);
        prepareToTransferTask = new TimedTask(TaskNames.prepareToTransfer, intakeTasksGroup);
        checkSampleTask = new TimedTask(TaskNames.checkSample, intakeTasksGroup);
        ejectBadSampleTask = new TimedTask(TaskNames.ejectBadSample, intakeTasksGroup);
        prepareToHangRobotTask = new TimedTask(TaskNames.prepareToHangRobotTask, intakeTasksGroup);
        hangRobotTask = new TimedTask(TaskNames.hangRobotTask, intakeTasksGroup);
    }

    public void startAutoHome() { autoHomeTask.restart(); }

    public void constructAllIntakeTasks() {

        /* == Task: autonomousSampleTask == */
        autonomousSampleTask.autoStart = false;
        autonomousSampleTask.addStep(() -> {
            intake.getHardware().flipper.setPosition(intake.getSettings().flipperAutoSample);  //flipperAlmostFloor
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerIn);
        });
        autonomousSampleTask.addStep(() -> intake.getHardware().flipper.isDone());
        // 20250219 try disabling flipper servo like done in teleop; remove if trouble (does this help with new bouncing/rising problem?)
        autonomousSampleTask.addStep(() -> intake.getHardware().flipper.disable());
        autonomousSampleTask.addStep(() -> intake.setSlidePosition(intake.getSettings().autoSampleSlideDistanceTest, 1));
//        autonomousSampleTask.addStep(intake::isSlideInTolerance); // todo: consider a timeout in case it jams against the field border at Sample3
        autonomousSampleTask.addTimedStep(() -> {}, () -> intake.isSamplePresent() || intake.isSlideInTolerance(), 1000);
//        autonomousSampleTask.addDelay(250); //500
        autonomousSampleTask.addTimedStep(() -> {}, intake::isSamplePresent, 250);
        autonomousSampleTask.addStep(() -> intake.setSlidePosition(intake.getSettings().autoSampleSlideMin, 1));
//        autonomousSampleTask.addStep(intake::isSlideInTolerance);
        // start new stuff - comment out if not working or worse and uncomment previous line
//        autonomousSampleTask.addStep(() -> intake.isSlideInTolerance() || (intake.sampleDistance() < 1.5));
//        autonomousSampleTask.addStep(() -> intake.setSlidePosition(intake.getSettings().autoSampleSlideDistance, 1));
//        autonomousSampleTask.addStep(() -> intake.isSlideInTolerance() || (intake.sampleDistance() < 1.5));
//        autonomousSampleTask.addStep(() -> intake.setSlidePosition(intake.getSettings().positionSlideMin, 1));
//        autonomousSampleTask.addStep(() -> intake.isSlideInTolerance() || (intake.sampleDistance() < 1.5));
        // revised steps to bypass actions if sample is/was detected
        autonomousSampleTask.addTimedStep(() -> {}, () -> intake.isSamplePresent() || intake.isSlideInTolerance(), 1000);
        autonomousSampleTask.addStep(() -> {
            if (!intake.wasSamplePresent()) intake.setSlidePosition(intake.getSettings().autoSampleSlideDistanceTest, 1);
        });
        autonomousSampleTask.addTimedStep(() -> {}, () -> intake.wasSamplePresent() || intake.isSamplePresent() || intake.isSlideInTolerance(), 1500);
        autonomousSampleTask.addStep(() -> {
            if (!intake.wasSamplePresent()) intake.setSlidePosition(intake.getSettings().autoSampleSlideMin, 1);
        });
        autonomousSampleTask.addTimedStep(() -> {}, () -> intake.wasSamplePresent() || intake.isSamplePresent() || intake.isSlideInTolerance(), 1500);
        // end new stuff
        autonomousSampleTask.addStep(prepareToTransferTask::restart);
        autonomousSampleTask.addStep(prepareToTransferTask::isDone);
        // todo: comment the following two lines!? ****************
//        autonomousSampleTask.addDelay(250);
//        autonomousSampleTask.addStep(transferTask::isDone);
        /* todo: This needs more testing, improvement */

        specAutoIntakePickupTask.autoStart = false;
        specAutoIntakePickupTask.addStep(() -> {
            intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor);
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerIn);
        });
        specAutoIntakePickupTask.addStep(() -> intake.getHardware().flipper.isDone());
        specAutoIntakePickupTask.addStep(() -> intake.setSlidePosition(intake.getSettings().positionSlideAutoSamplePickup, 1));
        specAutoIntakePickupTask.addTimedStep(() -> {}, () -> intake.isSamplePresent() || intake.isSlideInTolerance(), 500);
        specAutoIntakePickupTask.addTimedStep(() -> {}, intake::isSamplePresent, 250);
        specAutoIntakePickupTask.addStep(() -> intake.setSlidePosition(intake.getSettings().positionSlideAutoSampleRetract, 1));

        specAutoIntakeDepositTask.autoStart = false;
        specAutoIntakeDepositTask.addStep(() -> intake.setSlidePosition(intake.getSettings().positionSlideAutoSampleDeposit, 1));
        specAutoIntakeDepositTask.addTimedStep(() -> {}, intake::isSlideInTolerance, 500);
        specAutoIntakeDepositTask.addStep(() -> {
            intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor);
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOut);
        });
        specAutoIntakeDepositTask.addStep(() -> intake.getHardware().flipper.isDone());
//        specAutoIntakeDepositTask.addDelay(400);
        specAutoIntakeDepositTask.addTimedStep(() -> {}, () -> intake.readSampleDistance() >= intake.getSettings().distSampleEmpty, 500);
        specAutoIntakeDepositTask.addStep(() -> intake.setSlidePosition(intake.getSettings().positionSlideAutoSampleRetract, 1));
        specAutoIntakeDepositTask.addStep(() -> intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff));

        // Sample Three Deposit Task

        autoDepositSample3.autoStart = false;
        autoDepositSample3.addStep(intake::initSampleLiftPID);
        autoDepositSample3.addStep(() -> {
            intake.getHardware().chute.setPosition(intake.getSettings().chuteReady);
            intake.setLiftPosition(intake.getSettings().positionLiftMax, 1);
        });
//        depositTask.addStep(() -> intake.isLiftInTolerance() && intake.getHardware().chute.isDone());
        autoDepositSample3.addStep(() -> intake.getLiftPosition() > intake.getSettings().positionLiftPreDump);
        autoDepositSample3.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteSampleDeposit));
        autoDepositSample3.addStep(() -> intake.getHardware().chute.isDone());
        autoDepositSample3.addDelay(400); // 200
        autoDepositSample3.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteParked));
        autoDepositSample3.addStep(() -> intake.getHardware().chute.isDone());

        // Move to PickupSpecimen Position - for use in Teleop.
        moveToPickupSpecimenTask.autoStart = false;
        moveToPickupSpecimenTask.addStep(() -> intake.positionSolver.setNewTarget(intake.p_nearObsZone.withZ(0), true));
        moveToPickupSpecimenTask.addDelay(150);
        moveToPickupSpecimenTask.addStep(() -> intake.positionSolver.setNewTarget(intake.p_nearObsZone, true));
        moveToPickupSpecimenTask.addStep(() -> intake.positionSolver.isDone());

        // Move to HangSpecimen Position - for use in Teleop.
        moveToHangSpecimenTask.autoStart = false;
        moveToHangSpecimenTask.addStep(() -> intake.positionSolver.setNewTarget(intake.p_beforeHighRung.withZ(0), true));
        moveToHangSpecimenTask.addDelay(150);
        moveToHangSpecimenTask.addStep(() -> intake.positionSolver.setNewTarget(intake.p_beforeHighRung, true));
        moveToHangSpecimenTask.addStep(() -> intake.positionSolver.isDone());

        /* == Task: prepareToIntakeTask == */
        prepareToIntakeTask.autoStart = false;
        prepareToIntakeTask.addStep(() -> {
            intake.getHardware().flipper.setPosition(intake.getSettings().flipperSafe);
            intake.setSlidePosition(intake.getSettings().positionSlideStartIntake, 1);
        });
        prepareToIntakeTask.addStep(intake::isSlideInTolerance);
        prepareToIntakeTask.addStep(() -> intake.getHardware().flipper.setPosition(intake.getSettings().flipperAlmostFloor));
        prepareToIntakeTask.addStep(() -> intake.getHardware().flipper.isDone() );

        /* == Task: prepareToGetSpecimenTask == */
        prepareToGetSpecimenTask.autoStart = false;
        prepareToGetSpecimenTask.addStep(intake::initSpecLiftPID);
        prepareToGetSpecimenTask.addStep(() -> intake.setSlidePosition(intake.getSettings().positionSlideSpecimen, 0.2));
        prepareToGetSpecimenTask.addStep(() -> {
            intake.setLiftPosition(intake.getSettings().positionLiftGetSpecimen, 1);
            intake.getHardware().pinch.setPosition(intake.getSettings().pinchFullOpen);
        });
        prepareToGetSpecimenTask.addStep(intake::isLiftInTolerance);
        prepareToGetSpecimenTask.addStep(() -> intake.getHardware().pinch.isDone());

        /* == Task: getSpecimenTask == */
        getSpecimenTask.autoStart = false;
        getSpecimenTask.addStep(intake::initSpecLiftPID);
        getSpecimenTask.addStep(() -> intake.setSlidePosition(intake.getSettings().positionSlideSpecimen, 0.2));
        getSpecimenTask.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchClosed));
        getSpecimenTask.addStep(() -> intake.getHardware().pinch.isDone());
        getSpecimenTask.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftRaiseSpecimen, 1.0)); //0.7
        getSpecimenTask.addStep(intake::isLiftInTolerance);

        /* == Task: prepareToHangSpecimenTask == */
        prepareToHangSpecimenTask.autoStart = false;
        prepareToHangSpecimenTask.addStep(intake::initSpecLiftPID);
        prepareToHangSpecimenTask.addStep(() -> intake.setSlidePosition(intake.getSettings().positionSlideSpecimen, 0.2));
        prepareToHangSpecimenTask.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchLoose));
        prepareToHangSpecimenTask.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftHangReady, 1));
        prepareToHangSpecimenTask.addStep(intake::isLiftInTolerance);
        prepareToHangSpecimenTask.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchClosed));


        /* == Task: hangSpecimenTask == */
        hangSpecimenTask.autoStart = false;
        hangSpecimenTask.addStep(intake::initSpecLiftPID);
        hangSpecimenTask.addStep(() -> intake.setSlidePosition(intake.getSettings().positionSlideSpecimen, 0.2));
        hangSpecimenTask.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchClosed)); // pinchSuperLoose
        hangSpecimenTask.addStep(() -> intake.setLiftPosition(intake.getSettings().positionLiftHangRelease, 1)); // power: 0.7
        hangSpecimenTask.addStep(intake::isLiftInTolerance);
        hangSpecimenTask.addStep(() -> intake.getHardware().pinch.setPosition(intake.getSettings().pinchFullOpen));
        hangSpecimenTask.addDelay(50);
        hangSpecimenTask.addStep(dockTask::restart);

        /* == Task: lowDumpIntakeTask == */
        lowDumpIntakeTask.autoStart = false;
        lowDumpIntakeTask.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteHumanDeposit));
        lowDumpIntakeTask.addStep(() -> intake.getHardware().chute.isDone());
        lowDumpIntakeTask.addDelay(500);
        lowDumpIntakeTask.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteParked));

        /* == Task: prepareToHangRobotTask == */
        prepareToHangRobotTask.autoStart = false;
        prepareToHangRobotTask.addStep(() -> intake.setHangPosition(intake.getSettings().positionHangReady, 1));
        prepareToHangRobotTask.addStep(() -> intake.getHardware().hang.setPosition(intake.getSettings().hangServoPreUp));
        /* == Task: hangRobotTask == */

        hangRobotTask.autoStart = false;
        hangRobotTask.addStep(() ->intake.getHardware().hang.setPosition(intake.getSettings().hangServoHang));
        hangRobotTask.addStep(() -> intake.setHangPosition(intake.getSettings().positionHangFinal, 1));
        hangRobotTask.addDelay(1000);
        hangRobotTask.addStep(() ->intake.getHardware().hang.disable());

        /* == Task: dockTask == */
        dockTask.autoStart = false;
        dockTask.addStep(() -> {
            //prepareToIntakeTask.runCommand(Group.Command.PAUSE);
            intake.preventUserControl = true;
            intake.getHardware().pinch.setPosition(intake.getSettings().pinchFullOpen);
            intake.getHardware().flipper.setPosition(intake.getSettings().flipperParked);
            intake.getHardware().chute.setPosition(intake.getSettings().chuteParked);
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff);
            intake.setSlidePosition(intake.getSettings().positionSlideOvershoot, 1);
            intake.setLiftPosition(intake.getSettings().positionLiftMin, .667); //1
            intake.getHardware().park.setPosition(intake.getSettings().parkDown);
        });
        dockTask.addStep(() ->
            intake.isSlideInTolerance() &&
            intake.isLiftInTolerance() &&
            intake.getHardware().flipper.isDone() &&
            intake.getHardware().chute.isDone());
        dockTask.addStep(() -> {
            intake.getHardware().flipper.disable();
            intake.getHardware().chute.disable();
            intake.preventUserControl = false;
        });

        /* == Task: transferTask == */
        transferTask.autoStart = false;
        transferTask.addStep(dockTask::restart);
        transferTask.addStep(dockTask::isDone);
        transferTask.addStep(() -> {
            intake.getHardware().flipper.setPosition(intake.getSettings().flipperParked);
            intake.getHardware().chute.setPosition(intake.getSettings().chuteParked);
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerTransfer);
    //        intake.getHardware().spinner.setPosition(intake.testSpinnerOut);
        });
//        transferTask.addDelay(500); // 1500  //todo: Are you SURE this is a good idea?
        transferTask.addTimedStep(() -> {}, () -> intake.readSampleDistance() >= intake.getSettings().distSampleEmpty, 500);
        transferTask.addDelay(50);  // a bit of time to truly finish; test and change as necessary
        transferTask.addStep(() -> {
            intake.getHardware().flipper.disable();
            intake.getHardware().chute.disable();
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff);
        });

        /* == Task: prepareToDepositTask == */
        prepareToDepositTask.autoStart = false;
        prepareToDepositTask.addStep(intake::initSampleLiftPID);
        prepareToDepositTask.addStep(() -> intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff));
        prepareToDepositTask.addStep(() -> intake.getHardware().spinner.isDone());
        prepareToDepositTask.addStep(() -> {
            intake.getHardware().chute.setPosition(intake.getSettings().chuteReady);
            intake.setLiftPosition(intake.getSettings().positionLiftReady, 1);
        });
        prepareToDepositTask.addStep(intake::isLiftInTolerance);
//           AUTO DEPOSIT
        autoDepositTask.autoStart = false;
        autoDepositTask.addStep(intake::initSampleLiftPID);
        autoDepositTask.addStep(() -> {
            intake.getHardware().chute.setPosition(intake.getSettings().chuteReady);
            intake.setLiftPosition(intake.getSettings().positionLiftMax, 1);
        });
//        depositTask.addStep(() -> intake.isLiftInTolerance() && intake.getHardware().chute.isDone());
        autoDepositTask.addStep(() -> intake.getLiftPosition() > intake.getSettings().positionLiftPreDump);
        autoDepositTask.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteSampleDeposit));
        autoDepositTask.addStep(() -> intake.getHardware().chute.isDone());
        autoDepositTask.addDelay(400); // 200
        autoDepositTask.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteParked));
        autoDepositTask.addStep(() -> intake.getHardware().chute.isDone());
        autoDepositTask.addStep(autoHomeTaskLift::restart);

        /* == Task: depositTask == */
        depositTask.autoStart = false;
        depositTask.addStep(intake::initSampleLiftPID);
        depositTask.addStep(() -> {
            intake.getHardware().chute.setPosition(intake.getSettings().chuteReady);
            intake.setLiftPosition(intake.getSettings().positionLiftMax, 1);
        });
//        depositTask.addStep(() -> intake.isLiftInTolerance() && intake.getHardware().chute.isDone());
        depositTask.addStep(() -> intake.getLiftPosition() > intake.getSettings().positionLiftPreDump);
        depositTask.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteSampleDeposit));
        depositTask.addStep(() -> intake.getHardware().chute.isDone());
        depositTask.addDelay(400); // 200
        depositTask.addStep(() -> intake.getHardware().chute.setPosition(intake.getSettings().chuteParked));
        depositTask.addStep(() -> intake.getHardware().chute.isDone());
        depositTask.addStep(dockTask::restart);

        /* == Task: autoIntakeTask == */
        autoIntakeTask.autoStart = false;
        autoIntakeTask.addStep(() -> {
            intake.getHardware().flipper.setPosition((intake.getSettings().flipperAlmostFloor));
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerIn);
        });
        //lk added this; subtract if trouble
        autoIntakeTask.addStep(() -> intake.getHardware().flipper.isDone());
        autoIntakeTask.addStep(() -> intake.getHardware().flipper.disable());
        //end lk
        autoIntakeTask.addStep(() -> (intake.readSampleDistance() <= intake.getSettings().distSampleGood));
        autoIntakeTask.addStep(() -> (intake.identifySampleColor() > 0));
        autoIntakeTask.addStep(() -> {
            if (intake.isSampleGood(intake.lastSample)) {
                prepareToTransferTask.restart();
            } else {
                ejectBadSampleTask.restart();
            }
        });

        /* == Task: ejectBadSampleTask == */
        ejectBadSampleTask.autoStart = false;
        ejectBadSampleTask.addStep(() -> intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOut));
        ejectBadSampleTask.addDelay(1500);
        ejectBadSampleTask.addStep(autoIntakeTask::restart);

        /* == Task: prepareToTransferTask == */
        prepareToTransferTask.autoStart = false;
        prepareToTransferTask.addStep(() -> {
            intake.getHardware().flipper.setPosition(intake.getSettings().flipperSafe);
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerSlowOut);
        });
        //prepareToTransferTask.addDelay(350);
        prepareToTransferTask.addTimedStep(() -> {}, () -> intake.readSampleDistance() >= intake.getSettings().distSampleUnload, 2000); //todo: try a longer timeout
        prepareToTransferTask.addStep(() -> {
            intake.getHardware().spinner.setPosition(intake.getSettings().spinnerOff);
            transferTask.restart();
        });

        /* == Task: autoHomeTask == */
        autoHomeTask.autoStart = false;
        autoHomeTask.addStep(autoHomeTaskLift::restart);
        autoHomeTask.addStep(autoHomeTaskSlide::restart);
        autoHomeTask.addStep(autoHomeTaskSlide::isDone);
        autoHomeTask.addStep(autoHomeTaskLift::isDone);

        //homes lift
        autoHomeTaskLift.autoStart = false;
        autoHomeTaskLift.addStep(() -> this.setSlideToHomeConfig(1));
        autoHomeTaskLift.addTimedStep(
                () -> robot.opMode.telemetry.addData("homing", intake.getHardware().liftZeroSwitch.getState()),
                () -> intake.getHardware().liftZeroSwitch.getState(), 10000);
        autoHomeTaskLift.addStep(() -> {
            intake.getHardware().liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake.zeroLiftPos();
            intake.getHardware().liftMotor.setTargetPosition(intake.getSettings().positionLiftHome);
            intake.liftTargetPosition = intake.getSettings().positionLiftHome;
            setMotorsToRunConfig(1);
        });

        //homes slides
        autoHomeTaskSlide.autoStart = false;
        autoHomeTaskSlide.addStep(() -> this.setSlideToHomeConfig(2));
        autoHomeTaskSlide.addTimedStep(
                () -> robot.opMode.telemetry.addData("homingH", intake.getHardware().slideZeroSwitch.getState()),
                () -> intake.getHardware().slideZeroSwitch.getState(), 10000);
        autoHomeTaskSlide.addStep(() -> {
            intake.getHardware().slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake.getHardware().slideMotor.setTargetPosition(intake.getSettings().positionSlideHome);
            intake.slideTargetPosition = intake.getSettings().positionSlideHome;
            setMotorsToRunConfig(2);
        });

        /* ========== end of constructAllIntakeTasks() ========== */
    }

    private void setSlideToHomeConfig(int i) {
        double power = -.33; //-0.125;
        if (i == 1) {
            intake.getHardware().liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intake.getHardware().liftMotor.setPower(power);
        } else if (i == 2) {
            intake.getHardware().slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intake.getHardware().slideMotor.setPower(power);
        }
    }

    private void setMotorsToRunConfig(int i) {
        if (i == 1) {
            intake.getHardware().liftMotor.setPower(IntakeHardware.slideHoldPower);
            intake.getHardware().liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        } else if (i == 2) {
            intake.getHardware().slideMotor.setPower(IntakeHardware.slideHoldPower);
            intake.getHardware().slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }

    /***********************************************************************************/
    public static final class TaskNames {
        public final static String autonomousSample = "autonomous sample";
        public final static String specAutoIntakePickup = "specimen Auto Intake Pickup";
        public final static String specAutoIntakeDeposit = "specimen Auto Intake Deposit";
        public final static String moveToPickupSpecimen = "move to Pickup Specimen Position";
        public final static String moveToHangSpecimen = "move to Hang Specimen Position";
        public final static String autoHome = "auto home";
        public final static String prepareToIntake = "prepare to intake";
        public final static String safe = "safe";
        public final static String transfer = "transfer";
        public final static String lowDumpIntake = "low dump intake for specimen";
        public final static String prepareToGetSpecimen = "prepare to get specimen";
        public final static String getSpecimen = "get specimen";
        public final static String hangSpecimen = "hang specimen";
        public final static String prepareToHangSpecimen = "prepare to hang specimen";
        public final static String prepareToDeposit = "prepare to deposit";
        public final static String deposit = "deposit";
        public final static String autoIntake = "auto intake";
        public final static String checkSample = "check sample";
        public final static String ejectBadSample = "eject the sample";
        public final static String prepareToTransfer = "prepare the transfer";
        public final static String prepareToHangRobotTask = "prepare to hang robot";
        public final static String hangRobotTask = "hang robot";
    }

    public static final class Events {
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}
