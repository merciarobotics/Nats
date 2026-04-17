package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.TextFile;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class NatsTeleopBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private IMU imu;

    //Define variables here
    double crawlMode;
    double pitch;

    //Define servos and motors here

    private Servo turretPitchServo;
    public DcMotorEx intakeMotor;
    public DcMotorEx flywheelMotor1;
    public DcMotorEx flywheelMotor2;
    private Servo gateServo;








    //Add cases here
    public enum PathState{


        STANDBY,
        FIELD_ORIENTATED_MANUAL_DRIVE_START,
        FIELD_ORIENTATED_MANUAL_DRIVE,
        ROBOT_ORIENTATED_MANUAL_DRIVE_START,
        ROBOT_ORIENTATED_MANUAL_DRIVE,
        DRIVE_TO_BLUE_FINISH,
        FAR_SHOOT_POS,
        NEAR_SHOOT_POS,
        GATE_POS,
        NORMAL_MODE,
        CRAWL_MODE,
        INTAKE_FORWARD,
        INTAKE_STATIONARY,
        INTAKE_REVERSE,
        INTAKE,
        UPTAKE,
        TURRET_PITCH_AUTO,
        TURRET_PITCH_MANUAL,
        FLYWHEEL_MANUAL,
        FLYWHEEL_OFF,
        FLYWHEEL_AUTO

    }

    //Define PathStates here
    PathState drivetrainPathState;
    PathState crawlModePathState;
    PathState flywheelPathstate;
    PathState intakePathState;
    PathState turretPitchState;





    //Define Positions here
    private final Pose blueFinish = new Pose(38,35,Math.toRadians(90));
    private final Pose closeBlueLaunchPose = new Pose(49,98,Math.toRadians(135));
    private final Pose farBlueLaunchPose = new Pose(57,9,Math.toRadians(110));
    private final Pose blueGatePos = new Pose(10,59,Math.toRadians(150));
//


    //Define PathChains here
    private PathChain driveToBlueFinish;
    private PathChain driveToCloseLaunchPose;
    private PathChain driveToFarLaunchPose;
    private PathChain driveToBlueGatePos;
    private PathChain lockOnGoal;










    public double getTargetHeading(){
        double changeY = 138 - follower.getPose().getY();      // TODO for red
        double changeX = follower.getPose().getX();

        return Math.PI + Math.atan(changeY/changeX);
    }


    public void setUpTakeServoDirection1(double direction){
        gateServo.setPosition(direction);
    }


    public void setFlywheelMotorPower(double curTargetVelocity){
        flywheelMotor1.setVelocity(curTargetVelocity);
        flywheelMotor2.setVelocity(curTargetVelocity);

        curVelocity = flywheelMotor1.getVelocity();
    }






    public void buildPaths(){

        //Build PathChains here

        driveToBlueFinish = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),blueFinish.getPose()))
                .setLinearHeadingInterpolation(follower.getHeading(),blueFinish.getHeading())
                .build();
        driveToCloseLaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),closeBlueLaunchPose.getPose()))
                .setLinearHeadingInterpolation(follower.getHeading(),closeBlueLaunchPose.getHeading())
                .build();
        driveToFarLaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),farBlueLaunchPose.getPose()))
                .setLinearHeadingInterpolation(follower.getHeading(),farBlueLaunchPose.getHeading())
                .build();
        driveToBlueGatePos = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),blueGatePos.getPose()))
                .setLinearHeadingInterpolation(follower.getHeading(),blueGatePos.getHeading())
                .build();
        lockOnGoal = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),follower.getPose()))
                .setLinearHeadingInterpolation(follower.getHeading(),getTargetHeading())
                .build();
    }

    public void statePathUpdate(){



        //Add switches here

        switch(drivetrainPathState){

            case FIELD_ORIENTATED_MANUAL_DRIVE_START:
                follower.startTeleopDrive();
                setDrivetrainPathState(PathState.FIELD_ORIENTATED_MANUAL_DRIVE);
                break;

            case FIELD_ORIENTATED_MANUAL_DRIVE:
                follower.setTeleOpDrive(gamepad1.left_stick_y*crawlMode, gamepad1.left_stick_x*crawlMode, -gamepad1.right_stick_x*crawlMode,false);
                break;

            case ROBOT_ORIENTATED_MANUAL_DRIVE_START:
                follower.startTeleopDrive();
                setDrivetrainPathState(PathState.ROBOT_ORIENTATED_MANUAL_DRIVE);
                break;

            case ROBOT_ORIENTATED_MANUAL_DRIVE:
                follower.setTeleOpDrive(-gamepad1.left_stick_y*crawlMode, -gamepad1.left_stick_x*crawlMode, -gamepad1.right_stick_x*crawlMode,true);

                break;

            case DRIVE_TO_BLUE_FINISH:
                follower.followPath(driveToBlueFinish,true);
                setDrivetrainPathState(PathState.STANDBY);
                break;
            case FAR_SHOOT_POS:
                follower.followPath(driveToFarLaunchPose,true);
                setDrivetrainPathState(PathState.STANDBY);
                break;
            case NEAR_SHOOT_POS:
                follower.followPath(driveToCloseLaunchPose,true);
                setDrivetrainPathState(PathState.STANDBY);
                break;
            case GATE_POS:
                follower.followPath(driveToBlueGatePos,true);
                setDrivetrainPathState(PathState.STANDBY);
                break;

            case STANDBY:
                break;

            default:
                setDrivetrainPathState(PathState.STANDBY);
                break;


        }











        switch (crawlModePathState){
            case NORMAL_MODE:
                crawlMode = 1;
                setCrawlModePathState(PathState.STANDBY);
                break;

            case CRAWL_MODE:
                crawlMode = 0.25;
                setCrawlModePathState(PathState.STANDBY);
                break;

            case STANDBY:
                if(gamepad1.leftStickButtonWasPressed() && crawlMode == 0.25){
                    setCrawlModePathState(PathState.NORMAL_MODE);
                }if(gamepad1.leftStickButtonWasPressed() && crawlMode == 1){
                setCrawlModePathState(PathState.CRAWL_MODE);
            }

                break;

            default:
                setCrawlModePathState(PathState.NORMAL_MODE);
                break;


        }












        switch (intakePathState){
            case INTAKE_FORWARD:
                intakeMotor.setVelocity(900);
                break;

            case INTAKE_STATIONARY:
                intakeMotor.setVelocity(0);
                break;

            case INTAKE_REVERSE:
                intakeMotor.setVelocity(-900);
                break;

            default:
                setIntakePathState(PathState.INTAKE_STATIONARY);
                break;

        }




        switch (flywheelPathstate){





            case FLYWHEEL_MANUAL:
                setFlywheelMotorPower(rpm);

                break;

            case FLYWHEEL_OFF:
                setFlywheelMotorPower(0.0);
                break;


            case FLYWHEEL_AUTO:
                autoVelocity();
                break;

            default:
                setFlywheelPathState(FLYWHEEL_OFF);
                break;




        }












        switch (turretPitchState){
            case TURRET_PITCH_AUTO:
                autoPitch();
                telemetry.addLine("Turret Pitch PathState: Auto");
                break;

            case TURRET_PITCH_MANUAL:
                setTurretPitchServoPosition(Math.abs(pitch));
                telemetry.addData("Manual Pitch:", turretPitchServo.getPosition());
                break;
        }
    }







    //Add custom methods here
    public void setDriveMode() {
        if (gamepad1.right_trigger > 0.9) {
            setDrivetrainPathState(PathState.FIELD_ORIENTATED_MANUAL_DRIVE_START);
        }
        if (gamepad2.rightStickButtonWasPressed()) {
            setDrivetrainPathState(PathState.ROBOT_ORIENTATED_MANUAL_DRIVE_START);
        }
        if (gamepad1.rightBumperWasPressed()) {
            setDrivetrainPathState(PathState.FAR_SHOOT_POS);
        }
        if (gamepad1.leftBumperWasPressed()) {
            setDrivetrainPathState(PathState.NEAR_SHOOT_POS);
        }
        if (gamepad1.left_trigger > 0.9) {
            setDrivetrainPathState(PathState.DRIVE_TO_BLUE_FINISH);
        }
        if (gamepad1.rightStickButtonWasPressed()) {
            setDrivetrainPathState(PathState.GATE_POS);
        }

    }
    public void setIntakeMode(){
        if (gamepad2.dpadUpWasReleased()){
            setIntakePathState(PathState.INTAKE_FORWARD);
        }if (gamepad2.dpadLeftWasReleased()){
            setIntakePathState(PathState.INTAKE_STATIONARY);
        }if (gamepad2.dpadDownWasReleased()){
            setIntakePathState(PathState.INTAKE_REVERSE);
        }
    }

//    public void setIntakeServoPower(double power){
//        intakeServo.setPower(power);
//        intakeServo2.setPower(power);
//        intakeServo3.setPower(power);
//
//    }
    public void autoPitch(){
        setTurretPitchServoPosition(0.00000159852*Math.pow(getDistance(),3)-0.000458069*Math.pow(getDistance(),2)+0.0318736*getDistance()+0.122825);
    }public void setTurretPitchServoPosition(double position){
        turretPitchServo.setPosition(position);
    }public double getDistance(){
        return follower.getPose().distanceFrom(new Pose(6,144)) + 3.5;
    }















    //Set path states here
    public void setDrivetrainPathState(PathState newState){
        drivetrainPathState = newState;
        pathTimer.resetTimer();
    }public void setCrawlModePathState(PathState newState){
        crawlModePathState = newState;
    }public void setFlywheelPathstate(PathState newState){
        flywheelPathstate = newState;
    }public void setIntakePathState(PathState newState){
        intakePathState = newState;
    }public void setTurretPitchPathState(PathState newState){
        turretPitchState = newState;
    }








    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(farBlueLaunchPose);
        follower.update();
        buildPaths();
        drivetrainPathState = PathState.FIELD_ORIENTATED_MANUAL_DRIVE_START;
        crawlModePathState = PathState.NORMAL_MODE;
        intakePathState = PathState.INTAKE_STATIONARY;
        turretPitchState = PathState.TURRET_PITCH_AUTO;


        //Add init mechanisms here

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(RevOrientation));

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        turretPitchServo = hardwareMap.get(Servo.class, "turretPitchServo");

        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");
        flywheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(32,0,0,0.92);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");


        pathTimer = new Timer();
        double[] data = TextFile.loadAll();

//        if (data != null && data.length >= 8) {
////            pos1            = (int) data[0];
////            pos3            = (int) data[1];
////            pos5            = (int) data[2];
//
////            currentGreenPos = (int) data[6];
////            motifGreenPos   = (int) data[7];
//
//            follower.setPose(new Pose(data[3], data[4], data[5]));
//
//        }else {
//            follower.setPose(new Pose(0,0,0));
//        }


    }









    public void start(){
        setDrivetrainPathState(drivetrainPathState);
        setCrawlModePathState(crawlModePathState);
        setIntakePathState(intakePathState);
        setTurretPitchPathState(turretPitchState);

    }









    public void loop(){
        telemetry.addData("Pose",follower.getPose());
        telemetry.addData("Drivetrain PathState:", drivetrainPathState);
        telemetry.addLine("RACING STRIPES");
        follower.update();
        statePathUpdate();
        setIntakeMode();
        setDriveMode();

    }

}
