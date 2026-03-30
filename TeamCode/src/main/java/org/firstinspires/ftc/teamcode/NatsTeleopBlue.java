package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.mechanisms.TextFile;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class NatsTeleopBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer;

    //Define variables here
    double crawlMode;







    //Add cases here
    public enum PathState{

        //Drivetrain Cases
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
        CRAWL_MODE

    }

    //Define PathStates here
    PathState drivetrainPathState;
    PathState crawlModePathState;





    //Define Positions here
    private final Pose blueFinish = new Pose(38,35,Math.toRadians(90));
    private final Pose closeBlueLaunchPose = new Pose(49,98,Math.toRadians(135));
    private final Pose farBlueLaunchPose = new Pose(57,9,Math.toRadians(110));
    private final Pose blueGatePos = new Pose(10,59,Math.toRadians(150));


    //Define PathChains here
    private PathChain driveToRedFinish;
    private PathChain driveToCloseLaunchPose;
    private PathChain driveToFarLaunchPose;
    private PathChain driveToBlueGatePos;



    public void buildPaths(){

        //Build PathChains here

        driveToRedFinish = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),blueFinish))
                .setLinearHeadingInterpolation(follower.getHeading(),blueFinish.getHeading())
                .build();
        driveToCloseLaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),closeBlueLaunchPose))
                .setLinearHeadingInterpolation(follower.getHeading(),closeBlueLaunchPose.getHeading())
                .build();
        driveToFarLaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),farBlueLaunchPose))
                .setLinearHeadingInterpolation(follower.getHeading(),farBlueLaunchPose.getHeading())
                .build();
        driveToBlueGatePos = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),blueGatePos))
                .setLinearHeadingInterpolation(follower.getHeading(),blueGatePos.getHeading())
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
                follower.followPath(driveToRedFinish,true);
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
                drivetrainPathState = PathState.STANDBY;
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















    //Set path states here
    public void setDrivetrainPathState(PathState newState){
        drivetrainPathState = newState;
        pathTimer.resetTimer();
    }public void setCrawlModePathState(PathState newState){
        crawlModePathState = newState;
    }










    public void init(){
        drivetrainPathState = PathState.FIELD_ORIENTATED_MANUAL_DRIVE_START;
        crawlModePathState = PathState.NORMAL_MODE;
        follower = Constants.createFollower(hardwareMap);

        pathTimer = new Timer();
        double[] data = TextFile.loadAll();

        if (data != null && data.length >= 8) {
//            pos1            = (int) data[0];
//            pos3            = (int) data[1];
//            pos5            = (int) data[2];

//            currentGreenPos = (int) data[6];
//            motifGreenPos   = (int) data[7];

            follower.setPose(new Pose(data[3], data[4], data[5]));
            buildPaths();
        }

    }









    public void start(){
        setDrivetrainPathState(drivetrainPathState);
        setCrawlModePathState(crawlModePathState);

    }









    public void loop(){
        telemetry.addData("Pose",follower.getPose());
        telemetry.addData("Drivetrain PathState:", drivetrainPathState);
        telemetry.addLine("RACING STRIPES");
        follower.update();
        statePathUpdate();
        setDriveMode();

    }

}
