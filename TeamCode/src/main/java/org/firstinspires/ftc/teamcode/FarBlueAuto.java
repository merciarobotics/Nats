package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.TextFile;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class FarBlueAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, uptakeTimer;
    private IMU imu;
    DetectedColor detectedColor;
    NormalizedColorSensor colorSensor;
    private float gain = 9;
    boolean tagDetected = false;
    int rpm = 1150;





    public DcMotorEx intakeMotor;
    public DcMotorEx flywheelMotor1;
    public DcMotorEx flywheelMotor2;






    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public enum PathState{
        START_TO_SHOOT,
        SHOOT_ZERO,
        SHOOT_TO_FIRST_INTAKE,
        FIRST_INTAKE,
        FIRST_INTAKE_TO_SHOOT,
        SHOOT_ONE,
        SHOOT_TO_SECOND_INTAKE,
        SECOND_INTAKE,
        SECOND_INTAKE_TO_SHOOT,
        SHOOT_TWO,
        INTAKE_FORWARD,
        INTAKE_STATIONARY,
        INTAKE_REVERSE,
        INTAKE,
        UPTAKE,
        STANDBY,
        GATE_SHOOT,
        GATE_CLOSED,
        FLYWHEEL_AUTO,
        FLYWHEEL_OFF,
        FLYWHEEL_MANUAL

    }

    PathState autoPathState;
    PathState intakePathState;

    PathState gatePathstate;
    PathState flywheelPathstate;

    private final Pose startPose = new Pose(57,9,Math.toRadians(90));
    private final Pose farBlueLaunchPose = new Pose(57,9,Math.toRadians(110));

    private final Pose firstIntakePose1 = new Pose(48.30985915492958,36,Math.toRadians(180));
    private final Pose firstIntakePose2 = new Pose(12.5774647887324,36,Math.toRadians(180));       //TODO change these for the other autos
    private final Pose secondIntakePose1 = new Pose(48.30985915492958,60,Math.toRadians(180));
    private final Pose secondIntakePose2 = new Pose(12.5774647887324,60,Math.toRadians(180));
    TextFile text = new TextFile();
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private PathChain startPoseToFarLaunchPose;
    private PathChain farLaunchPoseToFirstIntake;
    private PathChain firstIntake;
    private PathChain firstIntakeToFarLaunchPose;
    private PathChain farLaunchPoseToSecondIntake;
    private PathChain secondIntake;
    private PathChain secondIntakeToFarLaunchPose;












    public void setFlywheelMotorPower(double curTargetVelocity){
        flywheelMotor1.setVelocity(curTargetVelocity);
        flywheelMotor2.setVelocity(curTargetVelocity);

//        curVelocity = flywheelMotor1.getVelocity();
    }










    public void buildPaths(){

        //Build PathChains here

        startPoseToFarLaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose.getPose(),farBlueLaunchPose.getPose()))
                .setLinearHeadingInterpolation(startPose.getHeading(),farBlueLaunchPose.getHeading())
                .build();

        farLaunchPoseToFirstIntake = follower.pathBuilder()
                .addPath(new BezierLine(farBlueLaunchPose.getPose(),firstIntakePose1.getPose()))
                .setLinearHeadingInterpolation(farBlueLaunchPose.getHeading(),firstIntakePose1.getHeading())
                .build();

        firstIntake = follower.pathBuilder()
                .addPath(new BezierLine(firstIntakePose1.getPose(),firstIntakePose2.getPose()))
                .setLinearHeadingInterpolation(firstIntakePose1.getHeading(),firstIntakePose2.getHeading())
                .build();

        firstIntakeToFarLaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(firstIntakePose2.getPose(),farBlueLaunchPose.getPose()))
                .setLinearHeadingInterpolation(firstIntakePose2.getHeading(),farBlueLaunchPose.getHeading())
                .build();

        farLaunchPoseToSecondIntake = follower.pathBuilder()
                .addPath(new BezierLine(farBlueLaunchPose.getPose(),secondIntakePose1.getPose()))
                .setLinearHeadingInterpolation(farBlueLaunchPose.getHeading(),secondIntakePose1.getHeading())
                .build();

        secondIntake = follower.pathBuilder()
                .addPath(new BezierLine(secondIntakePose1.getPose(),secondIntakePose2.getPose()))
                .setLinearHeadingInterpolation(secondIntakePose1.getHeading(),secondIntakePose2.getHeading())      //TODO change these for the other autos
                .build();

        secondIntakeToFarLaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(secondIntakePose2.getPose(),farBlueLaunchPose.getPose()))
                .setLinearHeadingInterpolation(secondIntakePose2.getHeading(),farBlueLaunchPose.getHeading())
                .build();

    }


    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //returns 4 values R, G, B, A

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        if(0.07 < normRed && normRed < 0.095 && 0.0975 < normGreen && normGreen < 0.12 && 0.12 < normBlue && normBlue < 0.15 ){
            return DetectedColor.PURPLE;
        }
        else if(0.045 < normRed && normRed < 0.065 && 0.145 < normGreen && normGreen < 0.18 && 0.11 < normBlue && normBlue < 0.13 ){
            return DetectedColor.GREEN;
        }else {
            return DetectedColor.UNKNOWN;
        }


    }
    public void statePathUpdate(){
        switch (autoPathState){
            case START_TO_SHOOT:
                follower.followPath(startPoseToFarLaunchPose,true);
                setAutoPathState(PathState.SHOOT_ZERO);
                break;

            case SHOOT_ZERO:
                if (!follower.isBusy()){
                    setAutoPathState(PathState.SHOOT_TO_FIRST_INTAKE);
                }
                break;

            case SHOOT_TO_FIRST_INTAKE:
                //Add shooting code here
                if (pathTimer.getElapsedTimeSeconds() > 3.0){
                    follower.followPath(farLaunchPoseToFirstIntake,true);           //TODO change these for the other autos
                    setAutoPathState(PathState.FIRST_INTAKE);
                }
                break;

            case FIRST_INTAKE:
                if (!follower.isBusy()){
                    follower.followPath(firstIntake,true);
                    setAutoPathState(PathState.FIRST_INTAKE_TO_SHOOT);
                }

                break;

            case FIRST_INTAKE_TO_SHOOT:
                //Add intake code here
                if (!follower.isBusy()){
                    follower.followPath(firstIntakeToFarLaunchPose,true);
                    setAutoPathState(PathState.FIRST_INTAKE_TO_SHOOT);
                }

                break;

            case SHOOT_ONE:
                if (!follower.isBusy()){
                    setAutoPathState(PathState.SHOOT_TO_SECOND_INTAKE);
                }
                break;

            case SHOOT_TO_SECOND_INTAKE:
                //Add shooting code here
                if (pathTimer.getElapsedTimeSeconds() > 3.0){
                    follower.followPath(farLaunchPoseToSecondIntake,true);
                    setAutoPathState(PathState.SECOND_INTAKE);
                }
                break;

            case SECOND_INTAKE:
                if (!follower.isBusy()){
                    follower.followPath(secondIntake,true);
                    setAutoPathState(PathState.SECOND_INTAKE_TO_SHOOT);
                }
                break;

            case SECOND_INTAKE_TO_SHOOT:
                if (!follower.isBusy()){
                    follower.followPath(secondIntakeToFarLaunchPose,true);
                    setAutoPathState(PathState.SHOOT_TWO);
                }
                break;

            case SHOOT_TWO:
                if (!follower.isBusy()){
                    //Add shooting code here
                }
                break;

        }







        switch (gatePathstate){
            case GATE_SHOOT:

//                setGateServoPosition();

            case GATE_CLOSED:                //TODO

//                setGateServoPosition();
        }










        switch (flywheelPathstate){


            case FLYWHEEL_MANUAL:
                setFlywheelMotorPower(rpm);

                break;

            case FLYWHEEL_OFF:
                setFlywheelMotorPower(0.0);
                break;


            case FLYWHEEL_AUTO:
//                autoVelocity();  // TODO
                break;

            default:
                setFlywheelPathState(PathState.FLYWHEEL_OFF);
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











    }





    public void setAutoPathState(PathState newState){
        autoPathState = newState;
        pathTimer.resetTimer();
    }public void setIntakePathState(PathState newState) {
        intakePathState = newState;
    }public void setFlywheelPathState(PathState newState) {
        flywheelPathstate = newState;
    }public void setGatePathstate(PathState newState) {
        gatePathstate = newState;
    }







    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        uptakeTimer = new Timer();
        autoPathState = PathState.START_TO_SHOOT;


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




        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(RevOrientation));


        aprilTagWebcam.init(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        uptakeTimer.resetTimer();
        setAutoPathState(autoPathState);

    }

    @Override
    public void loop() {
        statePathUpdate();
        follower.update();
        detectedColor = getDetectedColor(telemetry);
        telemetry.addData("Follower Busy:",follower.isBusy());
        telemetry.addData("Auto PathState",autoPathState);
        telemetry.addData("Color Detected",detectedColor);

        text.saveAll( follower.getPose().getX(),  follower.getPose().getY(),  follower.getPose().getHeading());

    }
}




