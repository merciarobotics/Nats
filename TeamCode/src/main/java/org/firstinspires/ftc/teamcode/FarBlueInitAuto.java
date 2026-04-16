package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
public class FarBlueInitAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer, uptakeTimer;
    private IMU imu;
    DetectedColor detectedColor;
    NormalizedColorSensor colorSensor;
    private float gain = 9;
    boolean tagDetected = false;



    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN
    }
    private final Pose startPose = new Pose(57,9,Math.toRadians(90));
    TextFile text = new TextFile();
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();


    public void buildPaths(){


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






    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        uptakeTimer = new Timer();








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

    }

    @Override
    public void loop() {
        statePathUpdate();
        follower.update();

        text.saveAll(0,0,0,
        follower.getPose().getX(),  follower.getPose().getY(),  follower.getPose().getHeading(), 1,0);

    }
}




