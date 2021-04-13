package org.firstinspires.ftc.teamcode;


//todo: position manager doesn't work, heading is having issues again,
// 3 posible solutions:
// 1: conversion error
public class PositionAndTargetManager {
    /*info found: https://firstinspiresst01.blob.core.windows.net/first-game-changers/ftc/field-setup-guide.pdf starting page 8
     *        and: https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-2-remote-events.pdf starting page 26
     * Coords format: x,y,z
     * Origin (0,0) : center of the 6x6(square) grid
     *      field x bounds [-1.79705m,+1.79705m]
     *      field y bounds [-1.79705m,+1.79705m]
     * z = height from the foam grid
     * x+ = toward red alliance station (right from audience area perspective)
     * x- = toward blue alliance station
     * y+ = toward the tower goal and power shot targets
     * y- = toward audience
     *
     * position of red targets (center of volume) (x,y,z)
     *  - power shot 1:     +0.09525m,      +1.79705m,    +0.784225m
     *  - power shot 2:     +0.28575m,      +1.79705m,    +0.784225m
     *  - power shot 3:     +0.47625m,      +1.79705m,    +0.784225m
     *  - high goal:        +0.8905875m,    +1.79705m,    +0.911225m
     *  - medium goal:      -0.8905875m,    +1.79705m,    +0.6873875m
     *  - low goal:         +0.8905875m,    +1.79705m,    +0.4318m
     *
     * position of blue targets (center of volume) (x,y,z)
     *  - power shot 1:     -0.09525m,      +1.79705m,    +0.784225m
     *  - power shot 2:     -0.28575m,      +1.79705m,    +0.784225m
     *  - power shot 3:     -0.47625m,      +1.79705m,    +0.784225m
     *  - high goal:        -0.8905875m,    +1.79705m,    +0.911225m
     *  - medium goal:      +0.8905875m,    +1.79705m,    +0.6873875m
     *  - low goal:         -0.8905875m,    +1.79705m,    +0.4318m
     *
     * Red starting pos:    +___, -___, 0m
     * blue starting pos:   -___, -___, 0m
     */
    /*
        [r]
        r = 0, power shot 1
        r = 1, power shot 2
        r = 2, power shot 3
        r = 3, high goal
        r = 4, medium goal
        r = 5, low goal
        [c]
        c = 0, x
        c = 1, y
        c = 2, z
     */
    //initializes assuming it's on team red, in the constructor, values will be changed as needed if it's on team blue
    double[][] targets = {
            /*x,y,z*/
            {0.09525, 1.79705, 0.784225},     /*power shot 1*/
            {0.28575, 1.79705, 0.784225},     /*power shot 2*/
            {0.47625, 1.79705, 0.784225},     /*power shot 3*/
            {0.8905875, 1.79705, 0.911225},   /*high goal*/
            {0.8905875, 1.79705, 0.6873875}, /*medium goal*/ //flip x value for in person events
            {0.8905875, 1.79705, 0.4318}      /*low goal*/
    };

    /*
        [r]
        r = 0, x
        r = 1, y
        r = 2, z
     */
    //initializes assuming it's on team red, in the constructor, values will be changed as needed if it's on team blue
    double[] robotPosition = new double[2]; //TODO: update these coords when they are known to a more accurate degree
    double robotHeading = Math.PI / 2; //heading relative to field, pi/2 = toward goals

    double[] targetPosition = new double[3];
    int currentTarget;

    double launchZone = 2.06375 - 1.79705/*+- 0.0254m*/;       //any position with a y coordinate less (maybe more than?) than launchZone is in the launch zone

    double metersPerCount;
    double robotOdoWidth;
    double horizRadiansPerCount;
    double robotOdoLength;

    byte powerShotsHit = 0;

    int previousLeftCounts = 0;
    int previousRightCounts = 0;
    int previousHorizCounts = 0;

    short leftDirection;
    short rightDirection;
    short horizDirection;
    ////////////////////////////// constructors //////////////////////////////

    /**
     * constructor for the PositionAndTargetManager class
     * use this constructor if no autonomous was run
     * this constructor assumes that the robot is in the corner, facing toward the wall with the goal targets
     *
     * @param robot     passed to allow for some calculations, and access to certain robot dimensions
     * @param isTeamRed true if on team red, false otherwise, used to personalize the target array, and initial position for what team the robot is on
     */
    public PositionAndTargetManager(HardwareUltimateGoal robot, boolean isTeamRed) {
        //take some variables from the robot
        metersPerCount = robot.ODOMETRY_METERS_PER_COUNT;
        robotOdoWidth = robot.getRobotOdometryWidth();
        robotOdoLength = robot.getRobotOdometryLength();
        horizRadiansPerCount = metersPerCount / robotOdoLength;
        leftDirection = robot.getLeftDirection();
        rightDirection = robot.getRightDirection();
        horizDirection = robot.getHorizDirection();

        robotPosition = new double[]{1.79705 - 0.8, -1.79705 + 0.225}; //0.57785 is the distance from the right wall, 0.4572 is the length of the robot, //remeasured, but old measurements are what's recorded in comments
        //flip some things for if the robot is on blue team
        if (!isTeamRed) {
            for (int r = 0; r < targets.length; r++) {
                targets[r][0] *= -1.0; //flip the x-axis if not on the red team
            }

            robotPosition[0] *= -1.0; //flip the x-axis if not on the red team
        }
    }

    /**
     * constructor for the PositionAndTargetManager class
     * use this constructor if an autonomous was run
     *
     * @param robot        passed to allow for some calculations, and access to certain robot dimensions
     * @param initPosition this array should be given by the readPosition() method of HardwareUltimateGoal
     * @param initHeading  this should be given by the readHeading() method of HardwareUltimateGoal
     * @param isTeamRed    true if on team red, false otherwise, used to personalize the target array for what team the robot is on
     */
    public PositionAndTargetManager(HardwareUltimateGoal robot, double[] initPosition, double initHeading, boolean isTeamRed) {
        //take some variables from the robot
        metersPerCount = robot.ODOMETRY_METERS_PER_COUNT;
        robotOdoWidth = robot.getRobotOdometryWidth();
        robotOdoLength = robot.getRobotOdometryLength();
        horizRadiansPerCount = metersPerCount / robotOdoLength;
        leftDirection = robot.getLeftDirection();
        rightDirection = robot.getRightDirection();
        horizDirection = robot.getHorizDirection();

        //flip some things for if the robot is on blue team
        if (!isTeamRed) {
            for (int r = 0; r < targets.length; r++) {
                targets[r][0] *= -1.0; //flip the x-axis if not on the red team
            }
        }
        //set position and heading to given values
        robotPosition[0] = initPosition[0];
        robotPosition[1] = initPosition[1];
        robotHeading = initHeading;

    }

    ////////////////////////////// update and calculate method //////////////////////////////
    public void update(int leftCounts, int rightCounts, int horizCounts) {
        double headingChange = 0.0;

        int leftChange = leftCounts - previousLeftCounts;
        int rightChange = rightCounts - previousRightCounts;
        int horizChange = horizCounts - previousHorizCounts;

        previousLeftCounts = leftCounts;
        previousRightCounts = rightCounts;
        previousHorizCounts = horizCounts;

        leftChange *= leftDirection;
        rightChange *= rightDirection;
        horizChange *= horizDirection;

        //positions
        double s1 = leftChange * metersPerCount;  // distance the left wheel traveled (m) (delta s1)
        double s2 = rightChange * metersPerCount; // distance the right wheel traveled (m) (delta s2)
        double s = (s1 + s2) / 2.0; // average distance travelled between the two wheels; also the length of the arc the robot moved

        //Calculate Angle change, robot width may need to be adjusted, and must be accurate to a high degree
        double n = s2 - s1;
        headingChange = (s2 - s1) / robotOdoWidth;
        robotHeading += headingChange;

        horizChange -= headingChange/horizRadiansPerCount;
        //some house keeping


        //calculate the changes in position

        robotPosition[0] += (s*Math.sin(robotHeading) + n*Math.cos(robotHeading));// * metersPerCount;
        robotPosition[1] += (s*Math.cos(robotHeading) - n*Math.sin(robotHeading));// * metersPerCount;


        //make sure robotHeading is in the range [pi,-pi] not [2pi, 0]
        double twoPI = 2 * Math.PI;
        if (robotHeading > Math.PI) {
            robotHeading -= twoPI;
        } else if (robotHeading < -Math.PI) {
            robotHeading += twoPI;
        }

        /*using Wizards ideas would involve,
        replacing line 149 (robotPosition[0] += s * Math.cos(robotHeading);) with robotPosition[0] += (p*Math.sin(robotHeading) + n*Math.cos(robotHeading)) * metersPerCount;
        line 150 (robotPosition[1] += s * Math.sin(robotHeading);
        and putting 151 (robotHeading += headingChange;) after line 146 (previousRightCounts = rightCounts;

        and putting these lines:
            double p = (leftChange + rightChange) / 2.0; // average encoder ticks, tracking robot as whole rather than one side
            double n = (leftChange-rightChange); //horizontalChange
        before line 149
         */

    }

    /**
     * acts as an update method, that also returns the value it is setting the target position to, works by cycling targets
     *
     * @param timeSeconds time elapsed during match, measured in seconds, used to differentiate between mid game and end game targets
     * @return the position of the target selected, it also sets targetPosition to these coordinates
     */
    public double[] bestTargetPosition(double timeSeconds) {
        int curTar = currentTarget;
        //cycle targets
        curTar++;
        //make sure it stays in bounds
        if (curTar > 5) curTar = 0;

        //if the robot is out of launch area, it can only target the low goal (5)
        if (robotPosition[1] > launchZone && curTar != 5)
            curTar = 5;  //TODO: 10/19/2020 change the > to < depending on what the launch zone actually is
            // if it's not the endgame, stop it from aiming at a power shot
        else if (timeSeconds <= 200 && curTar <= 2) curTar += 3;
        //otherwise, it's the endgame, and it is able to aim at anything, so do no further modifications

        //actually change the target
        currentTarget = curTar;
        targetPosition = getTargetPosition(curTar);
        return targetPosition;
    }

    ////////////////////////////// get methods //////////////////////////////

    /**
     * @return robotHeading:     the heading, in radians, that the robot is facing
     */
    public double getRobotHeading() {
        return robotHeading;
    }

    /**
     * @return robotPosition:   the position (x,y) of the robot on the field, measured in meters
     */
    public double[] getRobotPosition() {
        return robotPosition;
    }

    /**
     * @return targetPosition:  the position (x,y,z) of the current target, measured in meters
     */
    public double[] getTargetPosition() {
        return targetPosition;
    }

    public double[] getTargetPosition(int i) {
        return targets[i];
    }

    /**
     * @return a description of the current target
     */
    public String getCurrentTargetDesc() {
        /*
        r = 0, power shot 1
        r = 1, power shot 2
        r = 2, power shot 3
        r = 3, high goal
        r = 4, medium goal
        r = 5, low goal
         */
        String description;
        switch (currentTarget) {
            case 0:
                description = "power shot 1";
                break;
            case 1:
                description = "power shot 2";
                break;
            case 2:
                description = "power shot 3";
                break;
            case 3:
                description = "high goal";
                break;
            case 4:
                description = "medium goal";
                break;
            case 5:
                description = "low goal";
                break;
            default:
                description = "no target selected, or unknown";
                break;
        }
        return description;
    }

    /**
     * @return the integer representing the current target
     */
    public int getCurrentTarget() {
        return currentTarget;
    }

    ////////////////////////////// set methods //////////////////////////////
    public void setTarget(int i) {
        if (i > 5 || i < 0) i=3;
        currentTarget = 3;
        targetPosition = getTargetPosition(i);
    }
    public void cycleTarget() {
        currentTarget++;
        if (currentTarget > 5) currentTarget=0;
        targetPosition = getTargetPosition(currentTarget);
    }
}