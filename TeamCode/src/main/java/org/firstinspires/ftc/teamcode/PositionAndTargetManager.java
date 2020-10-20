package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PositionAndTargetManager {
    /**info found: https://firstinspiresst01.blob.core.windows.net/first-game-changers/ftc/field-setup-guide.pdf starting page 8
     *        and: https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-2-remote-events.pdf starting page 26
     * Coords format: x,y,z
     * Origin (0,0) : center of the 6x6(square) grid
     *      field x bounds [-1.79705m,+1.79705m]
     *      field y bounds [-1.79705m,+1.79705m]
     * z = height from the foam grid
     * x+ = toward red alliance station (right from audience area perspective)
     * x- = toward blue alliance station
     * y+ = toward the tower goal and power shot targets
     * x- = toward audience
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
            {0.09525,1.79705,0.784225},     /*power shot 1*/
            {0.28575,1.79705,0.784225},     /*power shot 2*/
            {0.47625,1.79705,0.784225},     /*power shot 3*/
            {0.8905875,1.79705,0.911225},   /*high goal*/
            {-0.8905875,1.79705,0.6873875}, /*medium goal*/
            {0.8905875,1.79705,0.4318}      /*low goal*/
    };

    /*
        [r]
        r = 0, x
        r = 1, y
        r = 2, z
     */
    //initializes assuming it's on team red, in the constructor, values will be changed as needed if it's on team blue
    double[] robotPosition = new double[3]; //TODO: 10/18/2020 replace "new double[3];" with the array of the starting position if on red team
    double[] targetPosition = new double[3];
    double launchZone = 2.06375-1.79705/*+- 0.0254m*/;       //any position with a y coordinate less (maybe more than?) than launchZone is in the launch zone

    double heading = 90;

    double wheelRadius;
    double robotWidth;

    short powerShotsHit = 0;
    ////////////////////////////// constructors //////////////////////////////
    /**
     * constructor for the PositionAndTargetManager
     * @param isTeamRed pass as true if the robot is on the red alliance
     */
    public PositionAndTargetManager(HardwareUltimateGoal robot, boolean isTeamRed) {
        //take some variables from the robot
        wheelRadius = robot.driveWheelRadius;
        robotWidth = robot.robotWidth;
        //flip some things for if the robot is on blue team
        if (!isTeamRed) {
            for (int r = 0; r < targets.length; r ++) {
                targets[r][0] *= -1; //flip the x-axis if not on the red team
            }

            robotPosition[0] *= -1; //flip the x-axis if not on the red team
        }
    }
    ////////////////////////////// update and calculate method //////////////////////////////
    //TODO: 10/18/2020 optimize these if/else statements sometime
    public void update(double leftRotations, double rightRotations, ElapsedTime timeElapsed) {
        double positionChange;
        double headingChange = 0;
        //calculate in radians, convert heading back into degrees after

        //calculate the change in position, and heading
        if (leftRotations == rightRotations) { // both sides going either forward or backward same speed;
            positionChange = leftRotations * 2 * Math.PI * wheelRadius;
        } else if ( (leftRotations < 0 && rightRotations < 0) || (leftRotations > 0 && rightRotations > 0) ) { //both sides either going forward, or backward (different speed) (coded with forward in mind)
            //TODO: 10/18/2020 check if this works when the robot is going backwards too
            //variables
            double s1 = leftRotations * 2 * Math.PI * wheelRadius; //distance the left wheel traveled (m)
            double s2 = rightRotations * 2 * Math.PI * wheelRadius; //distance the right wheel traveled (m)
            double r = 0; //radius of the inner circle

            //calculate r, headingChange, and position change
            if (s1 > s2) {
                r = (s1 - s2) / (s2 * robotWidth); // calculating r
                headingChange = s2/r;//https://www.desmos.com/calculator/r16kcermq2
            }
            else if (s1 < s2) {
                r = (s2 - s1) / (s1 * robotWidth);// calculating r
                headingChange = s1/r; //calculating headingChange
            }
            positionChange = Math.sqrt( Math.pow((r + robotWidth/2)*Math.cos(Math.toRadians(heading) + headingChange) - robotPosition[0], 2) + Math.pow((r + robotWidth/2)*Math.sin(Math.toRadians(heading) + headingChange) - robotPosition[1], 2) );//sqrt of (delta x)^2 + (delta y)^2
        } else if (leftRotations < 0 || rightRotations < 0) { //one side going forward, other going backward
            //TODO: 10/19/2020 check if this works when the robot is going backwards too
            //variables
            double s1 = leftRotations * 2 * Math.PI * wheelRadius; //distance the left wheel traveled (m)
            double s2 = rightRotations * 2 * Math.PI * wheelRadius; //distance the right wheel traveled (m)
            double r; //radius of the inner circle

            //calculate r, headingChange, and position change
            if (Math.abs(s1) > Math.abs(s2)) { //turning clockwise
                r = (robotWidth * s2) / (s1 + s2);
                headingChange = s1/r; //calculating headingChange
            } else { //turning counter clockwise
                r = (robotWidth * s1) / (s2 + s1);
                headingChange = s2/r; //calculating headingChange
            }

            positionChange = Math.sqrt( Math.pow((robotWidth/2 -r)*Math.cos(Math.toRadians(heading) + headingChange) - robotPosition[0], 2) + Math.pow((robotWidth/2 -r)*Math.sin(Math.toRadians(heading) + headingChange) - robotPosition[1], 2) );//sqrt of (delta x)^2 + (delta y)^2
        } else { //one side isn't moving
            //TODO: 10/19/2020 check if this works when the robot is going backwards too
            //variables
            double s = leftRotations * 2 * Math.PI * wheelRadius; //distance the left wheel traveled (m)
            if (s == 0) {
                s = rightRotations * 2 * Math.PI * wheelRadius; //distance the right wheel traveled (m)
            }
            double r = robotWidth;

            headingChange = s/r;

            positionChange = Math.sqrt( Math.pow((robotWidth/2)*Math.cos(Math.toRadians(heading) + headingChange) - robotPosition[0], 2) + Math.pow((robotWidth/2)*Math.sin(Math.toRadians(heading) + headingChange) - robotPosition[1], 2) );//sqrt of (delta x)^2 + (delta y)^2
        }

        //store changes to position and heading
        heading += Math.toDegrees(headingChange);
        robotPosition[0] += Math.cos(heading) * positionChange;
        robotPosition[1] += Math.sin(heading) * positionChange;

        //call a method to choose the best target at the moment
        bestTargetPosition(timeElapsed);
    }

    public void bestTargetPosition(ElapsedTime time) {
        double bestTarget[];

        int i = (int) (Math.random() * (targets.length - 2)); //random row of targets -1
        if (i <= 3 && i > 4) { //prevents the robot from shooting the powershots before endgame, and increased the chance of aiming to the high goal
            i = 3;
        } // at this point, i should be either 3 or 4
        //TODO: 10/19/2020 change the > to < depending on what the launch zone actually is
        if (robotPosition[1] > launchZone) { // if the robot is outside of the launchZone
            i = 5; //target low goal
        }
        bestTarget = targets[i];

        if (time.seconds() >= 200 && (int) (Math.random()*3 + 1) == 2 && powerShotsHit <= 2) { //if it's the endgame,  and rng (1/3)
            bestTarget = targets[powerShotsHit]; // cycle through the powershots
            powerShotsHit ++;
        }

        targetPosition = bestTarget;
    }
    ////////////////////////////// get methods //////////////////////////////
}