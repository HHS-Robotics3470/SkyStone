package org.firstinspires.ftc.teamcode;

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
    double launchZone = 2.06375/*+- 0.0254m*/;       //any position with a y coordinate less (maybe more than?) than launchZone is in the launch zone

    double heading = 0;

    double wheelRadius;
    double robotWidth;
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
    ////////////////////////////// update method //////////////////////////////
    //TODO: 10/18/2020 optimize these if/else statements sometime
    public void update(double leftRotations, double rightRotations) {
        double positionChange;
        double headingChange;

        //calculate the change in position, and heading
        if (leftRotations == rightRotations) {
            positionChange = leftRotations * 2 * Math.PI * wheelRadius;
            headingChange = 0;
        } else if (leftRotations > 0 && rightRotations > 0) {
            //TODO: 10/18/2020 calculate this reeee

            // variables
            double s1 = leftRotations * 2 * Math.PI * wheelRadius; //distance the left wheel traveled (m)
            double s2 = rightRotations * 2 * Math.PI * wheelRadius; //distance the right wheel traveled (m)
            double r; //radius of the inner circle

            if (s1 > s2) {r = () / s1};
            /*
            s1/r1 = s2/r2
            s1 * r2 = s2 * (r + 18)
            s1 * r = s2 *r + s2 * 18
            s1 * r -
             */

        } else if (leftRotations < 0 || rightRotations < 0) {
            //TODO: calculate this reeee
        }

        //call a method to choose the best target at the moment
        //targetPosition = bestTargetPosition(robotPosition);
    }
    ////////////////////////////// calculate method //////////////////////////////
    ////////////////////////////// get methods //////////////////////////////
}
