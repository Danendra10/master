$(function () {
    const MULT = 0.5;
    const FIELD_WIDTH = 1200 * MULT;
    const FIELD_HEIGHT = 800 * MULT;
    const BORDER_X = 100 * MULT;
    const BORDER_Y = 110 * MULT;
    const MAXWIDTH = (FIELD_WIDTH + (2 * BORDER_X));
    const MAXHEIGHT = (FIELD_HEIGHT + (2 * BORDER_Y));
    const CENTERX = ((FIELD_WIDTH / 2) + BORDER_X);
    const CENTERY = ((FIELD_HEIGHT / 2) + BORDER_Y);

    const PENALTY_WIDTH = 180 * MULT;
    const PENALTY_HEIGHT = 525 * MULT;

    const KEEPER_WIDTH = 50 * MULT;
    const KEEPER_HEIGHT = 325 * MULT;

    const GOAL_WIDTH = 80 * MULT;
    const GOAL_HEIGHT = 200 * MULT;

    var fieldCanvas = new fabric.StaticCanvas('fieldCanvas');

    var greenField = new fabric.Rect({
        left: 0,
        top: 0,
        fill: 'green',
        width: MAXWIDTH,
        height: MAXHEIGHT,
        selectable: false,
    });

    fieldCanvas.add(greenField);

    var borderField = new fabric.Rect({
        left: BORDER_X,
        top: BORDER_Y,
        fill: 'transparent',
        stroke: 'white',
        strokeWidth: 3,
        width: FIELD_WIDTH - 3,
        height: FIELD_HEIGHT - 3,
        selectable: false,
    });

    fieldCanvas.add(borderField);

    var midLineField = new fabric.Line([CENTERX, BORDER_Y, CENTERX, MAXHEIGHT - BORDER_Y], {
        stroke: 'white',
        strokeWidth: 3,
        originX: 'center',
        originY: 'center',
    });

    fieldCanvas.add(midLineField);

    var midCircleField = new fabric.Circle({
        left: CENTERX,
        top: CENTERY,
        radius: 50,
        fill: 'transparent',
        stroke: 'white',
        strokeWidth: 3,
        originX: 'center',
        originY: 'center',
    });

    fieldCanvas.add(midCircleField);

    var penaltyField = new fabric.Rect({
        left: BORDER_X,
        top: CENTERY,
        width: PENALTY_WIDTH,
        height: PENALTY_HEIGHT - 3,
        fill: 'transparent',
        stroke: 'white',
        strokeWidth: 3,
        originX: 'left',
        originY: 'center'
    });

    fieldCanvas.add(penaltyField);

    var penaltyField2 = new fabric.Rect({
        left: MAXWIDTH - BORDER_X,
        top: CENTERY,
        width: PENALTY_WIDTH,
        height: PENALTY_HEIGHT - 3,
        fill: 'transparent',
        stroke: 'white',
        strokeWidth: 3,
        originX: 'right',
        originY: 'center'
    });

    fieldCanvas.add(penaltyField2);

    var keeperField = new fabric.Rect({
        left: BORDER_X,
        top: CENTERY,
        width: KEEPER_WIDTH,
        height: KEEPER_HEIGHT - 3,
        fill: 'transparent',
        stroke: 'white',
        strokeWidth: 3,
        originX: 'left',
        originY: 'center'
    });

    fieldCanvas.add(keeperField);

    var keeperField2 = new fabric.Rect({
        left: MAXWIDTH - BORDER_X,
        top: CENTERY,
        width: KEEPER_WIDTH,
        height: KEEPER_HEIGHT - 3,
        fill: 'transparent',
        stroke: 'white',
        strokeWidth: 3,
        originX: 'right',
        originY: 'center'
    });

    fieldCanvas.add(keeperField2);

    var goalField = new fabric.Rect({
        left: BORDER_X + 3,
        top: CENTERY,
        width: GOAL_WIDTH,
        height: GOAL_HEIGHT - 3,
        fill: 'transparent',
        stroke: 'white',
        strokeWidth: 3,
        originX: 'right',
        originY: 'center'
    });

    fieldCanvas.add(goalField);

    var goalField2 = new fabric.Rect({
        left: MAXWIDTH - BORDER_X - 3,
        top: CENTERY,
        width: GOAL_WIDTH,
        height: GOAL_HEIGHT - 3,
        fill: 'transparent',
        stroke: 'white',
        strokeWidth: 3,
        originX: 'left',
        originY: 'center'
    });

    fieldCanvas.add(goalField2);

    //========================================================
    var objectCanvas = new fabric.StaticCanvas('objectCanvas');

    var robotBody = new fabric.Circle({
        left: 0,
        top: 0,
        radius: 15,
        fill: 'transparent',
        stroke: 'red',
        strokeWidth: 3,
        originX: 'center',
        originY: 'center',
    });
    // objectCanvas.add(robot);

    var robotAngle = new fabric.Line([15, 0, 0, 0], {
        stroke: 'red',
        strokeWidth: 3,
        originX: 'center',
        originY: 'center',
    });

    // objectCanvas.add(robotAngle);
    var robot = new fabric.Group([robotBody, robotAngle], {
        left: BORDER_X + 0 * MULT,
        top: BORDER_Y + 0 * MULT,
        originX: 'center',
        originY: 'center',
        angle: '0'
    });
    objectCanvas.add(robot);

    var particleBody = new fabric.Circle({
        left: 0,
        top: 0,
        radius: 7,
        fill: 'transparent',
        stroke: 'blue',
        strokeWidth: 3,
        originX: 'center',
        originY: 'center',
    });

    var particleAngle = new fabric.Line([7, 0, 0, 0], {
        stroke: 'blue',
        strokeWidth: 3,
        originX: 'center',
        originY: 'center',
    });

    var particles = new Array(400);
    for(let i = 0;i < particles.length; i++) {
        var particle = new fabric.Group([particleBody, particleAngle], {
            left: 0 * MULT,
            top: 0 * MULT,
            originX: 'center',
            originY: 'center',
            angle: '0'
        });
        objectCanvas.add(particle);
        particles[i] = particle;
    }

    var prediksiRobotBody = new fabric.Circle({
        left: 0,
        top: 0,
        radius: 15,
        fill: 'transparent',
        stroke: 'cyan',
        strokeWidth: 3,
        originX: 'center',
        originY: 'center',
    });

    var prediksiRobotAngle = new fabric.Line([15, 0, 0, 0], {
        stroke: 'cyan',
        strokeWidth: 3,
        originX: 'center',
        originY: 'center',
    });

    var prediksiRobot = new fabric.Group([prediksiRobotBody, prediksiRobotAngle], {
        left: BORDER_X + 0 * MULT,
        top: BORDER_Y + 0 * MULT,
        originX: 'center',
        originY: 'center',
        angle: '0'
    });
    objectCanvas.add(prediksiRobot);

    var ball = new fabric.Circle({
        left: BORDER_X + 0,
        top: BORDER_Y + 0,
        radius: 6,
        fill: 'orange',
        stroke: 'orange',
        strokeWidth: 3,
        originX: 'center',
        originY: 'center',
    });
    objectCanvas.add(ball);

    var scanPoints = new Array(144);
    for (var i = 0; i < 144; i++) {
        var scanPoint = new fabric.Circle({
            left: BORDER_X + -999,
            top: BORDER_Y + -999,
            radius: 1,
            fill: 'transparent',
            stroke: 'red',
            strokeWidth: 3,
            originX: 'center',
            originY: 'center',
        });
        objectCanvas.add(scanPoint);
        scanPoints[i] = scanPoint;
    }

    var prediksiScanPoints = new Array(144);
    for (var i = 0; i < 144; i++) {
        var scanPoint = new fabric.Circle({
            left: BORDER_X + 0,
            top: BORDER_Y + 0,
            radius: 1,
            fill: 'transparent',
            stroke: 'cyan',
            strokeWidth: 3,
            originX: 'center',
            originY: 'center',
        });
        objectCanvas.add(scanPoint);
        prediksiScanPoints[i] = scanPoint;
    }

    // Converts from degrees to radians.
    function degrees_to_radians(degrees) {
        var pi = Math.PI;
        return degrees * (pi/180);
    }

    // Converts from radians to degrees.
    function radians_to_degrees(degrees)
    {
        var pi = Math.PI;
        return degrees * (pi/180);
    }

    function loop() {
        robot.set('top', BORDER_Y + $scope.pos_x * MULT);
        robot.set('left', BORDER_X + $scope.pos_y * MULT);
        robot.set('angle', 90 - $scope.theta);
        // ball.set('top', BORDER_Y + $scope.ball_x * MULT);
        // ball.set('left', BORDER_X + $scope.ball_y * MULT);

        prediksiRobot.set('top', BORDER_Y + $scope.prediksi_x * MULT);
        prediksiRobot.set('left', BORDER_X + $scope.prediksi_y * MULT);
        prediksiRobot.set('angle', 90 - $scope.prediksi_theta);

        for (let i = 0; i < 144; i++) {
            if($scope.scan_point[i] != 0) {
                var scan_x = $scope.scan_point[i] * Math.cos(degrees_to_radians(i * 2.5));
                var scan_y = $scope.scan_point[i] * Math.sin(degrees_to_radians(i * 2.5));

                // scanPoints[i].set('top', BORDER_Y + ($scope.pos_x + scan_x) * MULT);
                // scanPoints[i].set('left', BORDER_X + ($scope.pos_y + scan_y) * MULT);

                var pred_c = Math.cos(degrees_to_radians($scope.theta - $scope.prediksi_theta));
                var pred_s = Math.sin(degrees_to_radians($scope.theta - $scope.prediksi_theta));
                prediksiScanPoints[i].set('top', BORDER_Y + ($scope.prediksi_x + (pred_c * scan_x + pred_s * scan_y)) * MULT);
                prediksiScanPoints[i].set('left', BORDER_X + ($scope.prediksi_y + (-pred_s * scan_x + pred_c * scan_y)) * MULT);
            }
            else {
                // scanPoints[i].set('top', -99);
                // scanPoints[i].set('left', -99);
                prediksiScanPoints[i].set('top', -99);
                prediksiScanPoints[i].set('left', -99);
            }
        }

        // for(let i = 0;i < particles.length; i++) {
        //     particles[i].set('top', BORDER_Y + $scope.particles[i].x * MULT);
        //     particles[i].set('left', BORDER_X + $scope.particles[i].y * MULT);
        //     particles[i].set('angle', 90 - $scope.particles[i].theta);
        // }

        objectCanvas.renderAll();
    }

    var myTimer = setInterval(loop, 100);
});