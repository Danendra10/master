var mainApp = angular.module("mainApp", []);

function cekColor(elem, value, limit) {
  if( Math.abs(value) > limit ) {
    elem.className = "";
    elem.className = "badge badge-danger";
  }else{
    elem.className = "";
    elem.className = "badge badge-success";
  }
}

mainApp.controller("mainAppController", function ($scope, $interval) {
  $scope.ns = "";
  
  // Agar scope dapat diakses sebagai global variabel
  window.$scope = $scope;
  
  $scope.menu_activeIndex = 1;

  $scope.menu_buttonState = function (index) {
    if (index == $scope.menu_activeIndex) return "btn-success";
    else return "btn-outline-success";
  };

  $scope.menu_contentState = function (index) {
    if (index == $scope.menu_activeIndex) return true;
    else return false;
  };

  $scope.changeActiveIndex = function (index) {
    $scope.menu_activeIndex = index;

    // let imgs = document.getElementsByTagName('img');

    // for(let img of imgs)
    // {
    //   let imgsrc = img.src;
    //   img.src = '';

    //   setTimeout(() => {
    //     img.src = imgsrc;
    //   }, 500);
    // }

  };

  $scope.field_thresholdParameter = [
    { label: "H-", value: 0, min: 0, max: 180 },
    { label: "H+", value: 180, min: 0, max: 180 },
    { label: "S-", value: 0, min: 0, max: 255 },
    { label: "S+", value: 255, min: 0, max: 255 },
    { label: "V-", value: 0, min: 0, max: 255 },
    { label: "V+", value: 255, min: 0, max: 255 }
  ];

  $scope.ball_thresholdParameter = [
    { label: "H-", value: 0, min: 0, max: 180 },
    { label: "H+", value: 180, min: 0, max: 180 },
    { label: "S-", value: 0, min: 0, max: 255 },
    { label: "S+", value: 255, min: 0, max: 255 },
    { label: "V-", value: 0, min: 0, max: 255 },
    { label: "V+", value: 255, min: 0, max: 255 }
  ];

  $scope.obverse_thresholdParameter = [
    { label: "H-", value: 0, min: 0, max: 180 },
    { label: "H+", value: 180, min: 0, max: 180 },
    { label: "S-", value: 0, min: 0, max: 255 },
    { label: "S+", value: 255, min: 0, max: 255 },
    { label: "V-", value: 0, min: 0, max: 255 },
    { label: "V+", value: 255, min: 0, max: 255 }
  ];

  $scope.line_thresholdParameter = [
    { label: "H-", value: 0, min: 0, max: 180 },
    { label: "H+", value: 180, min: 0, max: 180 },
    { label: "L-", value: 0, min: 0, max: 255 },
    { label: "L+", value: 255, min: 0, max: 255 },
    { label: "S-", value: 0, min: 0, max: 255 },
    { label: "S+", value: 255, min: 0, max: 255 }
  ];

  //============
  //-----Updater
  //============
  $interval(function () {

  }, 1000);

  //========
  //-----ROS
  //========
  $scope.ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9900"
  });

  // GET AUTO NAMESPACES
  $scope.simParam = new ROSLIB.Param({
    ros: $scope.ros,
    name: 'is_sim'
  });

  $scope.simParam.get((value) => {
    if (value == true) {

      $scope.ros.getParams((params) => {
        let robotName = '';
        params.forEach(param => {
          robotName = param.includes("robot_name") ? param.match(/(?<=\/).*(?=\/)/) : robotName;
        });
    
        $scope.ns = '/' + robotName;
        console.info("This Robot Run In [SIMULATION] Mode with [ns:" + $scope.ns + "].");
    
      });
    }
    else if(value == false) {
      $scope.ns = '';
      console.info("This Robot Run In [HARDWARE] with NO [ns].");

    }

  });

  $scope.frame_fieldRawThreshold = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/vision_field/raw_threshold&type=ros_compressed";

  $scope.frame_ballRawThreshold = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/vision_ball/raw_threshold&type=ros_compressed";

  $scope.frame_obstacleDisplayOut = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/vision_obstacle/display_out&type=ros_compressed";
  
  // $scope.frame_obverseRawThreshold = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/vision_obverse/raw_threshold&type=ros_compressed";
  
  // $scope.frame_obverseDisplayOut = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/vision_obverse/display_out&type=ros_compressed";
  
  // $scope.frame_calibrateRawThreshold = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/vision_calibrate/raw_threshold&type=ros_compressed";

  $scope.frame_whitePoints = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/vision_line/white_points&type=ros_compressed";
  $scope.frame_whiteLine = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/vision_line/white_line&type=ros_compressed";
  $scope.frame_localizationDebuger = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/localization/debuger&type=ros_compressed";


  //========================
  //-----HSV Threshold Topic
  //========================
  $scope.top_field_hsv = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "vision_field/hsv_threshold",
    messageType: "std_msgs/UInt16MultiArray"
  });
  $scope.top_ball_hsv = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "vision_ball/hsv_threshold",
    messageType: "std_msgs/UInt16MultiArray"
  });
  $scope.top_obverse_hsv = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "vision_obverse/hsv_threshold",
    messageType: "std_msgs/UInt16MultiArray"
  });
  // $scope.top_calibrate_hsv = new ROSLIB.Topic({
  //   ros: $scope.ros,
  //   name: "vision_calibrate/hsv_threshold",
  //   messageType: "std_msgs/UInt16MultiArray"
  // });
  //========================
  //-----HLS Threshold Topic
  //========================
  $scope.top_line_hls = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "vision_line/hls_threshold",
    messageType: "std_msgs/UInt16MultiArray"
  });

  //========================
  //-----HSV Threshold Msg
  //========================
  $scope.msg_hsv = new ROSLIB.Message({
    layout: { dim: [], data_offset: 0 },
    data: [0, 0, 0, 0, 0, 0]
  });
  //========================
  //-----HLSThreshold Msg
  //========================
  $scope.msg_hls = new ROSLIB.Message({
    layout: { dim: [], data_offset: 0 },
    data: [0, 0, 0, 0, 0, 0]
  });

  //============================
  //-----HSV Threshold Publisher
  //============================
  $scope.pub_field_threshold_value = function (object) {
    for (var i = 0; i < 6; i++)
      $scope.msg_hsv.data[i] = object[i].value;
    $scope.top_field_hsv.publish($scope.msg_hsv);
  };
  $scope.pub_ball_threshold_value = function (object) {
    for (var i = 0; i < 6; i++)
      $scope.msg_hsv.data[i] = object[i].value;
    $scope.top_ball_hsv.publish($scope.msg_hsv);
  };
  $scope.pub_obverse_threshold_value = function (object) {
    for (var i = 0; i < 6; i++)
      $scope.msg_hsv.data[i] = object[i].value;
    $scope.top_obverse_hsv.publish($scope.msg_hsv);
  };
  // $scope.pub_calibrate_threshold_value = function (object) {
  //   for (var i = 0; i < 6; i++)
  //     $scope.msg_hsv.data[i] = object[i].value;
  //   $scope.top_calibrate_hsv.publish($scope.msg_hsv);
  // };
  //============================
  //-----HLS Threshold Publisher
  //============================
  $scope.pub_line_threshold_value = function (object) {
    for (var i = 0; i < 6; i++)
      $scope.msg_hls.data[i] = object[i].value;
    $scope.top_line_hls.publish($scope.msg_hls);
  };

  //==================================
  //-----HSV Threshold Service Request
  //==================================
  $scope.req_hsv = new ROSLIB.ServiceRequest({
  });
  //==================================
  //-----HLS Threshold Service Request
  //==================================
  $scope.req_hls = new ROSLIB.ServiceRequest({
  });

  //=================================
  //-----HSV Threshold Service Client
  //=================================
  $scope.ser_field_hsv = new ROSLIB.Service({
    ros: $scope.ros,
    name: "vision_field/hsv_threshold_init",
    messageType: "iris_its/hsv_init"
  });
  $scope.ser_field_hsv.callService($scope.req_hsv, res => {
    for (var i = 0; i < 6; i++)
      $scope.field_thresholdParameter[i].value = res.hsv_threshold[i];
  });

  $scope.ser_ball_hsv = new ROSLIB.Service({
    ros: $scope.ros,
    name: "vision_ball/hsv_threshold_init",
    messageType: "iris_its/hsv_init"
  });
  $scope.ser_ball_hsv.callService($scope.req_hsv, res => {
    for (var i = 0; i < 6; i++)
      $scope.ball_thresholdParameter[i].value = res.hsv_threshold[i];
  });

  // $scope.ser_obverse_hsv = new ROSLIB.Service({
  //   ros: $scope.ros,
  //   name: "vision_obverse/hsv_threshold_init",
  //   messageType: "iris_its/hsv_init"
  // });
  // $scope.ser_obverse_hsv.callService($scope.req_hsv, res => {
  //   for (var i = 0; i < 6; i++)
  //     $scope.obverse_thresholdParameter[i].value = res.hsv_threshold[i];
  // });

  // $scope.ser_calibrate_hsv = new ROSLIB.Service({
  //   ros: $scope.ros,
  //   name: "vision_calibrate/hsv_threshold_init",
  //   messageType: "iris_its/hsv_init"
  // });
  // $scope.ser_calibrate_hsv.callService($scope.req_hsv, res => {
  //   for (var i = 0; i < 6; i++)
  //     $scope.calibrate_thresholdParameter[i].value = res.hsv_threshold[i];
  // });
  //=================================
  //-----HLS Threshold Service Client
  //=================================
  // $scope.ser_line_hls = new ROSLIB.Service({
  //   ros: $scope.ros,
  //   name: "vision_line/hls_threshold_init",
  //   messageType: "iris_its/hls_init"
  // });
  // $scope.ser_line_hls.callService($scope.req_hls, res => {
  //   for (var i = 0; i < 6; i++)
  //     $scope.line_thresholdParameter[i].value = res.hls_threshold[i];
  // });

  //=============
  //-----Odometry
  //=============
  $scope.top_odometry = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "stm2pc/odometry_buffer",
    messageType: "geometry_msgs/Pose2D"
  });
  $scope.top_odometry.subscribe(msg => {
    $scope.pos_x = msg.x;
    $scope.pos_y = msg.y;
    $scope.theta = msg.theta;
  });

  //=============
  //-----AMCL
  //=============
  $scope.top_amclPose = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "localization/amclPose",
    messageType: "geometry_msgs/Pose2D"
  });
  $scope.top_amclPose.subscribe(msg => {
    $scope.amcl_x = msg.y;
    $scope.amcl_y = msg.x;
    $scope.amcl_theta = msg.theta;

    if($scope.amcl_theta > 180){
      $scope.amcl_theta = $scope.amcl_theta - 360;
    }

    // $scope.error_x = $scope.amcl_x - $scope.pos_x;
    // $scope.error_y = $scope.amcl_y - $scope.pos_y;
    // $scope.error_theta = Math.abs( $scope.amcl_theta ) - Math.abs( $scope.theta );

    // var er_x = document.getElementById("er_x");
    // var er_y = document.getElementById("er_y");
    // var er_theta = document.getElementById("er_theta");

    // cekColor(er_x, $scope.error_x, 10);
    // cekColor(er_y, $scope.error_y, 10);
    // cekColor(er_theta, $scope.error_theta, 5);
  });

  //=============
  //-----Velocity
  //=============
  $scope.top_velocity = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "pc2stm/velocity",
    messageType: "geometry_msgs/Twist"
  });
  $scope.top_velocity.subscribe(msg => {
    $scope.vel_x = msg.linear.x;
    $scope.vel_y = msg.linear.y;
    $scope.omega = msg.angular.z;
  });

  //==================
  //-----Offset Blue on Field for KRI Online
  //==================
  // $scope.top_calibrate = new ROSLIB.Topic({
  //   ros: $scope.ros,
  //   name: "robot_vision_calibrate",
  //   messageType: "iris_its/VisionCalibrate"
  // });
  // $scope.top_calibrate.subscribe(msg => {
  //   $scope.offset_x = msg.calibrate_x_pada_lapangan;
  //   $scope.offset_y = msg.calibrate_y_pada_lapangan;
  //   $scope.offset_theta = msg.calibrate_theta_pada_lapangan;
  // });

  //========================
  //-----KeyPress Topic
  //========================
  $scope.top_web_key = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "web2pc/keypress",
    messageType: "std_msgs/String"
  });
  //========================
  //-----KeyPress Msg
  //========================
  $scope.msg_web_key = new ROSLIB.Message({
    // layout: { dim: [], data_offset: 0 },
    data: 0
  });
  //============================
  //-----KeyPress Publisher
  //============================
  document.addEventListener('keydown', (e) => {
    if(e.keyCode == 32 && e.target == document.body) {
      e.preventDefault();
    }
    
    let cmd = e.key
    cmd = cmd.toString();

    console.log("Pressed :", cmd, cmd.length);

    if(cmd.length != 1)
      return;
    
    $scope.msg_web_key.data = cmd;
    $scope.top_web_key.publish($scope.msg_web_key);
  });

  $scope.togleFrameKiri = function() {
    var frameKiri = document.getElementById("frameKiri");
    console.log(frameKiri);
    
    if(frameKiri.src == $scope.frame_obstacleDisplayOut)
      frameKiri.src = $scope.frame_whiteLine
    else
      frameKiri.src = $scope.frame_obstacleDisplayOut

  }

});
