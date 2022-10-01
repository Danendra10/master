var mainApp = angular.module("mainApp", []);

function cekColor(elem, value, limit) {
  if (Math.abs(value) > limit) {
    elem.className = "";
    elem.className = "badge badge-danger";
  } else {
    elem.className = "";
    elem.className = "badge badge-success";
  }
}

mainApp.controller("mainAppController", function ($scope, $interval) {
  $scope.ns = "";

  // Agar scope dapat diakses sebagai global variabel
  window.$scope = $scope;

  //Area buat resume jeda video
  $scope.stream_vision = [true, true, true];

  $scope.pause_start_stream = function (index_stream) {
    console.log($scope.stream_vision[index_stream]);
    if ($scope.stream_vision[index_stream]) {
      $scope.stream_vision[index_stream] = false;
      if (index_stream == 0) {
        $scope.frame_obstacleDisplayOut = "None";
      } else if (index_stream == 1) {
        $scope.frame_fieldRawThreshold = "None";
      } else if (index_stream == 2) {
        $scope.frame_ballRawThreshold = "None";
      }
    } else {
      $scope.stream_vision[index_stream] = true;
      if (index_stream == 0) {
        $scope.frame_obstacleDisplayOut =
          "http://" +
          window.location.hostname +
          ":9901" +
          "/stream?topic=" +
          $scope.ns +
          "/vision_obstacle/display_out&type=ros_compressed";
      } else if (index_stream == 1) {
        $scope.frame_fieldRawThreshold =
          "http://" +
          window.location.hostname +
          ":9901" +
          "/stream?topic=" +
          $scope.ns +
          "/vision_field/raw_threshold&type=ros_compressed";
      } else if (index_stream == 2) {
        $scope.frame_ballRawThreshold =
          "http://" +
          window.location.hostname +
          ":9901" +
          "/stream?topic=" +
          $scope.ns +
          "/vision_ball/raw_threshold&type=ros_compressed";
      }
    }
  };

  $scope.get_stream_status = function (index_stream) {
    return $scope.stream_vision[index_stream];
  };

  $scope.stream_button_state = function (index_stream) {
    if (!$scope.stream_vision[index_stream]) {
      return { text: "Resume", css: "hover:bg-sky-700 bg-sky-500" };
    } else {
      return { text: "Pause", css: "hover:bg-rose-700 bg-rose-500" };
    }
  };

  // ---------------------------------------

  //Area Untuk Regresi
  $scope.jarak_sekarang = 0;
  $scope.array_regresi = [];

  // $scope.regresi_pixel_data = function()  {
  //   const regresi_jarak_awal = document.querySelector("#jarak-awal");
  //   const regresi_selisih_jarak = document.querySelector("#selisih-jarak");
  //   const regresi_textarea = document.querySelector("#regression-area");
  //   regresi_textarea.scrollTop = regresi_textarea.scrollHeight
  //   if ($scope.jarak_sekarang === 0) {
  //     $scope.jarak_sekarang = regresi_jarak_awal.value;
  //   }
  //   $scope.pixel_sekarang = $scope.jarak_bola_pada_lapangan;
  //   if ($scope.array_regresi.length!=0) {
  //     $scope.array_regresi = regresi_textarea.value.split('\n')
  //   }

  //   $scope.array_regresi.push(`${$scope.jarak_sekarang} ${$scope.pixel_sekarang}`)
  //   regresi_textarea.value = $scope.array_regresi.join('\n')
  //   $scope.jarak_sekarang = Number($scope.jarak_sekarang) + Number(regresi_selisih_jarak.value)
  // }

  $scope.reset_regresi = function () {
    const regresi_textarea = document.querySelector("#regression-area");
    regresi_textarea.value = "";
    $scope.jarak_sekarang = 0;
    $scope.array_regresi = [];
  };

  $scope.convert_format = function () {
    const array_regresi = document
      .querySelector("#data-regresi")
      .value.split("\n");
    const converted_format = array_regresi.map((current, index) => {
      return `orde_${index}: ${current}`;
    });
    document.querySelector("#formated").value = `max_orde : ${
      array_regresi.length - 1
    }\n${converted_format.join("\n")}`;
    console.log(array_regresi);
  };

  // -----------------------------------------

  // UNTUK SUBSCRIBE TOPIC

  $scope.menu_activeIndex = 1;

  $scope.menu_buttonState = function (index) {
    if (index == $scope.menu_activeIndex) return "bg-white text-black";
    else return "hover:text-black";
  };

  $scope.menu_contentState = function (index) {
    if (index == $scope.menu_activeIndex) return true;
    else return false;
  };

  $scope.changeActiveIndex = function (index) {
    $scope.menu_activeIndex = index;
    if ($scope.menu_activeIndex != 7) {
      $scope.frame_original_bgr = "None";
    } else {
      $scope.frame_original_bgr =
        "http://" +
        window.location.hostname +
        ":9901" +
        "/stream?topic=" +
        $scope.ns +
        "/vision_frame_bgr&type=ros_compressed";
    }
    console.log($scope.frame_original_bgr);

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
    { label: "V+", value: 255, min: 0, max: 255 },
  ];

  $scope.ball_thresholdParameter = [
    { label: "H-", value: 0, min: 0, max: 180 },
    { label: "H+", value: 180, min: 0, max: 180 },
    { label: "S-", value: 0, min: 0, max: 255 },
    { label: "S+", value: 255, min: 0, max: 255 },
    { label: "V-", value: 0, min: 0, max: 255 },
    { label: "V+", value: 255, min: 0, max: 255 },
  ];

  $scope.obverse_thresholdParameter = [
    { label: "H-", value: 0, min: 0, max: 180 },
    { label: "H+", value: 180, min: 0, max: 180 },
    { label: "S-", value: 0, min: 0, max: 255 },
    { label: "S+", value: 255, min: 0, max: 255 },
    { label: "V-", value: 0, min: 0, max: 255 },
    { label: "V+", value: 255, min: 0, max: 255 },
  ];

  $scope.line_thresholdParameter = [
    { label: "H-", value: 0, min: 0, max: 180 },
    { label: "H+", value: 180, min: 0, max: 180 },
    { label: "L-", value: 0, min: 0, max: 255 },
    { label: "L+", value: 255, min: 0, max: 255 },
    { label: "S-", value: 0, min: 0, max: 255 },
    { label: "S+", value: 255, min: 0, max: 255 },
  ];

  $scope.camera_parameter = [
    { label: "Bright", value: 0, min: 0, max: 255 },
    { label: "Contr", value: 0, min: 0, max: 255 },
    { label: "Satur", value: 0, min: 0, max: 255 },
  ];

  //============
  //-----Updater
  //============
  $interval(function () {}, 1000);

  //========
  //-----ROS
  //========
  $scope.ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9900",
  });

  // GET AUTO NAMESPACES
  $scope.simParam = new ROSLIB.Param({
    ros: $scope.ros,
    name: "is_sim",
  });

  $scope.simParam.get((value) => {
    if (value == true) {
      $scope.ros.getParams((params) => {
        let robotName = "";
        params.forEach((param) => {
          robotName = param.includes("robot_name")
            ? param.match(/(?<=\/).*(?=\/)/)
            : robotName;
        });

        $scope.ns = "/" + robotName;
        console.info(
          "This Robot Run In [SIMULATION] Mode with [ns:" + $scope.ns + "]."
        );
      });
    } else if (value == false) {
      $scope.ns = "";
      console.info("This Robot Run In [HARDWARE] with NO [ns].");
    }
  });

  $scope.frame_fieldRawThreshold =
    "http://" +
    window.location.hostname +
    ":9901" +
    "/stream?topic=" +
    $scope.ns +
    "/vision_field/raw_threshold&type=ros_compressed";

  $scope.frame_ballRawThreshold =
    "http://" +
    window.location.hostname +
    ":9901" +
    "/stream?topic=" +
    $scope.ns +
    "/vision_ball/raw_threshold&type=ros_compressed";

  $scope.frame_obstacleDisplayOut =
    "http://" +
    window.location.hostname +
    ":9901" +
    "/stream?topic=" +
    $scope.ns +
    "/vision_obstacle/display_out&type=ros_compressed";

  $scope.frame_obverseRawThreshold =
    "http://" +
    window.location.hostname +
    ":9901" +
    "/stream?topic=" +
    $scope.ns +
    "/vision_obverse/raw_threshold&type=ros_compressed";

  $scope.frame_obverseDisplayOut =
    "http://" +
    window.location.hostname +
    ":9901" +
    "/stream?topic=" +
    $scope.ns +
    "/vision_obverse/display_out&type=ros_compressed";

  $scope.frame_original_bgr =
    "http://" +
    window.location.hostname +
    ":9901" +
    "/stream?topic=" +
    $scope.ns +
    "/vision_frame_bgr&type=ros_compressed";

  // /vision_frame_bgr&type=ros_compressed

  // $scope.frame_calibrateRawThreshold = "http://" + window.location.hostname + ":9901" + "/stream?topic="+ $scope.ns +"/vision_calibrate/raw_threshold&type=ros_compressed";

  $scope.frame_whitePoints =
    "http://" +
    window.location.hostname +
    ":9901" +
    "/stream?topic=" +
    $scope.ns +
    "/vision_line/white_points&type=ros_compressed";
  $scope.frame_whiteLine =
    "http://" +
    window.location.hostname +
    ":9901" +
    "/stream?topic=" +
    $scope.ns +
    "/vision_line/white_line&type=ros_compressed";
  $scope.frame_localizationDebuger =
    "http://" +
    window.location.hostname +
    ":9901" +
    "/stream?topic=" +
    $scope.ns +
    "/localization/debuger&type=ros_compressed";

  //========================
  //-----HSV Threshold Topic
  //========================
  $scope.top_field_hsv = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "vision_field/hsv_threshold",
    messageType: "std_msgs/UInt16MultiArray",
  });
  $scope.top_ball_hsv = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "vision_ball/hsv_threshold",
    messageType: "std_msgs/UInt16MultiArray",
  });
  $scope.top_obverse_hsv = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "vision_obverse/hsv_threshold",
    messageType: "std_msgs/UInt16MultiArray",
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
    messageType: "std_msgs/UInt16MultiArray",
  });

  //========================
  //-----HSV Threshold Msg
  //========================
  $scope.msg_hsv = new ROSLIB.Message({
    layout: { dim: [], data_offset: 0 },
    data: [0, 0, 0, 0, 0, 0],
  });
  //========================
  //-----HLSThreshold Msg
  //========================
  $scope.msg_hls = new ROSLIB.Message({
    layout: { dim: [], data_offset: 0 },
    data: [0, 0, 0, 0, 0, 0],
  });

  //============================
  //-----HSV Threshold Publisher
  //============================
  $scope.pub_field_threshold_value = function (object) {
    for (var i = 0; i < 6; i++) $scope.msg_hsv.data[i] = object[i].value;
    $scope.top_field_hsv.publish($scope.msg_hsv);
  };
  $scope.pub_ball_threshold_value = function (object) {
    for (var i = 0; i < 6; i++) $scope.msg_hsv.data[i] = object[i].value;
    $scope.top_ball_hsv.publish($scope.msg_hsv);
  };
  $scope.pub_obverse_threshold_value = function (object) {
    for (var i = 0; i < 6; i++) $scope.msg_hsv.data[i] = object[i].value;
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
    for (var i = 0; i < 6; i++) $scope.msg_hls.data[i] = object[i].value;
    $scope.top_line_hls.publish($scope.msg_hls);
  };

  //==================================
  //-----HSV Threshold Service Request
  //==================================
  $scope.req_hsv = new ROSLIB.ServiceRequest({});
  //==================================
  //-----HLS Threshold Service Request
  //==================================
  $scope.req_hls = new ROSLIB.ServiceRequest({});

  //=================================
  //-----HSV Threshold Service Client
  //=================================
  $scope.ser_field_hsv = new ROSLIB.Service({
    ros: $scope.ros,
    name: "vision_field/hsv_threshold_init",
    messageType: "iris_its/hsv_init",
  });
  $scope.ser_field_hsv.callService($scope.req_hsv, (res) => {
    for (var i = 0; i < 6; i++)
      $scope.field_thresholdParameter[i].value = res.hsv_threshold[i];
  });

  $scope.ser_ball_hsv = new ROSLIB.Service({
    ros: $scope.ros,
    name: "vision_ball/hsv_threshold_init",
    messageType: "iris_its/hsv_init",
  });
  $scope.ser_ball_hsv.callService($scope.req_hsv, (res) => {
    for (var i = 0; i < 6; i++)
      $scope.ball_thresholdParameter[i].value = res.hsv_threshold[i];
  });

  $scope.ser_obverse_hsv = new ROSLIB.Service({
    ros: $scope.ros,
    name: "vision_obverse/hsv_threshold_init",
    messageType: "iris_its/hsv_init",
  });
  $scope.ser_obverse_hsv.callService($scope.req_hsv, (res) => {
    for (var i = 0; i < 6; i++)
      $scope.obverse_thresholdParameter[i].value = res.hsv_threshold[i];
  });
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
  $scope.ser_line_hls = new ROSLIB.Service({
    ros: $scope.ros,
    name: "vision_line/hls_threshold_init",
    messageType: "iris_its/hls_init",
  });
  $scope.ser_line_hls.callService($scope.req_hls, (res) => {
    for (var i = 0; i < 6; i++)
      $scope.line_thresholdParameter[i].value = res.hls_threshold[i];
  });

  //=============
  //-----Odometry
  //=============
  $scope.top_odometry = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "stm2pc/odometry_buffer",
    messageType: "geometry_msgs/Pose2D",
  });
  $scope.top_odometry.subscribe((msg) => {
    $scope.pos_x = msg.x;
    $scope.pos_y = msg.y;
    $scope.theta = msg.theta;
  });

  //Vision Ball Subsicribe
  $scope.vision_ball = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "robot_vision_ball",
    messageType: "iris_its/VisionBall",
  });

  $scope.vision_ball.subscribe((msg) => {
    // console.log(msg.bola_x_pada_frame)
    $scope.status_bola = msg.status_bola;
    $scope.bola_x_pada_frame = msg.bola_x_pada_frame;
    $scope.bola_y_pada_frame = msg.bola_y_pada_frame;
    $scope.bola_theta_pada_frame = msg.bola_theta_pada_frame;
    $scope.bola_x_pada_lapangan = msg.bola_x_pada_lapangan;
    $scope.bola_y_pada_lapangan = msg.bola_y_pada_lapangan;
    $scope.bola_theta_pada_lapangan = msg.bola_theta_pada_lapangan;
    $scope.jarak_bola_frame = Math.sqrt(
      Math.pow($scope.bola_x_pada_frame, 2) +
        Math.pow($scope.bola_y_pada_frame, 2)
    );
    $scope.jarak_bola_frame = parseFloat($scope.jarak_bola_frame).toFixed(2);
    $scope.jarak_bola_pada_lapangan = msg.jarak_bola_pada_lapangan;
    $scope.jarak_bola_pada_lapangan = parseFloat(
      $scope.jarak_bola_pada_lapangan
    ).toFixed(2);

    $scope.regresi_pixel_data = function () {
      const regresi_jarak_awal = document.querySelector("#jarak-awal");
      const regresi_selisih_jarak = document.querySelector("#selisih-jarak");
      const regresi_textarea = document.querySelector("#regression-area");
      regresi_textarea.scrollTop = regresi_textarea.scrollHeight;
      if ($scope.jarak_sekarang === 0) {
        $scope.jarak_sekarang = regresi_jarak_awal.value;
      }
      $scope.pixel_sekarang = $scope.jarak_bola_frame;
      if ($scope.array_regresi.length != 0) {
        $scope.array_regresi = regresi_textarea.value.split("\n");
      }

      $scope.array_regresi.push(
        `${$scope.pixel_sekarang} ${$scope.jarak_sekarang}`
      );
      regresi_textarea.value = $scope.array_regresi.join("\n");
      $scope.jarak_sekarang =
        Number($scope.jarak_sekarang) + Number(regresi_selisih_jarak.value);
    };
  });

  //Sensor Garis Subscribe
  $scope.sensor_garis = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "stm2pc/sensor_garis",
    messageType: "std_msgs/UInt8",
  });

  $scope.status_sensor_garis;
  $scope.sensor_garis.subscribe((msg) => {
    $scope.status_sensor_garis = msg.data;
  });

  $scope.isKiriDetect = () => {
    const is_detected = ($scope.status_sensor_garis & 0x02) >> 1;
    if (is_detected == 0) return "bg-sky-500";
    else return "bg-rose-500 ";
  };

  $scope.isKananDetect = () => {
    const is_detected = ($scope.status_sensor_garis & 0x01) >> 0;

    if (is_detected == 0) return "bg-sky-500";
    else return "bg-rose-500 ";
  };

  //////////////////////////////////////////
  /////// AREA UNTUK AUTO THRESSHOLD ///////
  //////////////////////////////////////////

  /// PUBLISH POINT AUTO THRESSHOLDING
  $scope.pub_autothresshold = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "auto_thresshold/point",
    messageType: "std_msgs/UInt16MultiArray",
  });

  $scope.msg_autothresshold = new ROSLIB.Message({
    layout: { dim: [], data_offset: 0 },
    data: [0, 0, 0, 0],
  });

  $scope.point = [];
  $scope.stateReady = false;

  // $scope.selectPoint = function() {
  //   $(document).ready(function() {
  //     $("#at_source").on("click", function(event) {
  //         var x = event.pageX - this.offsetLeft;
  //         var y = event.pageY - this.offsetTop;

  //         var x = x * (360/270);
  //         var y = y * (640/480);
  //         console.log(x,y);
  //         if (!($scope.point.length == 4)) {
  //           $scope.point.push(Math.round(x))
  //           $scope.point.push(Math.round(y))
  //         }
  //         else{
  //           $scope.msg_autothresshold.data = $scope.point
  //           console.log($scope.msg_autothresshold);
  //           // $scope.pub_autothresshold.publish($scope.msg_autothresshold);
  //           // $scope.pub_autothresshold.publish($scope.msg_autothresshold)
  //         }
  //     });
  // });
  // }

  var prev_x, prev_y;

  $scope.selectPoint = function () {
    if (1) {
      $(document).ready(function () {
        $("#at_source").on("click", function (event) {
          var x = event.pageX - this.offsetLeft;
          var y = event.pageY - this.offsetTop;

          var x = x * (360 / 270);
          var y = y * (640 / 480);
          console.log(x, y, $scope.point.length);
          console.log(prev_x, prev_y, x, y);
          if (!($scope.point.length == 4)) {
            if ($scope.point.length == 0) {
              if ($scope.point[0] != x && $scope.point[1] != y) {
                $scope.point.push(Math.round(x));
                $scope.point.push(Math.round(y));
              }
              prev_x = x;
              prev_y = y;
            } else if ($scope.point.length >= 2) {
              if (x != prev_x && y != prev_y) {
                $scope.point.push(Math.round(x));
                $scope.point.push(Math.round(y));
              }
            }
          } else {
            $scope.msg_autothresshold.data = $scope.point;
            console.log($scope.msg_autothresshold);
            // $scope.pub_autothresshold.publish($scope.msg_autothresshold);
            // $scope.pub_autothresshold.publish($scope.msg_autothresshold)
          }
        });
      });
    }
  };

  $scope.thressholdNow = function () {
    if ($scope.point.length == 4) {
      $scope.pub_autothresshold.publish($scope.msg_autothresshold);
    } else {
      alert("Belum 2 titik");
    }
  };

  $scope.clearPoint = function () {
    $scope.stateReady = false;
    $scope.point = [];
  };

  /////////////////////////////////////////

  $scope.res_auto = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "autothresshold/result_value",
    messageType: "iris_its/AutoThresshold",
  });

  $scope.res_auto.subscribe((msg) => {
    // console.log(msg.g_s_min);
    console.log("nilainya");
    console.log(msg);
    $scope.ball_thresholdParameter[0].value = msg.g_h_min;
    $scope.ball_thresholdParameter[1].value = msg.g_h_max;
    $scope.ball_thresholdParameter[2].value = msg.g_s_min;
    $scope.ball_thresholdParameter[3].value = msg.g_s_max;
    $scope.ball_thresholdParameter[4].value = msg.g_v_min;
    $scope.ball_thresholdParameter[5].value = msg.g_v_max;
    $scope.pub_ball_threshold_value($scope.ball_thresholdParameter);
  });

  // $scope.thresshold_now = function () {
  //   console.log($scope.point);

  // }

  /////////////////////////////////
  //CAMERA REALTIME CHANGE AREAA///
  /////////////////////////////////

  $scope.top_cameraConfig = new ROSLIB.Topic({
    ros: $scope.ros,
    name: "cameraConfig/value",
    messageType: "std_msgs/UInt16MultiArray",
  });

  $scope.msg_cameraConfig = new ROSLIB.Message({
    layout: { dim: [], data_offset: 0 },
    data: [0, 0, 0],
  });

  $scope.pub_cameraConfig = function (object) {
    for (let i = 0; i < $scope.camera_parameter.length; i++) {
      $scope.msg_cameraConfig.data[i] = object[i].value;
    }
    console.log($scope.msg_cameraConfig.data);
    $scope.top_cameraConfig.publish($scope.msg_cameraConfig);
  };

  $scope.req_camera_config = new ROSLIB.ServiceRequest({});
  $scope.ser_camera_config = new ROSLIB.Service({
    ros: $scope.ros,
    name: "cameraconfig/atur",
    messageType: "iris_its/camera_config",
  });

  $scope.ser_camera_config.callService($scope.req_camera_config, (res) => {
    for (var i = 0; i < 3; i++)
      $scope.camera_parameter[i].value = res.camera_config[i];
  });

  ///////////////////////////////////////////////////////

  $scope.set_default_thresshold_bola = function () {
    $scope.ball_thresholdParameter[0].value = 0;
    $scope.ball_thresholdParameter[1].value = 13;
    $scope.ball_thresholdParameter[2].value = 156;
    $scope.ball_thresholdParameter[3].value = 255;
    $scope.ball_thresholdParameter[4].value = 145;
    $scope.ball_thresholdParameter[5].value = 255;
    $scope.pub_ball_threshold_value($scope.ball_thresholdParameter);
  };

  $scope.set_default_thresshold_lapangan = function () {
    $scope.field_thresholdParameter[0].value = 0;
    $scope.field_thresholdParameter[1].value = 110;
    $scope.field_thresholdParameter[2].value = 52;
    $scope.field_thresholdParameter[3].value = 255;
    $scope.field_thresholdParameter[4].value = 80;
    $scope.field_thresholdParameter[5].value = 255;
    $scope.pub_field_threshold_value($scope.field_thresholdParameter);
  };

  $scope.set_default_camera_setting = function () {
    $scope.camera_parameter[0].value = 162;
    $scope.camera_parameter[1].value = 0;
    $scope.camera_parameter[2].value = 255;
    $scope.pub_cameraConfig($scope.camera_parameter);
  };
});
