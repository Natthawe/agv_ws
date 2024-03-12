// vanila js document ready function
document.addEventListener("DOMContentLoaded", function (event) {
    console.log("DOM fully loaded and parsed");
    // your code to run since DOM is loaded and ready

    // initialize the connection to the rosbridge server
    function init() {
        const ros = new ROSLIB.Ros({
            url: 'ws://10.1.10.146:9090'
        });

        // Create the main viewer. 16:9 ratio is best for the map
        let width = window.innerWidth;
        let height = window.innerHeight;
        let viewer_stageX = 0;
        let viewer_stageY = 0;

        // rem to px  0.5
        height = height - document.getElementById('navbar').clientHeight - 28;
        console.log(width, height);
        const viewer = new ROS2D.Viewer({
            divID: 'map',
            width: width,
            height: height,
            showGrid: true,
            background: '#7f7f7f'
        });

        // Setup the map client.
        const gridClient = new ROS2D.OccupancyGridClient({
            ros: ros,
            rootObject: viewer.scene
        });

        // Scale the canvas to fit to the map
        gridClient.on('change', function () {
            viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
            viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
            viewer.scene.scale.y *= -1;

        });

        // set viewer dragable
        viewer.scene.addEventListener('stagemousedown', function (evt) {
            viewer.scene.addEventListener('stagemousemove', function (ev) {
                viewer.scene.x = viewer_stageX + ev.stageX - evt.stageX;
                viewer.scene.y = viewer_stageY + ev.stageY - evt.stageY;
                viewer.scene.update();
                viewer_stageX = ev.stageX;
                viewer_stageY = ev.stageY
            });
        });

        viewer.scene.addEventListener('stagemouseup', function (evt) {
            viewer.scene.removeAllEventListeners('stagemousemove');
        });

        // body mousewheel event
        document.body.addEventListener('mousewheel', function (evt) {
            let scale = viewer.scene.scaleX;
            scale += evt.deltaY * 0.05;
            if (scale < 0.1) {
                scale = 0.1;
            }
            viewer.scene.scaleX = scale;
            viewer.scene.scaleY = scale;
            viewer.scene.update();
        });

        // init scale size for the map
        function init_scale() {
            scale = 20;
            viewer.scene.scaleX = scale;
            viewer.scene.scaleY = scale;
            viewer.scene.update();
        }

        // setup the robot marker
        const robotMarker = new ROS2D.NavigationImage({
            size: 0.4,
            image: 'images/robot2.png',
            pulse: true,
            alpha: 0.8
        });

        // setup the odometry client
        const odomClient = new ROSLIB.Topic({
            ros: ros,
            name: '/odom',
            messageType: 'nav_msgs/Odometry'
        });

        let _pose_x = 0;
        let _pose_y = 0;
        let _pose_w = 0;

        viewer.scene.addChild(robotMarker);
        odomClient.subscribe(function (message) {
            pose_x = message.pose.pose.position.x;
            pose_y = message.pose.pose.position.y * -1;
            // Scale the canvas to fit to the map
            robotMarker.x = pose_x;
            robotMarker.y = pose_y;
            // rotate the robot marker with the orientation to robot mrker angle
            robotMarker.rotation = new THREE.Euler().setFromQuaternion(new THREE.Quaternion(
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w)
            ).z * -180 / Math.PI;
            // add laser scan
            init_scale();
            _pose_x = pose_x;
            _pose_y = pose_y;
            _pose_w = robotMarker.rotation * Math.PI / 180;
        });

        // setup goal_pose button
        const goal_pose_buttons = document.getElementsByClassName('nav-goal');
        for (let i = 0; i < goal_pose_buttons.length; i++) {
            goal_pose_buttons[i].addEventListener('click', function (evt) {
                console.log(evt.target.dataset.x, evt.target.dataset.y, evt.target.dataset.w);
                let goal_pose = new ROSLIB.Topic({
                    ros: ros,
                    name: '/goal_pose',
                    messageType: 'geometry_msgs/PoseStamped'
                });
                let pose = new ROSLIB.Message({
                    header: {
                        frame_id: 'map'
                    },
                    pose: {
                        position: {
                            x: parseFloat(evt.target.dataset.x),
                            y: parseFloat(evt.target.dataset.y),
                            z: 0
                        },
                        orientation: {
                            x: 0,
                            y: 0,
                            z: 0,
                            w: parseFloat(evt.target.dataset.w)
                        }
                    }
                });
                goal_pose.publish(pose);
            });
        }

        // setup cancelTask button
        const cancelTask_button = document.getElementById('cancelTask');
        cancelTask_button.addEventListener('click', function (evt) {
            // get robot pose and publish to /cancelTask
            let cancelTask = new ROSLIB.Topic({
                ros: ros,
                name: '/goal_pose',
                messageType: 'geometry_msgs/PoseStamped'
            });
            let pose = new ROSLIB.Message({
                header: {
                    frame_id: 'map'
                },
                pose: {
                    position: {
                        x: parseFloat(_pose_x),
                        y: parseFloat(_pose_y * -1),
                        z: 0
                    },
                    orientation: {
                        x: 0,
                        y: 0,
                        z: 0,
                        w: parseFloat(_pose_w)
                    }
                }
            });
            cancelTask.publish(pose);
        });


        // cmd_vel topic lopp for robot control
        let cmd_vel = new ROSLIB.Topic({
            ros: ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });

        let twist = new ROSLIB.Message({
            linear: {
                x: 0,
                y: 0,
                z: 0
            },
            angular: {
                x: 0,
                y: 0,
                z: 0
            }
        });

        setInterval(function () {
            cmd_vel.publish(twist);
        }, 100);

        const mobile = /Android|webOS|iPhone|iPad|iPod|BlackBerry/i.test(navigator.userAgent);
        const start = mobile ? "touchstart" : "mousedown";
        const end = mobile ? "touchend" : "mouseup";

        // setup control buttons press down and up event
        const control_buttons = document.getElementsByClassName('control-buttons');
        for (let i = 0; i < control_buttons.length; i++) {
            control_buttons[i].addEventListener('start', function (evt) {
                switch (evt.target.id) {
                    case 'forward':
                        twist.linear.x = 0.2;
                        break;
                    case 'backward':
                        twist.linear.x = -0.2;
                        break;
                    case 'left':
                        twist.angular.z = 0.2;
                        break;
                    case 'right':
                        twist.angular.z = -0.2;
                        break;
                }
            });
            control_buttons[i].addEventListener('end', function (evt) {
                twist = new ROSLIB.Message({
                    linear: {
                        x: 0,
                        y: 0,
                        z: 0
                    },
                    angular: {
                        x: 0,
                        y: 0,
                        z: 0
                    }
                });
            });
        }

        // end ros function
    }
    // battery-level percentage timerinterval change style high
    let change_battery_level_ = 100;
    timer = setInterval(function () {
        let battery_level = document.getElementById('battery-level');
        battery_level.style.height = change_battery_level_ + '%';
        // console.log(battery_level.style.height);
        change_battery_level_ -= 0.001;
        if (change_battery_level_ <= 20 && change_battery_level_ > 10) {
            battery_level.classList.add('warn');
        }else if(change_battery_level_ <= 10){
            battery_level.classList.remove('warn');
            battery_level.classList.add('alert');
        }
    }, 1000);
    
    
    // check if the document is ready
    if (document.readyState === "complete" || (document.readyState !== "loading" && !document.documentElement.doScroll)) {
        init();
    }
    else {
        document.addEventListener("DOMContentLoaded", init);
    }
});
