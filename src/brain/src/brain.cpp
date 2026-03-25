#include <iostream>
#include <string>
#include <fstream>  // 添加这一行
#include <yaml-cpp/yaml.h>  // 添加这一行
#include <cmath>

#include "brain.h"
#include "utils/print.h"
#include "utils/math.h"
#include "utils/misc.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using std::placeholders::_1;

#define SUB_STATE_QUEUE_SIZE 1

Brain::Brain() : rclcpp::Node("brain_node")
{
    // 初始化tf广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    
    // 要注意参数必须先在这里声明，否则程序里也读不到
    // 配置在 yaml 文件中的参数，如果有层级结构，用点分号来获取

    declare_parameter<int>("game.team_id", 0);
    declare_parameter<int>("game.player_id", 29);
    declare_parameter<string>("game.field_type", "");

    declare_parameter<string>("game.player_role", "");
    declare_parameter<string>("game.player_start_pos", "left");
    declare_parameter<bool>("game.treat_person_as_robot", false);
    declare_parameter<int>("game.number_of_players", 2);

    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.vx_factor", 0.5);
    declare_parameter<double>("robot.yaw_offset", 0.0);
    declare_parameter<double>("robot.vx_limit", 1.0);
    declare_parameter<double>("robot.vy_limit", 0.4);
    declare_parameter<double>("robot.vtheta_limit", 1.0);

    declare_parameter<double>("strategy.ball_confidence_threshold", 50.0);   
    declare_parameter<double>("strategy.ball_memory_timeout", 3.0);
    declare_parameter<double>("strategy.tm_ball_dist_threshold", 3.0);
    declare_parameter<bool>("strategy.limit_near_ball_speed", true);
    declare_parameter<double>("strategy.near_ball_speed_limit", 0.3);
    declare_parameter<double>("strategy.near_ball_range", 4.0);
    declare_parameter<bool>("strategy.abort_kick_when_ball_moved", false);
    declare_parameter<bool>("strategy.enable_bypass", false);
    declare_parameter<bool>("strategy.enable_shoot", false);
    declare_parameter<bool>("strategy.enable_directional_kick", false);

    declare_parameter<bool>("strategy.use_squat_block", false);
    declare_parameter<double>("strategy.squat_block_msecs", 2000.0);
    declare_parameter<bool>("strategy.use_move_block", true);
    declare_parameter<double>("strategy.move_block_msecs", 2000.0);
    declare_parameter<bool>("strategy.enable_auto_visual_kick", false);
    declare_parameter<bool>("strategy.enable_auto_visual_defend", false);

    declare_parameter<bool>("strategy.power_shoot.enable", false);
    declare_parameter<bool>("strategy.power_shoot.use_for_kickoff", false);
    declare_parameter<double>("strategy.power_shoot.xmin", 0.5);
    declare_parameter<double>("strategy.power_shoot.xmax", 1.0);
    declare_parameter<double>("strategy.power_shoot.ymin", -0.5);
    declare_parameter<double>("strategy.power_shoot.ymax", 0.5);
    declare_parameter<double>("strategy.shoot.threat_threshold", 0.0);
    declare_parameter<double>("strategy.shoot.xmin", 0.5);
    declare_parameter<double>("strategy.shoot.xmax", 1.0);
    declare_parameter<double>("strategy.shoot.ymin", -0.5);
    declare_parameter<double>("strategy.shoot.ymax", 0.5);
    declare_parameter<bool>("strategy.cooperation.enable_role_switch", true);
    declare_parameter<double>("strategy.cooperation.ball_control_cost_threshold", 10.0);

    declare_parameter<int>("obstacle_avoidance.depth_sample_step", 16);
    declare_parameter<double>("obstacle_avoidance.obstacle_min_height", 0.15);
    declare_parameter<double>("obstacle_avoidance.grid_size", 0.2);
    declare_parameter<double>("obstacle_avoidance.max_x", 0.2);
    declare_parameter<double>("obstacle_avoidance.max_y", 0.2);
    declare_parameter<double>("obstacle_avoidance.exclusion_x", 0.25);
    declare_parameter<double>("obstacle_avoidance.exclusion_y", 0.4);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_radius", 0.3);
    declare_parameter<double>("obstacle_avoidance.ball_exclusion_height", 0.3);
    declare_parameter<int>("obstacle_avoidance.occupancy_threshold", 5);
    declare_parameter<double>("obstacle_avoidance.collision_threshold", 0.5);
    declare_parameter<double>("obstacle_avoidance.safe_distance", 2.0);
    declare_parameter<double>("obstacle_avoidance.avoid_secs", 3.0);
    declare_parameter<bool>("obstacle_avoidance.enable_freekick_avoid", false);
    declare_parameter<double>("obstacle_avoidance.freekick_start_placing_safe_distance", 0.5);
    declare_parameter<double>("obstacle_avoidance.freekick_start_placing_avoid_secs", 1.5);
    declare_parameter<double>("obstacle_avoidance.obstacle_memory_msecs", 500.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_chase", false);
    declare_parameter<double>("obstacle_avoidance.chase_ao_safe_dist", 2.0);
    declare_parameter<bool>("obstacle_avoidance.avoid_during_kick", false);
    declare_parameter<double>("obstacle_avoidance.kick_ao_safe_dist", 1.0);
    declare_parameter<bool>("obstacle_avoidance.kick_ao_use_shoot", false);
    
    declare_parameter<int>("locator.min_marker_count", 5);
    declare_parameter<double>("locator.max_residual", 0.3);

    declare_parameter<bool>("enable_com", false);

    declare_parameter<bool>("rerunLog.enable_tcp", false);
    declare_parameter<string>("rerunLog.server_ip", "");
    declare_parameter<bool>("rerunLog.enable_file", false);
    declare_parameter<string>("rerunLog.log_dir", "");
    declare_parameter<double>("rerunLog.max_log_file_mins", 5.0);
    declare_parameter<int>("rerunLog.img_interval", 10);

    declare_parameter<bool>("sound.enable", false);
    declare_parameter<string>("sound.sound_pack", "espeak");

    declare_parameter<string>("vision.image_topic", "/camera/camera/color/image_raw");
    declare_parameter<string>("vision.depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    declare_parameter<double>("vision.cam_pixel_width", 1280);
    declare_parameter<double>("vision.cam_pixel_height", 720);
    declare_parameter<double>("vision.cam_fov_x", 90);
    declare_parameter<double>("vision.cam_fov_y", 65);

    declare_parameter<string>("game_control_ip", "0.0.0.0");

    declare_parameter<string>("tree_file_path", "");
    declare_parameter<string>("vision_config_path", "");
    declare_parameter<string>("vision_config_local_path", "");

    declare_parameter<int>("recovery.retry_max_count", 2);
}

Brain::~Brain()
{

}

void Brain::init()
{
    
    config = std::make_shared<BrainConfig>();
    loadConfig();

    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();
    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);
    communication = std::make_shared<BrainCommunication>(this);

    
    locator->init(config->fieldDimensions, config->pfMinMarkerCnt, config->pfMaxResidual);

   
    tree->init();

   
    client->init();

    log->prepare();
    

    
    communication->initCommunication();

    data->lastSuccessfulLocalizeTime = get_clock()->now();
    data->timeLastDet = get_clock()->now();
    data->timeLastLineDet = get_clock()->now();
    data->timeLastGamecontrolMsg = get_clock()->now();
    data->ball.timePoint = get_clock()->now();

    
    auto now = get_clock()->now();
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        data->tmStatus[i].isAlive = false;
        data->tmStatus[i].timeLastCom = now;
    }
    data->tmLastCmdChangeTime = now;

   
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", SUB_STATE_QUEUE_SIZE, bind(&Brain::detectionsCallback, this, _1));
    subFieldLine = create_subscription<vision_interface::msg::LineSegments>("/booster_vision/line_segments", SUB_STATE_QUEUE_SIZE, bind(&Brain::fieldLineCallback, this, _1));
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state", SUB_STATE_QUEUE_SIZE, bind(&Brain::odometerCallback, this, _1));
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state", SUB_STATE_QUEUE_SIZE, bind(&Brain::lowStateCallback, this, _1));
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose", SUB_STATE_QUEUE_SIZE, bind(&Brain::headPoseCallback, this, _1));
    recoveryStateSubscription = create_subscription<booster_interface::msg::RawBytesMsg>("fall_down_recovery_state", SUB_STATE_QUEUE_SIZE, bind(&Brain::recoveryStateCallback, this, _1));

    if (config->rerunLogEnableFile || config->rerunLogEnableTCP) {
        string imageTopic = get_parameter("vision.image_topic").as_string();
        imageSubscription = create_subscription<sensor_msgs::msg::Image>(imageTopic, SUB_STATE_QUEUE_SIZE, bind(&Brain::imageCallback, this, _1));
    }
    string depthTopic = get_parameter("vision.depth_image_topic").as_string();
    depthImageSubscription = create_subscription<sensor_msgs::msg::Image>(depthTopic, SUB_STATE_QUEUE_SIZE, bind(&Brain::depthImageCallback, this, _1));
    
    pubSoundPlay = create_publisher<std_msgs::msg::String>("/play_sound", 10);
    pubSpeak = create_publisher<std_msgs::msg::String>("/speak", 10);
}

void Brain::loadConfig()
{
    get_parameter("game.team_id", config->teamId);
    get_parameter("game.player_id", config->playerId);
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("game.player_start_pos", config->playerStartPos);
    get_parameter("game.treat_person_as_robot", config->treatPersonAsRobot);
    get_parameter("game.number_of_players", config->numOfPlayers);

    get_parameter("robot.robot_height", config->robotHeight);
    get_parameter("robot.odom_factor", config->robotOdomFactor);
    get_parameter("robot.vx_factor", config->vxFactor);
    get_parameter("robot.yaw_offset", config->yawOffset);
    get_parameter("robot.vx_limit", config->vxLimit);
    get_parameter("robot.vy_limit", config->vyLimit);
    get_parameter("robot.vtheta_limit", config->vthetaLimit);

    get_parameter("strategy.ball_confidence_threshold", config->ballConfidenceThreshold);
    get_parameter("strategy.tm_ball_dist_threshold", config->tmBallDistThreshold);
    get_parameter("strategy.limit_near_ball_speed", config->limitNearBallSpeed);
    get_parameter("strategy.near_ball_speed_limit", config->nearBallSpeedLimit);
    get_parameter("strategy.near_ball_range", config->nearBallRange);

    get_parameter("obstacle_avoidance.collision_threshold", config->collisionThreshold);
    get_parameter("obstacle_avoidance.safe_distance", config->safeDistance);
    get_parameter("obstacle_avoidance.avoid_secs", config->avoidSecs);

    get_parameter("locator.min_marker_count", config->pfMinMarkerCnt);
    get_parameter("locator.max_residual", config->pfMaxResidual);

    get_parameter("enable_com", config->enableCom);

    // get_parameter("rerunLog.enable", config->rerunLogEnable);
    get_parameter("rerunLog.enable_tcp", config->rerunLogEnableTCP);
    get_parameter("rerunLog.server_ip", config->rerunLogServerIP);
    get_parameter("rerunLog.enable_file", config->rerunLogEnableFile);
    get_parameter("rerunLog.log_dir", config->rerunLogLogDir);
    get_parameter("rerunLog.max_log_file_mins", config->rerunLogMaxFileMins);
    get_parameter("rerunLog.img_interval", config->rerunLogImgInterval);

    get_parameter("sound.enable", config->soundEnable);
    get_parameter("sound.sound_pack", config->soundPack);

    get_parameter("tree_file_path", config->treeFilePath);

    get_parameter("vision.cam_pixel_width", config->camPixX);
    get_parameter("vision.cam_pixel_height", config->camPixY);
    double camDegX, camDegY;
    get_parameter("vision.cam_fov_x", camDegX);
    get_parameter("vision.cam_fov_y", camDegY);
    config->camAngleX = deg2rad(camDegX);
    config->camAngleY = deg2rad(camDegY);

    // 从视觉 config 中加载相关参数
    string visionConfigPath, visionConfigLocalPath;
    get_parameter("vision_config_path", visionConfigPath);
    get_parameter("vision_config_local_path", visionConfigLocalPath);
    if (!filesystem::exists(visionConfigPath)) {
        // 报错然后退出
        RCLCPP_ERROR(get_logger(), "vision_config_path %s not exists", visionConfigPath.c_str());
        exit(1);
    }
    // else
    YAML::Node vConfig = YAML::LoadFile(visionConfigPath);
    if (filesystem::exists(visionConfigLocalPath)) {
        YAML::Node vConfigLocal = YAML::LoadFile(visionConfigLocalPath);
        MergeYAML(vConfig, vConfigLocal);
    }
    config->camfx = vConfig["camera"]["intrin"]["fx"].as<double>();
    config->camfy = vConfig["camera"]["intrin"]["fy"].as<double>();
    config->camcx = vConfig["camera"]["intrin"]["cx"].as<double>();
    config->camcy = vConfig["camera"]["intrin"]["cy"].as<double>();

    auto extrin = vConfig["camera"]["extrin"];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            config->camToHead(i, j) = extrin[i][j].as<double>();
        }
    }
    prtDebug(format("camfx: %f, camfy: %f, camcx: %f, camcy: %f", config->camfx, config->camfy, config->camcx, config->camcy));
    string str_cam2head = "camToHead: \n";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            str_cam2head += format("%.3f ", config->camToHead(i, j));
        }
        str_cam2head += "\n";
    }
    prtDebug(str_cam2head);


    config->handle();


    ostringstream oss;
    config->print(oss);
    prtDebug(oss.str());
}


void Brain::tick()
{
    // 输出 debug & log 相关信息
    logDebugInfo();
    // logObstacleDistance(); // 计算量大, 仅需要时使用
    logLags();
    statusReport();
    logStatusToConsole();
    playSoundForFun();
    updateLogFile();
    
    updateMemory();
    handleSpecialStates();
    handleCooperation();

    tree->tick();
}

void Brain::handleSpecialStates() {

    const double KICKOFF_DURATION = 10.0; 
    string gameState = tree->getEntry<string>("gc_game_state");
    bool isKickoffSide = tree->getEntry<bool>("gc_is_kickoff_side");
    string gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    bool isFreekickKickoffSide = tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    auto now = get_clock()->now();

    if (gameState == "SET" && isKickoffSide) {
        data->isKickingOff = true;
        data->kickoffStartTime = now;
    } else if (msecsSince(data->kickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isKickingOff = false;
    }

    if (gameState == "PLAY" && gameSubStateType == "FREE_KICK" && isFreekickKickoffSide) {
        data->isFreekickKickingOff = true;
        data->freekickKickoffStartTime = now;
    } else if (msecsSince(data->freekickKickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isFreekickKickingOff = false;
        data->isDirectShoot = false;
    }

    static int lastScore = 0;
    if (data->score > lastScore) {
        tree->setEntry<bool>("we_just_scored", true);
        lastScore = data->score;
    }
    if (gameState == "SET") {
        tree->setEntry<bool>("we_just_scored", false);
    }
}

void Brain::handleCooperation() {
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/handleCooperation", rerun::TextLog(msg));
    };
    log_("handle cooperation");

    const int CMD_COOLDOWN = 2000; 
    const int COM_TIMEOUT = 5000; 

    int selfId = config->playerId;
    int selfIdx = selfId - 1;
    int numOfPlayers = config->numOfPlayers;

    vector<int> aliveTmIdxs = {}; 


    data->tmImAlive = 
        (data->penalty[selfIdx] == PENALTY_NONE) 
        && tree->getEntry<bool>("odom_calibrated"); 
    updateCostToKick(); 
    log_(format("ImAlive: %d, myCost: %.1f", data->tmImAlive, data->tmMyCost));

    
    int gcAliveCount = 0; 
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++)
    {
        if (data->penalty[i] == PENALTY_NONE) {
            gcAliveCount++;

            int tmId = i + 1;
            if (tmId == config->playerId) continue;

            auto tmStatus = data->tmStatus[i];
            log->setTimeNow();
            auto color = 0x00FFFFFF;
            if (!tmStatus.isAlive) color = 0x006666FF;
            else if (!tmStatus.isLead) color = 0x00CCCCFF;
            string label = format("ID: %d, Cost: %.1f", tmId, tmStatus.cost);
            log->logRobot(format("field/teammate-%d", tmId).c_str(), tmStatus.robotPoseToField, color, label);
            log->logBall(
            format("tm_ball-%d", tmId).c_str(),
            tmStatus.ballPosToField, 
            tmStatus.ballDetected ? 0x00FFFFFF : (tmStatus.isAlive ? 0x006666FF : 0x003333FF),
            tmStatus.ballConfidence,
            tmStatus.ballLocationKnown
            );
        }
    }
    log_(format("gcAliveCnt: %d", gcAliveCount));

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue; 

        if (
            data->penalty[i] != PENALTY_NONE 
            || msecsSince(data->tmStatus[i].timeLastCom) > COM_TIMEOUT 
        ) {
            data->tmStatus[i].isAlive = false;
            data->tmStatus[i].isLead = false;
        }
        
        if (data->tmStatus[i].isAlive) {
            aliveTmIdxs.push_back(i);
            log->setTimeNow();
            log->log(format("debug/tm_cost_scalar_%d", i + 1), rerun::Scalar(data->tmStatus[i].cost));
            log->log(format("debug/tm_lead_scalar_%d", i + 1), rerun::Scalar(data->tmStatus[i].isLead));
        }
    }
    log_(format("alive TM Count: %d", aliveTmIdxs.size()));

    // log 当前 alive 队友的信息
    log_(format("Self: cost: %.1f, isLead: %d", data->tmMyCost, data->tmImLead));


    static rclcpp::Time lastTmBallPosTime = get_clock()->now();
    const double TM_BALL_TIMEOUT = 1000.; 
    const double RANGE_THRESHOLD = config->tmBallDistThreshold; 
    int trustedTMIdx = -1;
    double minRange = 1e6;
    log_(format("Find ball info among %d alive TMs", aliveTmIdxs.size()));
    for (int i = 0; i < aliveTmIdxs.size(); i++) {
        auto status = data->tmStatus[aliveTmIdxs[i]];
        log_(format("TM %d, ballDetected: %d, ballRange: %.1f", i + 1, status.ballDetected, status.ballRange));
        if (status.ballDetected && status.ballRange < minRange) {
            log_(format("tm ball range(%.1f) < minRange(%.1f)", status.ballRange, minRange));
            double dist = norm(status.ballPosToField.x - data->robotPoseToField.x, status.ballPosToField.y - data->robotPoseToField.y);
            if (dist > RANGE_THRESHOLD) {
                log_(format("tm ball dist to me(%.1f) > threshold(%.1f), TM %d can be trusted", dist, RANGE_THRESHOLD, i+ 1));
                minRange = status.ballRange;
                trustedTMIdx = aliveTmIdxs[i];
            }  else {
                log_(format("tm ball dist to me(%.1f) < threshold(%.1f), TM %d can NOT be trusted", dist, RANGE_THRESHOLD, i+ 1));
            }
        }
    }
    if (trustedTMIdx >= 0) {   
        log_(format("Reliable tm ball found. PlayerID = %d", trustedTMIdx + 1));
        data->tmBall.posToField = data->tmStatus[trustedTMIdx].ballPosToField;
        updateRelativePos(data->tmBall);

        tree->setEntry<bool>("tm_ball_pos_reliable", true);
        lastTmBallPosTime = get_clock()->now();
        if (!tree->getEntry<bool>("ball_location_known")) { 
            log_("update ball.posToField");
            data->ball.posToField = data->tmBall.posToField;
            updateRelativePos(data->ball);
        }
    } else {
        log_("TM reported NO BALL or can not be trusted");
        if (msecsSince(lastTmBallPosTime) > TM_BALL_TIMEOUT) {
            log_("TM ball timeout reached");
            tree->setEntry<bool>("tm_ball_pos_reliable", false);
        }
    }

    bool switchRole;
    get_parameter("strategy.cooperation.enable_role_switch", switchRole);
    if (switchRole) {
        if (data->penalty[selfIdx] == PENALTY_NONE) { 
            if (gcAliveCount < numOfPlayers) { 
                log_("Not full team. I must be Striker");
                tree->setEntry<string>("player_role", "striker"); 
            }
        } else { 
            if (gcAliveCount == numOfPlayers - 1) { 
                log_("I am only on under penalty, I must be goal keeper");
                tree->setEntry<string>("player_role", "goal_keeper"); 
            }
    
        }
    }

    if (tree->getEntry<string>("gc_game_state") == "INITIAL") {
        tree->setEntry<string>("player_role", config->playerRole);
    }


    double tmMinCost = 1e5;
    for (int i = 0; i < aliveTmIdxs.size(); i++) {
        int tmIdx = aliveTmIdxs[i];
        auto tmStatus = data->tmStatus[tmIdx];
        if (tmStatus.cost < tmMinCost) tmMinCost = tmStatus.cost;
    }
    double BALL_CONTROL_COST_THRESHOLD = 3.0;
    get_parameter("strategy.cooperation.ball_control_cost_threshold", BALL_CONTROL_COST_THRESHOLD);

    if (tmMinCost < BALL_CONTROL_COST_THRESHOLD && data->tmMyCost > tmMinCost) {

        data->tmImLead = false;
        tree->setEntry<bool>("is_lead", false);
        log_("I am not lead");

    } else {
        data->tmImLead = true;
        tree->setEntry<bool>("is_lead", true);
        log_("I am Lead");
    }
    log_(format("tmMinCost: %.1f, myCost: %.1f", tmMinCost, data->tmMyCost));


    if (
        data->tmImAlive 
        && tree->getEntry<string>("player_role") == "goal_keeper"
        && data->tmImLead 
        && msecsSince(data->tmLastCmdChangeTime) > CMD_COOLDOWN 
    ) {
        auto distToGoal = [=](Pose2D pose) {
            return norm(pose.x + config->fieldDimensions.length / 2.0, pose.y);
        };
        double maxDist = 0.0;
        double minDist = 1e6;
        int minIndex = -1; 
        double myDist = distToGoal(data->robotPoseToField);
        for (int i = 0; i < aliveTmIdxs.size(); i++) {
            int tmIdx = aliveTmIdxs[i];
            auto tmPose = data->tmStatus[tmIdx].robotPoseToField;
            double dist = distToGoal(tmPose);
            if (dist > maxDist) maxDist = dist;
            if (dist < minDist) {
                minIndex = tmIdx;
                minDist = dist;
            }
        }
        if (minIndex >= 0 && myDist > maxDist) {
            data->tmLastCmdChangeTime = get_clock()->now();
            data->tmMyCmd = 10 + minIndex + 1; 
            data->tmCmdId += 1;
            data->tmMyCmdId = data->tmCmdId;
            tree->setEntry<string>("player_role", "striker");
            log_(format("goalie: i am too far from goal, i ask player %d to attack", minIndex + 1));
        } else {
            log_(format("goalie: i am close enough to goal, no need to attack, my dist: %.2f", myDist));
        }
    }


    auto cmd = data->tmReceivedCmd;
    if (cmd != 0) {
        log_(format("received cmd %d from teammate", cmd));
        if (cmd > 10 && cmd < 20) { 
            log_("goalie wants to attack");
            int newGoalieId = cmd - 10;
            if (newGoalieId == selfId) { 
                log_("i become goalie");
                tree->setEntry<string>("player_role", "goal_keeper");
                speak("i become goalie", true);
            } else { 
                log_(format("teammate %d becomes goalie", newGoalieId));
            }
        } else {
            log_(format("unknown cmd %d from teammate", cmd));
        }

        data->tmReceivedCmd = 0; 
    }

    tree->setEntry<bool>("is_lead", data->tmImLead);

    if (
        (tree->getEntry<string>("gc_game_state") == "READY" || tree->getEntry<string>("gc_game_sub_state") == "GET_READY") 
        && gcAliveCount == numOfPlayers
    ) {
       
        tree->setEntry<string>("player_role", config->playerRole);
        log_(format("all teammates on field. Back to initial role: %s", config->playerRole.c_str()));
    }

    return;
}

void Brain::updateMemory()
{
    updateBallMemory();
    updateRobotMemory();
    updateObstacleMemory();
    updateKickoffMemory();
}

void Brain::updateObstacleMemory() {
   
    auto obstacles = data->getObstacles();
    vector<GameObject> obs_new = {};

    const double OBS_EXPIRE_TIME = get_parameter("obstacle_avoidance.obstacle_memory_msecs").as_double();
    for (int i = 0; i < obstacles.size(); i++) {
        auto obs = obstacles[i];
        if (obs.label == "Ball") continue; 
        if (msecsSince(obs.timePoint) > OBS_EXPIRE_TIME)  continue; 


        updateRelativePos(obs);
        obs_new.push_back(obs);
    }


    if (
        (get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() && isFreekickStartPlacing())
        || tree->getEntry<string>("gc_game_state") == "READY"
    ) {
        obs_new.push_back(data->ball);
    }

    data->setObstacles(obs_new);
    logObstacles();
}

void Brain::updateBallMemory()
{

    double secs = msecsSince(data->ball.timePoint) / 1000;
    
    double ballMemTimeout;
    get_parameter("strategy.ball_memory_timeout", ballMemTimeout);

    if (secs > ballMemTimeout) 
    { 
        tree->setEntry<bool>("ball_location_known", false);
        tree->setEntry<bool>("ball_out", false); 
    }

    
    updateRelativePos(data->ball);
    updateRelativePos(data->tmBall);
    tree->setEntry<double>("ball_range", data->ball.range);



    log->setTimeNow();
    log->logBall(
        "field/ball", 
        data->ball.posToField, 
        data->ballDetected ? 0x00FF00FF : 0x006600FF,
        data->ballDetected,
        tree->getEntry<bool>("ball_location_known")
        );
    log->logBall(
        "field/tmBall", 
        data->tmBall.posToField, 
        0xFFFF00FF,
        tree->getEntry<bool>("tm_ball_pos_reliable"),
        tree->getEntry<bool>("tm_ball_pos_reliable")
        );
}

void Brain::updateRobotMemory() {
    auto robots = data->getRobots();
    vector<GameObject> newRobots = {};

    for (int i = 0; i < robots.size(); i++) {
        auto r = robots[i];


        if (msecsSince(r.timePoint) > 1000)  continue;


        updateRelativePos(r);
        newRobots.push_back(r);
    }

    data->setRobots(newRobots);

    logMemRobots();
}

void Brain::updateKickoffMemory() {
    
    static Point ballPos;
    const double BALL_MOVE_THRESHOLD_FACTOR = 0.15; 
    const double BALL_MOVE_THRESHOLD_MIN = 0.3; 
    auto ballMoved = [=]() {
        if (!data->ballDetected) return false; 
        double range = data->ball.range;
        double threshold = max(range * BALL_MOVE_THRESHOLD_FACTOR, BALL_MOVE_THRESHOLD_MIN);
        double posChange = norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y);
        return posChange > threshold;
    };
    static rclcpp::Time kickOffTime;
    const double TIMEOUT = 1000 * 10; 
    auto timeReached = [=]() {
        return msecsSince(kickOffTime) > TIMEOUT;
    };
    bool isWaitingForKickoff = (
        (tree->getEntry<string>("gc_game_state") == "SET"  || tree->getEntry<string>("gc_game_state") == "READY")
        && !tree->getEntry<bool>("gc_is_kickoff_side")
    );
    bool isWaitingForFreekickKickoff = (
        (tree->getEntry<string>("gc_game_sub_state") == "SET" || tree->getEntry<string>("gc_game_sub_state") == "GET_READY")
        && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side")
    );
    if ( isWaitingForFreekickKickoff || isWaitingForKickoff) {
        ballPos = data->ball.posToRobot;
        kickOffTime = get_clock()->now();
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
    } else if (tree->getEntry<bool>("wait_for_opponent_kickoff")) {
        if (ballMoved() || timeReached()) {
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        }
    }
}

vector<double> Brain::getGoalPostAngles(const double margin)
{
    double leftX, leftY, rightX, rightY; 

    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;


    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++)
    {
        auto post = goalposts[i];
        if (post.name == "OL")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.name == "OR")
        {
            rightX = post.posToField.x;
            rightY = post.posToField.y;
        }
    }

    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);

    vector<double> vec = {theta_l, theta_r};
    return vec;
}

double Brain::calcKickDir(double goalPostMargin) {
    double dir_rb_f = data->robotBallAngleToField; 
    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1];
    
    if (isAngleGood(goalPostMargin)) return dir_rb_f;

    double delta_l = fabs(toPInPI(theta_l - dir_rb_f));
    double delta_r = fabs(toPInPI(theta_r - dir_rb_f));
    if (delta_l < delta_r) return theta_l;
    // else 
    return theta_r;
}

void Brain::updateCostToKick() {
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/updateCostToKick", rerun::TextLog(msg));
    };
    double cost = 0.;


    // if (!data->ballDetected) cost += 2.0;
    double secsSinceBallDet = msecsSince(data->ball.timePoint) / 1000;
    cost += secsSinceBallDet;
    log_(format("ball not dectect cost: %.1f", secsSinceBallDet));


    if (!tree->getEntry<bool>("ball_location_known")) {
        cost += 5.0;
        log_(format("ball lost cost: %.1f", 5.0));
    }


    cost += data->ball.range;
    log_(format("ball range cost: %.1f", data->ball.range));
    
    

    if (distToObstacle(data->ball.yawToRobot) < 1.5) {
        log_(format("obstacle cost: %.1f", 2.0));
        cost += 2.0;
    }


    cost += fabs(data->ball.yawToRobot) / 1.0; 
    log_(format("ball yaw cost: %.1f", fabs(data->ball.yawToRobot) / 1.0));



    int selfIdx = config->playerId - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue; 

        auto status = data->tmStatus[i]; 
        if (!status.isAlive) continue; 

        double theta_tm2ball = atan2(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
        double range_tm2ball = norm(status.ballPosToField.y - status.robotPoseToField.y, status.ballPosToField.x - status.robotPoseToField.x);
        double theta_me2ball = data->robotBallAngleToField;
        double range_me2ball = data->ball.range;
        double deltaTheta = fabs(toPInPI(theta_tm2ball - theta_me2ball));

        const double BUMP_DIST = 1.0;
        if (range_tm2ball < range_me2ball && sin(deltaTheta) * range_tm2ball < BUMP_DIST) {
            cost += 2.0;
            log_(format("bump cost: %.1f", 2.0));  
        }
    }

    cost += fabs(toPInPI(data->kickDir - data->robotBallAngleToField)) * 0.4 / 0.3; 
    log_(format("ajust cost: %.1f", fabs(toPInPI(data->kickDir - data->robotBallAngleToField)) * 0.4 / 0.3));
    

    if (data->recoveryState == RobotRecoveryState::HAS_FALLEN) {
        cost += 15.0;
        log_(format("fall cost: %.1f", 15.0));  
    }

    
    if (!tree->getEntry<bool>("odom_calibrated")) {
        cost += 100;
        log_(format("localization cost: %.1f", 100.0));  

    }
    
    double lastCost = data->tmMyCost;
    data->tmMyCost = lastCost * 0.5 + cost * 0.5;
    log_(format("lastCost: %.1f, newCost: %.1f, smoothCost: %.1f", lastCost, cost, data->tmMyCost));

    return;
}

bool Brain::isAngleGood(double goalPostMargin, string type) {
    double angle = 0;
    if (type == "kick") angle = data->robotBallAngleToField; // type=="kick" 机器人到球, field 坐标系中的方向
    if (type == "shoot") angle = data->robotPoseToField.theta; // type=="shoot" 机器人朝向
    

    auto goalPostAngles = getGoalPostAngles(goalPostMargin);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1]; 
    
    if (theta_l - theta_r < M_PI / 3 * 2) { 
        goalPostAngles = getGoalPostAngles(0.5);
        theta_l = goalPostAngles[0]; 
        theta_r = goalPostAngles[1]; 
    }

    return (theta_l > angle && theta_r < angle);
}

bool Brain::isPrimaryStriker() {
    string myRole = tree->getEntry<string>("player_role");
    if (myRole != "striker") return false; 

    if (!config->enableCom) return true; 

    // find first alive striker that is not me.
    auto firstAliveStrikerIdx = -1;
    auto myIdx = config->playerId - 1;

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        auto status = data->tmStatus[i];
        if (data->penalty[i] == PENALTY_NONE && status.isAlive && status.role == "striker") {
            firstAliveStrikerIdx = i;
            break;
        }
    }

    if ( firstAliveStrikerIdx >= 0 && firstAliveStrikerIdx <  myIdx) return false; // 有 id 更小的活的前锋, 让他来当主力

    // else 没有 id 更小的活的前锋, 我是主力
    return true;
}

bool Brain::isBallOut(double locCompareDist, double lineCompareDist)
{
    auto ball = data->ball;
    auto fd = config->fieldDimensions;

    if (fabs(ball.posToField.x) > fd.length / 2 + locCompareDist)
        return true;
    if (fabs(ball.posToField.y) > fd.width / 2 + locCompareDist)
        return true;
    
    auto fieldLines = data->getFieldLines();
    for (int i = 0; i < fieldLines.size(); i++) {
        auto line = fieldLines[i];
        if (
            (line.type == LineType::TouchLine || line.type == LineType::GoalLine)
            && line.confidence > 1.0
         ) {
            Point2D p = {ball.posToField.x, ball.posToField.y};
            // prtWarn(format("Ball: %.2f, %.2f PerpDist: %.2f", ball.posToField.x, ball.posToField.y, pointPerpDistToLine(p, line.posToField)));
            if (pointPerpDistToLine(p, line.posToField) > lineCompareDist) return true;
        }
    }
    return false;
}

void Brain::updateBallOut() {
    bool lastBallOut = tree->getEntry<bool>("ball_out");
    double range = lastBallOut ? 4.0 : 2.5;
    double threshold = config->ballOutThreshold;
    threshold += (data->isFreekickKickingOff ? 1.0 : 0.0); // 如果正在踢任意球, 则放宽出界判断
    threshold *= (lastBallOut ? 1.0 : 1.5); // 防止震荡. 如果上次判断为出界, 则放宽出界判断
    tree->setEntry<bool>("ball_out", isBallOut(threshold, 10.0) && data->ball.range < range); // 严格通过定位判断是否出界
}

double Brain::distToBorder() {
    vector<Line> borders;
    auto fd = config->fieldDimensions;
    borders.push_back({fd.length / 2, fd.width / 2, -fd.length / 2, fd.width / 2});
    borders.push_back({fd.length / 2, -fd.width / 2, -fd.length / 2, -fd.width / 2});
    borders.push_back({fd.length / 2, fd.width / 2, fd.length / 2, -fd.width / 2});
    borders.push_back({-fd.length / 2, fd.width / 2, -fd.length / 2, -fd.width / 2});
    double maxDist = -100;
    Point2D robot = {data->robotPoseToField.x, data->robotPoseToField.y};
    for (int i = 0; i < borders.size(); i++) {
        auto line = borders[i];
        double dist = pointPerpDistToLine(robot, line);
        if (dist > maxDist) maxDist = dist;
    }
    return maxDist;
}

bool Brain::isBoundingBoxInCenter(BoundingBox boundingBox, double xRatio, double yRatio) {
    double x = (boundingBox.xmin + boundingBox.xmax) / 2.0;
    double y = (boundingBox.ymin + boundingBox.ymax) / 2.0;

    return (x  > config->camPixX * (1 - xRatio) / 2)
        && (x < config->camPixX * (1 + xRatio) / 2)
        && (y > config->camPixY * (1 - yRatio) / 2)
        && (y < config->camPixY * (1 + yRatio) / 2);
}

bool Brain::isDefensing() {
    bool isFreeKick = tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK";
    bool isKickoffSide = tree->getEntry<bool>("gc_is_sub_state_kickoff_side");
    
    return isFreeKick && (!isKickoffSide);
}

void Brain::calibrateOdom(double x, double y, double theta)
{

    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    y_or = sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    theta_or = -data->robotPoseToOdom.theta;

    
    transCoord(x_or, y_or, theta_or,
               x, y, theta,
               data->odomToField.x, data->odomToField.y, data->odomToField.theta);


    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);


    double placeHolder;
    // ball
    transCoord(
        data->ball.posToRobot.x, data->ball.posToRobot.y, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        data->ball.posToField.x, data->ball.posToField.y, placeHolder 
    );

    // robots
    auto robots = data->getRobots();
    for (int i = 0; i < robots.size(); i++) {
        updateFieldPos(robots[i]);
    }
    data->setRobots(robots);

    // goalposts
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        updateFieldPos(goalposts[i]);
    }
    
    // markers
    auto markings = data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        updateFieldPos(markings[i]);
    }

    // relog
    log->setTimeNow();
    // logVisionBox(get_clock()->now());
    vector<GameObject> gameObjects = {};
    if(data->ballDetected) gameObjects.push_back(data->ball);
    for (int i = 0; i < markings.size(); i++) gameObjects.push_back(markings[i]);
    for (int i = 0; i < robots.size(); i++) gameObjects.push_back(robots[i]);
    for (int i = 0; i < goalposts.size(); i++) gameObjects.push_back(goalposts[i]);
    logDetection(gameObjects);
}

void Brain::playSound(string soundName, double blockMsecs, bool allowRepeat)
{
    if (!pubSoundPlay) return;

    static string _lastSound;
    static rclcpp::Time _lastTime;
    static double _lastBlockMsecs = 0;

    auto now = get_clock()->now();
    if (msecsSince(_lastTime) < _lastBlockMsecs) return;

    if (_lastSound == soundName && (!allowRepeat)) return;

    // else
    
    std_msgs::msg::String msg;
    msg.data = soundName;
    pubSoundPlay->publish(msg);

    _lastBlockMsecs = blockMsecs;
    _lastTime = now;
    _lastSound = soundName;
}

void Brain::speak(string text, bool allowRepeat)
{
    auto log_ = [=](string msg) {
        // log->setTimeNow();
        // log->log("debug/speak", rerun::TextLog(msg));
    };

    const double COOLDOWN_MSECS = 2000.;
    if (!pubSpeak) {
        log_("publisher not found");
        return;
    }
    if (!config->soundEnable || config->soundPack != "espeak") {
        log_("config not compatible");
        return;
    }

    static string _lastText;
    static rclcpp::Time _lastTime;

    if (msecsSince(_lastTime) < COOLDOWN_MSECS) {
        log_("cooldown in process");
        return;
    }
    
    if (_lastText == text && (!allowRepeat)) {
        log_("repeat not allowed");
        return;
    }
    
    // else
    _lastTime = get_clock()->now();
    std_msgs::msg::String msg;
    msg.data = text;
    pubSpeak->publish(msg);

    _lastText = text;
}

double Brain::msecsSince(rclcpp::Time time)
{
    auto now = this->get_clock()->now();
    if (time.get_clock_type() != now.get_clock_type()) return 1e18;
    return (now - time).nanoseconds() / 1e6;
}

rclcpp::Time Brain::timePointFromHeader(std_msgs::msg::Header header) {
    auto stamp = header.stamp;
    // NOTE 似乎无论 use_sim_time 是否为真, 都使用的是 ROS_TIME 
    auto sec = stamp.sec;
    auto nanosec = stamp.nanosec;
    if (sec <= 0 || nanosec <= 0) {
        sec = 1;
        nanosec = 1;
        prtErr(format("Negative time: sec: %d nanosec: %d"));
    }
    return rclcpp::Time(sec, nanosec, RCL_ROS_TIME); // should not crash
    // return rclcpp::Time(stamp.sec, stamp.nanosec, RCL_ROS_TIME);  // should crash sometimes
}


void Brain::joystickCallback(const booster_interface::msg::RemoteControllerState &joy)
{
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/joystick", rerun::TextLog(msg));
    };
    // prtDebug("joy!!", RED_CODE);
    string soundPack = config->soundPack;

    // 通过手柄控制机器人, 不阻塞按键
    if (
        fabs(joy.lx) > 0.1
        || fabs(joy.ly) > 0.1
        || fabs(joy.rx) > 0.1
        || fabs(joy.ry) > 0.1
    ) {
        tree->setEntry<bool>("go_manual", true);
        // prtWarn("GO Manual");
    } else {
        tree->setEntry<bool>("go_manual", false);
        // prtWarn("Axe manual take over end");
    }

    // 按键响应顺序: LT 组合键, RT 组合键, 单键
    if (joy.lt && !joy.rt) { // LT 组合键
        // 用于在线调试参数
        if (joy.hat_u || joy.hat_d)
        {
            config->vxFactor += 0.01 * (joy.hat_u ? 1.0 : -1.0);
            speak(format("vx factor: %.2f", config->vxFactor));
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset),
                RED_CODE
            );
        }

        if (joy.hat_l || joy.hat_r)
        {
            config->yawOffset += 0.01 * (joy.hat_r ? 1.0 : -1.0);
            speak(format("yaw offset: %.2f", config->yawOffset));
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset),
                RED_CODE
            );
        }

        // 用于控制切换不同的状态
        if (joy.x)
        {
            tree->setEntry<int>("control_state", 1);
            client->setVelocity(0., 0., 0.);
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
            // playSound("sad");
        }
        if (joy.a)
        {
            tree->setEntry<int>("control_state", 2);
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
            // playSound("search");
        }
        if (joy.b)
        {
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
            // playSound("exited");
        }
        else if (joy.y)
        {
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
            log_("SWITCH ROLE");
            // playSound("talk");
        }
    }

    if (joy.rt) { // RT 组合键
        // Nothing for now
    }

    // else, 单键位
    if (!joy.lt && !joy.rt) {
        if (joy.lb) {
            tree->setEntry<bool>("assist_chase", true);
            prtDebug("Assit Chase");
            playSound(soundPack + "-chase", 5000);
        } else {
            tree->setEntry<bool>("assist_chase", false);
            // prtWarn("Exit Assit Chase");
        }
        if (joy.rb) {
            tree->setEntry<bool>("assist_kick", true);
            prtDebug("Assit Kick");
            playSound(soundPack + "-kick", 5000);
        } else {
            tree->setEntry<bool>("assist_kick", false);
            // prtWarn("Exit Assit Kick");
        }

        if (joy.hat_u) {
            playSound(soundPack + "-celebrate", 2000, true);
        } else if (joy.hat_d) {
            playSound(soundPack + "-regret", 2000, true);
        } else if (joy.hat_r) {
            playSound(soundPack + "-ready", 2000, true);
        } else if (joy.hat_l) {
            playSound(soundPack + "-taunt", 2000, true);
        }
    }
}

void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    data->timeLastGamecontrolMsg = get_clock()->now();

    // 处理比赛的一级状态
    auto lastGameState = tree->getEntry<string>("gc_game_state"); // 比赛的一级状态
    vector<string> gameStateMap = {
        "INITIAL", // 初始化状态, 球员在场外准备
        "READY",   // 准备状态, 球员进场, 并走到自己的始发位置
        "SET",     // 停止动作, 等待裁判机发出开始比赛的指令
        "PLAY",    // 正常比赛
        "END"      // 比赛结束
    };
    string gameState = gameStateMap[static_cast<int>(msg.state)];
    tree->setEntry<string>("gc_game_state", gameState);
    bool isKickOffSide = (msg.kick_off_team == config->teamId); // 我方是否是开球方
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);

    // 处理比赛的二级状态
    string gameSubStateType;
    switch (static_cast<int>(msg.secondary_state)) {
        case 0:
            gameSubStateType = "NONE";
            data->realGameSubState = "NONE";
            break;
        case 3:
            gameSubStateType = "TIMEOUT"; // 包含两队 timeout 和 裁判 timeout
            data->realGameSubState = "TIMEOUT";
            break;
        // 暂时不处理其它状态, 除 TIMEOUT 外, 都按 FREE_KICK 处理
        case 4:
            // gameSubStateType = "DIRECT_FREEKICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "DIRECT_FREEKICK";
            data->isDirectShoot = true;
            break;
        case 5:
            // gameSubStateType = "INDIRECT_FREEKICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "INDIRECT_FREEKICK";
            break;
        case 6:
            // gameSubStateType = "PENALTY_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "PENALTY_KICK";
            data->isDirectShoot = true;
            break;
        case 7:
            // gameSubStateType = "CORNER_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "CORNER_KICK";
            break;
        case 8:
            // gameSubStateType = "GOAL_KICK";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "GOAL_KICK";
            data->isDirectShoot = true;
            break;
        case 9:
            // gameSubStateType = "THROW_IN";
            gameSubStateType = "FREE_KICK";
            data->realGameSubState = "THROW_IN";
            break;
        default:
            gameSubStateType = "FREE_KICK";
            break;
    }
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};                               // STOP: 停下来; -> GET_READY: 移动到进攻或防守位置; -> SET: 站住不动
    string gameSubState = gameSubStateMap[static_cast<int>(msg.secondary_state_info[1])];
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    bool isSubStateKickOffSide = (static_cast<int>(msg.secondary_state_info[0]) == config->teamId); // 在二级状态下, 我方是否是开球方. 例如, 当前二级状态为任意球, 我方是否是开任意球的一方
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);

    // cout << "game state: " << gameState << " game sub state type: " << gameSubStateType << endl;
    // 找到队的信息
    game_controller_interface::msg::TeamInfo myTeamInfo;
    game_controller_interface::msg::TeamInfo oppoTeamInfo;
    if (msg.teams[0].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[0];
        oppoTeamInfo = msg.teams[1];
    }
    else if (msg.teams[1].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[1];
        oppoTeamInfo = msg.teams[0];
    }
    else
    {
        // 数据包中没有包含我们的队，不应该再处理了
        prtErr(format("received invalid game controller message team0 %d, team1 %d, teamId %d",
            msg.teams[0].team_number, msg.teams[1].team_number, config->teamId));
        return;
    }

    int liveCount = 0;
    int oppoLiveCount = 0;
    // 处理判罚状态. penalty[playerId - 1] 代表我方的球员是否处于判罚状态, 处理判罚状态意味着不能移动
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        data->penalty[i] = static_cast<int>(myTeamInfo.players[i].penalty);
        
        if (static_cast<int>(myTeamInfo.players[i].red_card_count) > 0) {
            data->penalty[i] = PENALTY_SUBSTITUTE;
        }

        if (data->penalty[i] == PENALTY_NONE) liveCount++;
        data->oppoPenalty[i] = static_cast<int>(oppoTeamInfo.players[i].penalty);

        if (static_cast<int>(oppoTeamInfo.players[i].red_card_count) > 0) {
            data->oppoPenalty[i] = PENALTY_SUBSTITUTE;
        }

        if (data->oppoPenalty[i] == PENALTY_NONE) oppoLiveCount++;
    }
    data->liveCount = liveCount;
    data->oppoLiveCount = oppoLiveCount;

    // cout << "penalty: " << data->penalty[0] << " " << data->penalty[1] << " " << data->penalty[2] << " " << data->penalty[3] << endl;
    // cout << "oppo penalty: " << data->oppoPenalty[0] << " " << data->oppoPenalty[1] << " " << data->oppoPenalty[2] << " " << data->oppoPenalty[3] << endl;
    bool lastIsUnderPenalty = tree->getEntry<bool>("gc_is_under_penalty");
    bool isUnderPenalty = (data->penalty[config->playerId - 1] != PENALTY_NONE); // 当前 robot 是否被判罚中
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);
    if (isUnderPenalty && !lastIsUnderPenalty) tree->setEntry<bool>("odom_calibrated", false); // 被判罚了, 则需要重新进场, 因此需要重新定位

    // log game state   
    log->setTimeNow();
    log->logToScreen(
        "tick/gamecontrol",
        format("Player: %d  Role: %s PrimaryStriker: %s GameState: %s  SubStateType: %s  SubState: %s UnderPenalty: %d isKickoff: %d isSubStateKickoff: %d", 
            config->playerId, tree->getEntry<string>("player_role").c_str(), isPrimaryStriker() ? "Yes" : "No", gameState.c_str(), gameSubStateType.c_str(), gameSubState.c_str(), isUnderPenalty, isKickOffSide, isSubStateKickOffSide
            ),
        0xFFFFFFFF,
        30.0
    );

    // FOR FUN 处理进球后的庆祝挥手的逻辑
    data->score = static_cast<int>(myTeamInfo.score);
    data->oppoScore = static_cast<int>(oppoTeamInfo.score);
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    // std::lock_guard<std::mutex> guard(data->brainMutex);
    
    // auto detection_time_stamp = msg.header.stamp;
    // rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    data->camConnected = true;
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->timeLastDet = timePoint; // 用于在调试中输出延迟信息

    auto gameObjects = getGameObjects(msg);

    // 对检测到的对象进行分组
    vector<GameObject> balls, goalposts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalposts.push_back(obj);
        if (obj.label == "Person")
        {
            persons.push_back(obj);

            // 为了调试方便, 可以在 config 中设置 treat_person_as_robot, 使得 Person 被当作 Robot 处理
            if (config->treatPersonAsRobot)
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }

    // 分别处理分组后的对象
    detectProcessBalls(balls);
    detectProcessGoalposts(goalposts);
    detectProcessMarkings(markings);
    detectProcessRobots(robots);

    // 处理并记录视野信息
    detectProcessVisionBox(msg);

    // logVisionBox(timePoint);
    logDetection(gameObjects);
}

void Brain::updateLinePosToField(FieldLine& line) {
    double __; // __ is a placeholder for transformations
    transCoord(
        line.posToRobot.x0, line.posToRobot.y0, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x0, line.posToField.y0, __
    );
    transCoord(
        line.posToRobot.x1, line.posToRobot.y1, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        line.posToField.x1, line.posToField.y1, __
    );
}

void Brain::fieldLineCallback(const vision_interface::msg::LineSegments &msg)
{
    // auto timestamp = msg.header.stamp;
    // rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);
    auto timePoint = timePointFromHeader(msg.header);

    auto now = get_clock()->now();
    data->timeLastLineDet = timePoint; // 用于在调试中输出延迟信息

    vector<FieldLine> lines = {};
    FieldLine line;

    double x0, y0, x1, y1, __; // __ is a placeholder for transformations 
    for (int i = 0; i < msg.coordinates.size() / 4; i++) {
        int index = i * 4;
        line.posToRobot.x0 = msg.coordinates[index]; line.posOnCam.x0 = msg.coordinates_uv[index];
        line.posToRobot.y0 = msg.coordinates[index + 1]; line.posOnCam.y0 = msg.coordinates_uv[index + 1];
        line.posToRobot.x1 = msg.coordinates[index + 2]; line.posOnCam.x1 = msg.coordinates_uv[index + 2];
        line.posToRobot.y1 = msg.coordinates[index + 3]; line.posOnCam.y1 = msg.coordinates_uv[index + 3];
        updateLinePosToField(line);
        line.timePoint = timePoint;
        // TODO infer line dir and id

        lines.push_back(line);
    }
    lines = processFieldLines(lines);
    data->setFieldLines(lines);

    { // log processed lines
        log->setTimeSeconds(timePoint.seconds());
        vector<rerun::LineStrip2D> logLinesOnField = {};
        vector<rerun::LineStrip2D> logLinesOnCam = {};
        vector<rerun::LineStrip2D> logLinesOnRobotFrame = {};
        vector<string> logLabels = {};
        vector<unsigned int> logColors = {};
        
        for (int i = 0; i < lines.size(); i++) {
            auto line = lines[i];
            logLinesOnRobotFrame.push_back(rerun::LineStrip2D({
                {line.posToRobot.x0, -line.posToRobot.y0}, 
                {line.posToRobot.x1, -line.posToRobot.y1}, 
            }));
            logLinesOnField.push_back(rerun::LineStrip2D({
                {line.posToField.x0, -line.posToField.y0}, 
                {line.posToField.x1, -line.posToField.y1}, 
            }));
            logLinesOnCam.push_back(rerun::LineStrip2D({
                {line.posOnCam.x0, line.posOnCam.y0},
                {line.posOnCam.x1, line.posOnCam.y1},
            }));
            string label;
            unsigned int color = 0xFFFF00FF;
            if (line.type == LineType::GoalLine) {
                label = "GoalLine";
                color = 0xFF0000FF;
            }
            else if (line.type == LineType::TouchLine) {
                label = "TouchLine";
                color = 0xFF0000FF;
            } 
            else if (line.type == LineType::MiddleLine) label = "MiddleLine";
            else if (line.type == LineType::PenaltyArea) label = "PenaltyArea";
            else if (line.type == LineType::GoalArea) label = "GoalArea";
            else if (line.type == LineType::NA) label = "NA";
            else label = "Other"; // should not see this label logged

            label += format(" c = %.1f", line.confidence);

            // if (line.dir == LineDir::Horizontal) label += " Horizontal";
            // else if (line.dir == LineDir::Vertical) label += " Vertical";
            // else label += " NA";

            logLabels.push_back(label);
            logColors.push_back(color);
        };
        log->log(
            "field/det_lines",
            rerun::LineStrips2D(logLinesOnField)
                .with_colors(logColors)
                .with_radii(0.04)
                .with_draw_order(20)
                .with_labels(logLabels)
        );
        // log->log(
        //     "robotframe/det_lines",
        //     rerun::LineStrips2D(logLinesOnRobotFrame)
        //         .with_colors(logColors)
        //         .with_radii(0.04)
        //         .with_draw_order(20)
        //         .with_labels(logLabels)
        // );
        log->log(
            "image/det_lines",
            rerun::LineStrips2D(logLinesOnCam)
                .with_colors(logColors)
                .with_radii(1.0)
                .with_draw_order(20)
                .with_labels(logLabels)
        );
    }

    { // log original lines
        log->setTimeSeconds(timePoint.seconds());
        vector<rerun::LineStrip2D> logLinesOnField = {};
        vector<rerun::LineStrip2D> logLinesOnCam = {};
        vector<rerun::LineStrip2D> logLinesOnRobotFrame = {};

        for (int i = 0; i < lines.size(); i++) {
            auto line = lines[i];
            logLinesOnRobotFrame.push_back(rerun::LineStrip2D({
                {line.posToRobot.x0, -line.posToRobot.y0}, 
                {line.posToRobot.x1, -line.posToRobot.y1}, 
            }));
            logLinesOnField.push_back(rerun::LineStrip2D({
                {line.posToField.x0, -line.posToField.y0}, 
                {line.posToField.x1, -line.posToField.y1}, 
            }));
            logLinesOnCam.push_back(rerun::LineStrip2D({
                {line.posOnCam.x0, line.posOnCam.y0},
                {line.posOnCam.x1, line.posOnCam.y1},
            }));
        };
        // log->log(
        //     "field/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnField)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(0.1)
        //         .with_draw_order(10)
        // );
        // log->log(
        //     "robotframe/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnRobotFrame)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(0.1)
        //         .with_draw_order(10)
        // );
        // log->log(
        //     "image/det_lines_raw",
        //     rerun::LineStrips2D(logLinesOnCam)
        //         .with_colors(0xCCCCFFCC)
        //         .with_radii(3.0)
        //         .with_draw_order(10)
        // );
    }
}

void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{

    data->robotPoseToOdom.x = msg.x * config->robotOdomFactor;
    data->robotPoseToOdom.y = msg.y * config->robotOdomFactor;
    data->robotPoseToOdom.theta = msg.theta;

    // 根据 Odom 信息, 更新机器人在 Field 坐标系中的位置
    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

    // 发布tf变换
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    // 设置平移部分
    transform.transform.translation.x = data->robotPoseToOdom.x;
    transform.transform.translation.y = data->robotPoseToOdom.y;
    transform.transform.translation.z = 0.0;
    
    // 设置旋转部分（从欧拉角转换为四元数）
    tf2::Quaternion q;
    q.setRPY(0, 0, data->robotPoseToOdom.theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    log->setTimeNow();
    log->log("debug/odom_callback", rerun::TextLog(format("x: %.1f, y: %.1f, z: %.1f", data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta)));
    
    // 广播tf变换
    tf_broadcaster_->sendTransform(transform);

    // Log Odom 信息

    log->setTimeNow();
    auto color = 0x00FF00FF;
    if (!data->tmImAlive) color = 0x006600FF;
    else if (!data->tmImLead) color = 0x00CC00FF;
    string label = format("Cost: %.1f", data->tmMyCost);
    log->logRobot("field/robot", data->robotPoseToField, color, label, true);
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    data->headYaw = msg.motor_state_serial[0].q;
    data->headPitch = msg.motor_state_serial[1].q;
    log->log("debug/head_angles", rerun::TextLog(format("pitch: %.1f, yaw: %.1f", data->headYaw, data->headPitch)));
}

void Brain::imageCallback(const sensor_msgs::msg::Image &msg)
{

    static int counter = 0;
    counter++;
    if (counter % config->rerunLogImgInterval == 0)
    {
        // 未防止摄像头连接不好时, 自动降低分辨率, 影响 CamTrackBall 的计算, 更新分辨率配置
        config->camPixX = msg.width;
        config->camPixY = msg.height;
        log->log("debug/imageCallback", rerun::TextLog(format("img width: %.d, img height: %.d", msg.width, msg.height)));

        cv::Mat image;
        // 根据图像编码格式进行处理
        if (msg.encoding == "nv12" || msg.encoding == "NV12") {
            // NV12: Y plane (H x W) + interleaved UV (H/2 x W)
            size_t expected = (size_t)(msg.width * msg.height * 3 / 2);
            if (msg.data.size() < expected) {
                prtErr(format("NV12 buffer too small. got %zu expect >= %zu", msg.data.size(), expected));
                return;
            }
            cv::Mat yuv(msg.height + msg.height / 2, msg.width, CV_8UC1, const_cast<uint8_t*>(msg.data.data()));
            cv::cvtColor(yuv, image, cv::COLOR_YUV2BGR_NV12);
        } else if (msg.encoding == "bgra8") {
            // 创建 OpenCV Mat 对象，处理 BGRA 格式图像
            image = cv::Mat(msg.height, msg.width, CV_8UC4, const_cast<uint8_t *>(msg.data.data()));
            cv::Mat imageBGR;
            // 将 BGRA 转换为 BGR，忽略 Alpha 通道
            cv::cvtColor(image, imageBGR, cv::COLOR_BGRA2BGR);
            image = imageBGR;
        } else if (msg.encoding == "bgr8") {
            // 原有 BGR8 处理逻辑
            image = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
        } else if (msg.encoding == "rgb8") {
            // 原有 RGB8 处理逻辑
            image = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        } else {
            // 处理其他编码格式，或者记录错误日志
            prtErr(format("Unsupported image encoding: %s", msg.encoding.c_str()));
            return;
        }

        // 压缩图像
        std::vector<uint8_t> compressed_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10}; // 10 表示压缩质量，可以根据需要调整
        cv::imencode(".jpg", image, compressed_image, compression_params);

        // 将压缩后的图像数据传递给 rerun
        // double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        // log->setTimeSeconds(time);
        log->setTimeSeconds(timePointFromHeader(msg.header).seconds());
        log->log("image/img", rerun::EncodedImage::from_bytes(compressed_image));
    }
}

void Brain::headPoseCallback(const geometry_msgs::msg::Pose& msg)
{
    // 计算 head_to_base 矩阵
    Eigen::Matrix4d headToBase = Eigen::Matrix4d::Identity();
    
    // 从四元数获取旋转矩阵
    Eigen::Quaterniond q(
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z
    );
    headToBase.block<3,3>(0,0) = q.toRotationMatrix();
    
    // 设置平移向量
    headToBase.block<3,1>(0,3) = Eigen::Vector3d(
        msg.position.x,
        msg.position.y,
        msg.position.z
    );

    // // 定义并计算 cam_to_head 矩阵
    // Eigen::Matrix4d camToHead;
    // camToHead << 0,  0,  1,  0,
    //             -1,  0,  0,  0,
    //              0, -1,  0,  0,
    //              0,  0,  0,  1;

    // 计算 cam_to_base 矩阵并存储
    data->camToRobot = headToBase * config->camToHead;
}

void Brain::recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg)
{
    // uint8_t state; // IS_READY = 0, IS_FALLING = 1, HAS_FALLEN = 2, IS_GETTING_UP = 3,  
    // uint8_t is_recovery_available; // 1 for available, 0 for not available
    // 使用 RobotRecoveryState 结构，将msg里面的msg转换为RobotRecoveryState
    try
    {
        const std::vector<unsigned char>& buffer = msg.msg;
        RobotRecoveryStateData recoveryState;
        memcpy(&recoveryState, buffer.data(), buffer.size());

        vector<RobotRecoveryState> recoveryStateMap = {
            RobotRecoveryState::IS_READY,
            RobotRecoveryState::IS_FALLING,
            RobotRecoveryState::HAS_FALLEN,
            RobotRecoveryState::IS_GETTING_UP
        };
        this->data->recoveryState = recoveryStateMap[static_cast<int>(recoveryState.state)];
        this->data->isRecoveryAvailable = static_cast<bool>(recoveryState.is_recovery_available);
        this->data->currentRobotModeIndex = static_cast<int>(recoveryState.current_planner_index);
        
        // cout << "recoveryState: " << static_cast<int>(recoveryState.state) << endl;
        // cout << "recovery is available: " << static_cast<int>(recoveryState.is_recovery_available) << endl;
        // cout << "current planner idx: " << static_cast<int>(recoveryState.current_planner_index) << endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


int Brain::markCntOnFieldLine(const string markType, const FieldLine line, const double margin) {
    int cnt = 0;
    auto markings = data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        auto marking = markings[i];
        if (marking.label == markType) {
            Point2D point = {marking.posToField.x, marking.posToField.y};
            if (fabs(pointPerpDistToLine(point, line.posToField)) < margin) {
                cnt += 1;
            }
        }
    }
    return cnt;
}

int Brain::goalpostCntOnFieldLine(const FieldLine line, const double margin) {
    int cnt = 0;
    auto goalposts = data->getGoalposts();
    for (int i = 0; i < goalposts.size(); i++) {
        auto post = goalposts[i];
        Point2D point = {post.posToField.x, post.posToField.y};
        if (pointMinDistToLine(point, line.posToField) < margin) {
            cnt += 1;
        }
    }
    return cnt;
}

bool Brain::isBallOnFieldLine(const FieldLine line, const double margin) {
    auto ballPos = data->ball.posToField;
    Point2D point = {ballPos.x, ballPos.y}; 
    return fabs(pointPerpDistToLine(point, line.posToField)) < margin;
}

void Brain::identifyFieldLine(FieldLine& line) {
    auto mapLines = config->mapLines;
    FieldLine mapLine;
    double confidence;
    line.type = LineType::NA;

    double bestConfidence = 0;
    double secondBestConfidence = 0;
    int bestIndex = -1;
    for (int i = 0; i < mapLines.size(); i++) {
        mapLine = mapLines[i];
        confidence = line.dir == mapLine.dir ? 
            probPartOfLine(line.posToField, mapLine.posToField)
            : 0.0;

        // Boost confidence with other features
        if (mapLine.type == LineType::GoalLine) { 
            confidence += 0.3 * markCntOnFieldLine("TCross", line, 0.2);
            confidence += 0.5 * goalpostCntOnFieldLine(line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "CORNER_KICK")
            ) confidence += 0.3; // 角球时, 球在底线上
        }
        if (mapLine.type == LineType::MiddleLine) {
            confidence += 0.3 * markCntOnFieldLine("XCross", line, 0.2);
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK")
            ) confidence += 0.3; // 门球时, 球在中线上
        }
        if (mapLine.type == LineType::TouchLine) {
            if (
                isBallOnFieldLine(line)
                && (tree->getEntry<string>("gc_game_sub_state") == "GET_READY" || tree->getEntry<string>("gc_game_sub_state") == "SET")
                && (data->realGameSubState == "GOAL_KICK" || data->realGameSubState == "CORNER_KICK" || data->realGameSubState == "THROW_IN")
            ) confidence += 0.3; // 发角球, 门球和边线球时, 球在边线上
        }
        
        // 防止将 goalarealine 误认为 goalline
        auto fd = config->fieldDimensions;
        if (
            mapLine.type == LineType::GoalLine
            && fabs(line.posToField.y0) < fd.goalAreaWidth / 2 + 0.5
            && fabs(line.posToField.y1) < fd.goalAreaWidth / 2 + 0.5
        ) confidence -= 0.3;

        // 防止将 penalty area 误认为 touchline
        if (
            mapLine.type == LineType::TouchLine
            && min(fabs(line.posToField.x0), fabs(line.posToField.x1)) > fd.length / 2.0 -  fd.penaltyAreaLength - 0.5
            && line.posToField.x0 * line.posToField.x1 > 0
        ) confidence -= 0.3;

        double length = norm(line.posToField.x0 - line.posToField.x1, line.posToField.y0 - line.posToField.y1);
        if (length < 0.5) confidence -= 0.5;
        else if (length < 1.0) confidence -= 0.1;
        
        if (confidence > bestConfidence) {
            secondBestConfidence = bestConfidence;
            bestConfidence = confidence;
            bestIndex = i;
        }
    }

    if (bestConfidence - secondBestConfidence < 0.5) bestConfidence -= 0.5;



    if (bestIndex >= 0 && bestIndex < mapLines.size()) {
        line.type = mapLines[bestIndex].type;
        line.half = mapLines[bestIndex].half;
        line.side = mapLines[bestIndex].side;
        line.confidence = bestConfidence;
        return;
    }

    // else 
    line.type = LineType::NA;
    line.half = LineHalf::NA;
    line.side = LineSide::NA;
    line.confidence = 0.0;
    return;
}

void Brain::identifyMarking(GameObject& marking) {
    double minDist = 100;
    double secMinDist = 100;
    int mmIndex = -1;
    for (int i = 0; i < config->mapMarkings.size(); i++) {
       auto mm = config->mapMarkings[i];
       
       if (mm.type != marking.label) continue;

       double dist = norm(marking.posToField.x - mm.x, marking.posToField.y - mm.y);

       if (dist < minDist) {
           secMinDist = minDist;
           minDist = dist;
           mmIndex = i;
       } else if (dist < secMinDist) {
           secMinDist = dist; 
       }
    }

    auto fd = config->fieldDimensions;
    if (
        mmIndex >=0 && mmIndex < config->mapMarkings.size()
        && minDist < 1.5 * 14 / fd.length // 1.0 for adultsize
        && secMinDist - minDist > 1.5 * 14 / fd.length // 2.0 for adultsize
        // && marking.confidence > 70 
    ) {
        marking.id = mmIndex;
        marking.name = config->mapMarkings[mmIndex].name;
        marking.idConfidence = 1.0;
    } else {
        marking.id = -1;
        marking.name = "NA";
        marking.idConfidence = 0.0;
    }
}


void Brain::identifyGoalpost(GameObject& goalpost) {
    string side = "NA";
    string half = "NA";
    if (goalpost.posToField.x > 0) half = "O";
    else half = "S";

    if (goalpost.posToField.y > 0) side = "L";
    else side = "R";
    
    goalpost.id = 0;
    goalpost.name = half + side;
    goalpost.idConfidence = 1.0;
    // TODO 参考 markings, 做更为精细的 goalpostid
}

vector<FieldLine> Brain::processFieldLines(vector<FieldLine>& fieldLines) {
    vector<FieldLine> original = fieldLines;
    vector<FieldLine> res;
    

    int sizeBefore = original.size();
    // merge lines that are actually the same line
    for (int i = 0; i < original.size(); i++) {
        for (int j = i + 1; j < original.size(); j++) {
            auto line1 = original[i].posToField;
            auto line2 = original[j].posToField;
            if (isSameLine(line1, line2, 0.1, 1.0)) {
                FieldLine mergedLine;
                mergedLine.posToField = mergeLines(line1, line2);
                mergedLine.posToRobot = mergeLines(original[i].posToRobot, original[j].posToRobot);
                mergedLine.posOnCam = mergeLines(original[i].posOnCam, original[j].posOnCam);
                mergedLine.timePoint = original[i].timePoint;

                // replace first line in original with merged line and remove second line
                original[i] = mergedLine;
                original.erase(original.begin() + j);
                j--;
            }
        }
    }
    int sizeAfter = original.size();
    // prtWarn(format("Merged %d lines into %d lines", sizeBefore, sizeAfter));
    // return original;
    // filter out lines that are too short and infer direction while ditch lines whose dir cannot be inferred
    double valve = 0.2;
    for (int i = 0; i < original.size(); i++) {
        auto line = original[i];
        auto lineDir = atan2(line.posToField.y1 - line.posToField.y0, line.posToField.x1 - line.posToField.x0);

        if (fabs(toPInPI(lineDir - M_PI)) < 0.1 || fabs(lineDir) < 0.1) line.dir = LineDir::Vertical;
        else if (fabs(toPInPI(lineDir - M_PI/2)) < 0.1 || fabs(toPInPI(lineDir + M_PI/2)) < 0.1) line.dir = LineDir::Horizontal;
        else continue;

        // if line is direction can be verified, check if it is long enough
        if (lineLength(line.posToField) > valve) {
            res.push_back(line);
        }
    }

    // identify each line 
    for (int i = 0; i < res.size(); i++) {
        identifyFieldLine(res[i]);
    }
    return res;
}


vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;

    // auto timestamp = detections.header.stamp;
    // rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);
    auto timePoint = timePointFromHeader(detections.header);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        GameObject gObj;

        gObj.timePoint = timePoint;
        gObj.label = obj.label;
        gObj.color = obj.color;

        if (obj.target_uv.size() == 2)
        { // 地面标志点的精确像素位置信息
            gObj.precisePixelPoint.x = static_cast<double>(obj.target_uv[0]);
            gObj.precisePixelPoint.y = static_cast<double>(obj.target_uv[1]);
        }

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;

        // 深度优先
        // if (obj.position.size() > 0 && !(obj.position[0] == 0 && obj.position[1] == 0))
        // { // 深度测距成功， 以深度测距为准
        //     gObj.posToRobot.x = obj.position[0];
        //     gObj.posToRobot.y = obj.position[1];
        // }
        // else
        // { // 深度测距失败，以投影距离为准
        //     gObj.posToRobot.x = obj.position_projection[0];
        //     gObj.posToRobot.y = obj.position_projection[1];
        // } // 注意，z 值没有用到

        // 不用深度测距, 直接用投影距离
        gObj.posToRobot.x = obj.position_projection[0];
        gObj.posToRobot.y = obj.position_projection[1];

        // 计算角度
        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->robotHeight, gObj.range); // 注意这是一个近似值

        // 计算对象在 field 坐标系中的位置
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z // 注意, z 没有在其它地方使用, 这里仅为参数占位使用
        );

        res.push_back(gObj);
    }

    return res;
}

void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    double bestConfidence = 0;
    int indexRealBall = -1;  // 认为哪一个球是真的, -1 表示没有检测到球

    // 找出最可能的真球
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];
        auto oldBall = data->ball;

        // 防止把天上的灯识别为球
        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 15.0)
            continue;

        // 排除在场外太远的球, 注意这个功能会影响 cahse 等功能. 先不采用.
        // if (isBallOut(2.0, 2.0))
        //     continue;

        // 如果检测出的球与上次看到的球, 距离和时间都很近, 则对其 confidence 进行适当加分
        // double c = ballObj.confidence;
        // double oldC = oldBall.confidence;
        // double msecs = msecsSince(oldBall.timePoint);
        // double dist = norm(ballObj.posToField.x - oldBall.posToField.x, ballObj.posToField.y - oldBall.posToField.y);
        // ballObj.confidence += 0.5 * max(oldC, 0.0) * max(1 - msecs/1000, 0.0) * max(1 - dist / 2, 0.0);
        // ballObj.confidence = min(100.0, ballObj.confidence);
        // log->setTimeNow();
        // log->log("/debug/oldc_newc", rerun::TextLog(format(
        //     "oldc: %.2f  newc: %.2f",
        //     oldC,
        //     ballObj.confidence
        // )));

        // 判断: 如果置信度太低, 则认为是误检
        if (ballObj.confidence < config->ballConfidenceThreshold)
            continue;

        // TODO 加入更多排除参数, 例如在身体上, 明显在球场外, 位置突然大幅度变化等
        // 被遮挡的条件要加入. 如果突然消失, 没有遮挡的话, 则只相信一小会儿, 如果有遮挡, 可以相信比较长的时间.

        // 找出剩下的球中, 置信度最高的
        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }

    if (indexRealBall >= 0)
    { // 检测到球了
        data->ballDetected = true;

        data->ball = ballObjs[indexRealBall];
        data->ball.confidence = bestConfidence;

        tree->setEntry<bool>("ball_location_known", true);
        updateBallOut();        
    }
    else
    { // 没有检测到球
        log->setTimeNow();
        // log->log("image/detection_boxes_realball", rerun::Clear::FLAT);
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;
        // data->ball.confidence = 0; // DO NOT set confidence to 0, confidence decay depends on this.
    }

    // 计算机器人到球的向量, 在 field 坐标系中的方向
    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);
}

void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    // // for testing 测距稳定性 ---------
    // for (int i = 0; i < markingObjs.size(); i++) {
    //    auto m = markingObjs[i];
    //    if (m.label != "PenaltyPoint" || m.posToField.x < 0.0) continue;

    //    double range = norm(m.posToField.x - data->robotPoseToField.x, m.posToField.y - data->robotPoseToField.y);

    //    log->setTimeNow();
    //    log->log("debug/penalty_point/range", rerun::Scalar(range));
    //    log->log("debug/penalty_point/x", rerun::Scalar(m.posToField.x));
    //    log->log("debug/penalty_point/y", rerun::Scalar(m.posToField.y));
    // }
    // // end testing

    const double confidenceValve = 50; // confidence 低于这个阈值, 排除
    vector<GameObject> markings = {};
    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        // 判断: 如果置信度太低, 则认为是误检
        if (marking.confidence < confidenceValve)
            continue;

        // 排除天的上误识别标记
        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 15.0)
            continue;

        // 如果通过了重重考验, 则记入 brain
        identifyMarking(marking);
        markings.push_back(marking);
    }
    data->setMarkings(markings);

    // log identified markings
    log->setTimeNow();
    vector<rerun::LineStrip2D> circles = {};
    vector<string> labels = {};

    for (int i = 0; i < markings.size(); i++) {
        auto m = markings[i];
        if (markings[i].id >= 0) {
            circles.push_back(log->circle(m.posToField.x, -m.posToField.y, 0.3));
            labels.push_back(format("%s c=%.2f", m.name.c_str(), m.idConfidence));
        }
    }
    
    log->log("field/identified_markings",
        rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(circles))
       .with_radii(0.01)
       .with_labels(labels)
       .with_colors(0xFFFFFFFF));
}

void Brain::detectProcessGoalposts(const vector<GameObject> &goalpostObjs)
{
    const double confidenceValve = 50; // confidence 低于这个阈值, 排除
    vector<GameObject> goalposts = {};

    for (int i = 0; i < goalpostObjs.size(); i++) {
        auto goalpost = goalpostObjs[i];

        // 判断: 如果置信度太低, 则认为是误检
        if (goalpost.confidence < confidenceValve)
            continue;

        identifyGoalpost(goalpost);
        goalposts.push_back(goalpost);
    }
    data->setGoalposts(goalposts);

    // log identified goalposts
    log->setTimeNow();
    vector<rerun::LineStrip2D> circles = {};
    vector<string> labels = {};

    for (int i = 0; i < goalposts.size(); i++) {
        auto p = goalposts[i];
        if (goalposts[i].id >= 0) {
            circles.push_back(log->circle(p.posToField.x, -p.posToField.y, 0.3));
            labels.push_back(format("%s c=%.2f", p.name.c_str(), p.idConfidence));
        }
    }
    
    // log->log("field/identified_goalposts",
    //     rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(circles))
    //     .with_radii(0.01)
    //     .with_labels(labels)
    //     .with_colors(0xFFFFFFFF));
}


void Brain::detectProcessRobots(const vector<GameObject> &robotObjs) {
    // auto robots = data->getRobots();

    // for (int i = 0; i < robotObjs.size(); i++) {
    //     auto rbt = robotObjs[i];
    //     if (rbt.confidence < 50) continue;

    //     // find nearest robot in memory
    //     double minDist = 1e6; 
    //     int minIndex = -1;
    //     for (int j = 0; j < robots.size(); j++) {
    //         auto rm = robots[j];
    //         double dist = norm(rm.posToField.x - rbt.posToField.x, rm.posToField.y - rbt.posToField.y);
    //         if (dist < minDist) {
    //             minDist = dist; 
    //             minIndex = j;
    //         }
    //     }
    //     // prtDebug(format("minDist = %.2f", minDist));
    //     if (minDist < 0.5) { // 认为是同一个机器人
    //         robots[minIndex] = rbt;
    //     } else { // 认为是不同的机器人
    //         robots.push_back(rbt);
    //     }
    // }
    // // 注意这里不清理已经看不见的机器人, 而是在 updateMemory 中进行处理.

    vector<GameObject> robots = {};
    for (int i = 0; i < robotObjs.size(); i++) {
        auto rbt = robotObjs[i];
        if (rbt.confidence < 50) continue;
        
        // else
        robots.push_back(rbt);
    }

    data->setRobots(robots);
}


void Brain::detectProcessVisionBox(const vision_interface::msg::Detections &msg) {    
    // auto detection_time_stamp = msg.header.stamp;
    // rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    auto timePoint = timePointFromHeader(msg.header);

    // 处理并记录视野信息
    VisionBox vbox;
    vbox.timePoint = timePoint;
    for (int i = 0; i < msg.corner_pos.size(); i++) vbox.posToRobot.push_back(msg.corner_pos[i]);

    // 处理左上与右上两点 x 小于 0 , 实际为无限远的场景
    const double VISION_LIMIT = 20.0;
    vector<vector<double>> v = {};
    for (int i = 0; i < 4; i++) {
        int start = i; int end = (i + 1) % 4;
        v.push_back({vbox.posToRobot[end * 2] - vbox.posToRobot[start * 2], vbox.posToRobot[end * 2 + 1] - vbox.posToRobot[start * 2 + 1]});
        v.push_back({-vbox.posToRobot[end * 2] + vbox.posToRobot[start * 2], -vbox.posToRobot[end * 2 + 1] + vbox.posToRobot[start * 2 + 1]});
    }

    for (int i = 0; i < 2; i++) {
        double ox = vbox.posToRobot[2* i]; double oy = vbox.posToRobot[2 * i + 1];
        if (
            (i == 0 && crossProduct(v[5], v[6]) < 0)
            || (i == 1 && crossProduct(v[3], v[4]) < 0)
        ){
            vbox.posToRobot[2 * i] = -ox / fabs(ox) * VISION_LIMIT;
            vbox.posToRobot[2 * i + 1] = -oy / fabs(oy) * VISION_LIMIT;
        }
    }

    // 转换到 field 坐标系中
    for (int i = 0; i < 5; i++) {
        double xr, yr, xf, yf, __;
        xr = vbox.posToRobot[2 * i];
        yr = vbox.posToRobot[2 * i + 1];
        transCoord(
            xr, yr, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            xf, yf, __
        );
        vbox.posToField.push_back(xf);
        vbox.posToField.push_back(yf);
    }
    
    // 一次性将结果赋值到 data 中
    data->visionBox = vbox;
}

void Brain::logVisionBox(const rclcpp::Time &timePoint) {
    if (data->visionBox.posToField.size() >= 10) {
        auto v = data->visionBox.posToField;

        rerun::LineStrip2D logLines({{v[0], -v[1]}, {v[2], -v[3]}, {v[4], -v[5]}, {v[6], -v[7]}, {v[0], -v[1]}});
        log->setTimeSeconds(timePoint.seconds());
        log->log(
            "field/visionBox",
            rerun::LineStrips2D(logLines)
            .with_colors(0x0000FFCC)
            .with_radii(0.01)
        );
    }
}

void Brain::logDetection(const vector<GameObject> &gameObjects, bool logBoundingBox) {
    if (gameObjects.size() == 0) {
        if (logBoundingBox) log->log("image/detection_boxes", rerun::Clear::FLAT);
        log->log("field/detection_points", rerun::Clear::FLAT);
        // log->log("robotframe/detection_points", rerun::Clear::FLAT);
        return;
    }
    
    // else 
    rclcpp::Time timePoint = gameObjects[0].timePoint;
    log->setTimeSeconds(timePoint.seconds());

    map<std::string, rerun::Color> detectColorMap = {
        {"LCross", rerun::Color(0xFFFF00FF)},
        {"TCross", rerun::Color(0x00FF00FF)},
        {"XCross", rerun::Color(0x0000FFFF)},
        {"Person", rerun::Color(0xFF00FFFF)},
        {"Goalpost", rerun::Color(0x00FFFFFF)},
        {"Opponent", rerun::Color(0xFF0000FF)},
        {"PenaltyPoint", rerun::Color(0xFF9900FF)},
    };

    // for logging boundingBoxes
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    vector<rerun::Text> labels;
    vector<rerun::Color> colors;

    // for logging marker points in robot frame
    vector<rerun::Vec2D> points;
    vector<rerun::Vec2D> points_r; // robot frame
    vector<double> radiis;

    for (int i = 0; i < gameObjects.size(); i++)
    {
        auto obj = gameObjects[i];
        auto label = obj.label;
        labels.push_back(rerun::Text(
            format("%s x:%.2f y:%.2f c:%.1f", 
                label == "Opponent" || label == "Person" ? (label + "[" + obj.color + "]").c_str() : label.c_str(), 
                obj.posToRobot.x, 
                obj.posToRobot.y, 
                obj.confidence)
            )
        );
        points.push_back(rerun::Vec2D{obj.posToField.x, -obj.posToField.y}); // y 取反是因为 rerun Viewer 的坐标系是左手系。转一下看起来更方便。
        points_r.push_back(rerun::Vec2D{obj.posToRobot.x, -obj.posToRobot.y});
        mins.push_back(rerun::Vec2D{obj.boundingBox.xmin, obj.boundingBox.ymin});
        sizes.push_back(rerun::Vec2D{obj.boundingBox.xmax - obj.boundingBox.xmin, obj.boundingBox.ymax - obj.boundingBox.ymin});

        // if (obj.label == "Opponent") radiis.push_back(0.5);
        radiis.push_back(0.1);

        auto color = rerun::Color(0xFFFFFFFF);

        auto it = detectColorMap.find(label);
        if (it != detectColorMap.end())
        {
            color = detectColorMap[label];
        }
        else
        {
            // do nothing, use default
            // colors.push_back(rerun::Color(0xFFFFFFFF));
        }
        if (label == "Ball" && isBallOut(0.2, 10.0))
            color = rerun::Color(0x000000FF);
        if (label == "Ball" && obj.confidence < config->ballConfidenceThreshold)
            color = rerun::Color(0xAAAAAAFF);
        colors.push_back(color);
    }

    
    if (logBoundingBox) log->log("image/detection_boxes",
             rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                 .with_labels(labels)
                 .with_colors(colors));

    log->log("field/detection_points",
             rerun::Points2D(points)
                 .with_colors(colors)
                 .with_radii(radiis)
             // .with_labels(labels)
    );
    // log->log("robotframe/detection_points",
    //          rerun::Points2D(points_r)
    //              .with_colors(colors)
    //              .with_radii(radiis)
    //          // .with_labels(labels)
    // );
}


void Brain::logMemRobots() {
    auto rbts = data->getRobots();
    // prtDebug(format("logMemRobots called, robotsize = %d", rbts.size()), RED_CODE);

    if (rbts.size() == 0) {
        log->log("field/mem_robots", rerun::Clear::FLAT);
        // log->log("robotframe/mem_robots", rerun::Clear::FLAT);
        return;
    }
    
    // else 
    log->setTimeNow();
    // vector<rerun::Vec2D> points;
    vector<rerun::LineStrip2D> circles;
    vector<rerun::Vec2D> points_r; // robot frame
    for (int i = 0; i < rbts.size(); i++)
    {
        auto rbt = rbts[i];
        log->logRobot("field/robots", Pose2D({rbt.posToField.x, rbt.posToField.y, -M_PI}), 0xFF0000FF);
        // circles.push_back(log->circle(rbt.posToField.x, -rbt.posToField.y, 0.5)); // y 取反是因为 rerun Viewer 的坐标系是左手系。转一下看起来更方便。
        // points_r.push_back(rerun::Vec2D{rbt.posToRobot.x, -rbt.posToRobot.y});
    }

    // log->log("field/mem_robots",
    //          rerun::LineStrips2D(circles)
    //              .with_colors(0xFF0000AA)
    //              .with_radii(0.01)
    //          // .with_labels(labels)
    // );
    // log->log("robotframe/mem_robots",
    //          rerun::Points2D(points_r)
    //              .with_colors(0xFF0000AA)
    //              .with_radii(0.5)
    //          // .with_labels(labels)
    // );
}

void Brain::logObstacles() {
    // log->setTimeNow();
    // time is set on the outside
    
    // 记录障碍物(即有被占用的网格)
    auto obs = data->getObstacles();
    vector<rerun::Vec2D> points;
    vector<rerun::Color> colors;
    vector<rerun::Text> labels;
    const int occThreshold = get_parameter("obstacle_avoidance.occupancy_threshold").as_int();
    for (int i = 0; i < obs.size(); i++) {
        auto o = obs[i];

        if (o.confidence < occThreshold) continue; // 这个逻辑覆盖了后面的逻辑, 注掉可以以不同颜色 log 不同的置信度.

        points.push_back(rerun::Vec2D{o.posToField.x, -o.posToField.y});
        double mem_msecs = get_parameter("obstacle_avoidance.obstacle_memory_msecs").as_double();
        auto age = msecsSince(o.timePoint);
        uint8_t alpha = static_cast<uint8_t>(0xFF - 0xFF * age / mem_msecs);
        uint32_t color = (o.confidence > occThreshold) ? (0xFF000000 | alpha) : (0xFFFF0000 | alpha);
        colors.push_back(rerun::Color(color));

        labels.push_back(rerun::Text(format("count: %.0f age: %.0fms", o.confidence, age)));
    }
    log->log(
        "field/obstacles", 
        rerun::Points2D(points)
        .with_colors(colors)
        .with_labels(labels)
        .with_radii(0.1)
    );
}

void Brain::logDepth(int grid_x_count, int grid_y_count, vector<vector<int>> &grid_occupied, vector<rerun::Vec3D> &points_robot) {
    // time is set on the outside
    const double grid_size = get_parameter("obstacle_avoidance.grid_size").as_double();  // 网格大小
    const double x_min = 0.0, x_max = get_parameter("obstacle_avoidance.max_x").as_double();
    const double y_min = -get_parameter("obstacle_avoidance.max_y").as_double();
    const double y_max = -y_min;

    // 记录原始点云和网格
    vector<rerun::Position3D> vertices;
    vector<rerun::Color> vertex_colors;
    vector<array<uint32_t, 3>> triangle_indices;
    const int OCCUPANCY_THRESHOLD = get_parameter("obstacle_avoidance.occupancy_threshold").as_int(); // 设置一个显示用的阈值

    for (int i = 0; i < grid_x_count; i++) {
        for (int j = 0; j < grid_y_count; j++) {
            if (grid_occupied[i][j] > 0) {
                // 计算有障碍网格的四个顶点坐标
                double x0 = x_min + i * grid_size;
                double y0 = y_min + j * grid_size;
                double x1 = x0 + grid_size;
                double y1 = y0 + grid_size;

                // 添加四个顶点
                uint32_t base_index = vertices.size();
                vertices.push_back({x0, y0, 0.0});
                vertices.push_back({x1, y0, 0.0});
                vertices.push_back({x1, y1, 0.0});
                vertices.push_back({x0, y1, 0.0});

                // 设置颜色：根据占用情况设置不同的红色
                rerun::Color color;
                if (grid_occupied[i][j] > OCCUPANCY_THRESHOLD) {
                    color = rerun::Color(255, 0, 0, 255);  // RGBA, 红色
                } else {
                    color = rerun::Color(255, 255, 0, 255);  // RGBA, 黄色
                }
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);
                vertex_colors.push_back(color);

                // 添加两个三角形面
                triangle_indices.push_back({base_index, base_index + 1, base_index + 2});
                triangle_indices.push_back({base_index, base_index + 2, base_index + 3});
            }
        }
    }

    vector<uint32_t> point_colors;
    for (auto &point : points_robot) {
        float z_val = std::clamp(point.z(), 0.0f, 1.0f);
        const double obstacleMinHeight = get_parameter("obstacle_avoidance.obstacle_min_height").as_double();
        if (z_val < obstacleMinHeight) {
            point_colors.push_back(0x0000FFFF);
        } else {
            uint8_t r = static_cast<uint8_t>(z_val * 255);
            uint8_t g = static_cast<uint8_t>((1 - z_val) * 255);
            point_colors.push_back((r << 24) | (g << 16) | 0xFF);
        }
    }
    
    log->log("depth/depth_points",
            rerun::Points3D(points_robot)
                .with_radii(0.01)
                .with_colors(point_colors)
    );
    
    log->log("depth/grid_mesh",
            rerun::Mesh3D(vertices)
                .with_vertex_colors(vertex_colors)
                .with_triangle_indices(triangle_indices)
    );

    // 记录 ball exclusion box
    double r = get_parameter("obstacle_avoidance.ball_exclusion_radius").as_double();
    double h = get_parameter("obstacle_avoidance.ball_exclusion_height").as_double();
    log->log(
        "depth/ball_exclusion_box",
        rerun::Boxes3D::from_centers_and_half_sizes(
            {{ data->ball.posToRobot.x, data->ball.posToRobot.y, h/2}},
            {{ r, r, h/2}})
        .with_colors(0x00FF0044)     // 半透明绿色
    );
}

void Brain::logDebugInfo() {
    auto log_ = [=](string msg) {
        log->setTimeNow();
        log->log("debug/brain_tick", rerun::TextLog(msg));
    };
    string gameState = tree->getEntry<string>("gc_game_state");
    string gameSubState = tree->getEntry<string>("gc_game_sub_state");
    string gameSubStateType = tree->getEntry<string>("gc_game_sub_state_type");
    string isLead = data->tmImLead ? "ON" : "OFF";
    string ballOut = tree->getEntry<bool>("ball_out") ? "YES" : "NO";
    string ballDetected = data->ballDetected ? "YES" : "NO";
    string decision = tree->getEntry<string>("decision");
    string freeKickKickingOff = data->isFreekickKickingOff ? "YES" : "NO";
    string directShoot = data->isDirectShoot ? "YES" : "NO";
    string primaryStriker = isPrimaryStriker() ? "YES" : "NO";
    log_(format("Game State: %s, SubState: %s, SubStateType: %s, Lead: %s, Decision: %s, FreeKickKickingOff: %s, DirectShoot: %s, PrimaryStriker: %s",
        gameState.c_str(), gameSubState.c_str(), gameSubStateType.c_str(), isLead.c_str(), decision.c_str(), freeKickKickingOff.c_str(), directShoot.c_str(), primaryStriker.c_str()));

    log->setTimeNow();
    log->log("debug/my_cost_scalar", rerun::Scalar(data->tmMyCost));
    log->log("debug/my_lead_scalar", rerun::Scalar(data->tmImLead));
}

void Brain::updateRelativePos(GameObject &obj) {
    Pose2D pf;
    pf.x = obj.posToField.x;
    pf.y = obj.posToField.y;
    pf.theta = 0;
    Pose2D pr = data->field2robot(pf);
    obj.posToRobot.x = pr.x;
    obj.posToRobot.y = pr.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = asin(config->robotHeight / obj.range);
}

void Brain::updateFieldPos(GameObject &obj) {
    Pose2D pr;
    pr.x = obj.posToRobot.x;
    pr.y = obj.posToRobot.y;
    pr.theta = 0;
    Pose2D pf = data->robot2field(pr);
    obj.posToField.x = pf.x;
    obj.posToField.y = pf.y;
    obj.range = norm(obj.posToRobot.x, obj.posToRobot.y);
    obj.yawToRobot = atan2(obj.posToRobot.y, obj.posToRobot.x);
    obj.pitchToRobot = asin(config->robotHeight / obj.range);
}

void Brain::depthImageCallback(const sensor_msgs::msg::Image &msg)
{
    try {
        // 检查图像数据是否有效
        if (msg.data.empty() || msg.height == 0 || msg.width == 0) {
            RCLCPP_WARN(get_logger(), "Received empty depth image");
            return;
        }

        // 创建深度图像和转换
        cv::Mat depthFloat;
        // 根据图像编码格式进行处理
        if (msg.encoding == "16UC1" || msg.encoding == "mono16") {
            size_t expected = (size_t)msg.width * msg.height * sizeof(uint16_t);
            if (msg.data.size() < expected) {
                RCLCPP_ERROR(get_logger(), "Depth mono16 size mismatch");
                return;
            }
            cv::Mat depthRaw(msg.height, msg.width, CV_16UC1, const_cast<uint8_t*>(msg.data.data()));
            depthRaw.convertTo(depthFloat, CV_32FC1, 1.0 / 1000.0); // 若是实际深度单位 mm
        } else if (msg.encoding == "32FC1") {
            // 检查数据大小是否正确
            size_t expected_size = msg.height * msg.width * sizeof(float);
            if (msg.data.size() != expected_size) {
                RCLCPP_ERROR(get_logger(), "Depth image size mismatch: expected %zu, got %zu", 
                    expected_size, msg.data.size());
                return;
            }

            // 直接创建 32 位浮点数格式的深度图像
            depthFloat = cv::Mat(msg.height, msg.width, CV_32FC1, 
                const_cast<float*>(reinterpret_cast<const float*>(msg.data.data()))).clone();
            
        } else {
            RCLCPP_ERROR(get_logger(), "Unsupported depth image encoding: %s", msg.encoding.c_str());
            return;
        }

        vector<rerun::Vec3D> points_robot;  // for log

        const double fx = config->camfx;
        const double fy = config->camfy;
        const double cx = config->camcx;
        const double cy = config->camcy;
        // cout << "fx = " << fx << " fy = " << fy << " cx = " << cx << " cy = " << cy << endl;
        
        // 定义网格参数
        const double grid_size = get_parameter("obstacle_avoidance.grid_size").as_double();  // 网格大小
        const double x_min = 0.0, x_max = get_parameter("obstacle_avoidance.max_x").as_double();
        const double y_min = -get_parameter("obstacle_avoidance.max_y").as_double();
        const double y_max = -y_min;
        const int grid_x_count = static_cast<int>((x_max - x_min) / grid_size);
        const int grid_y_count = static_cast<int>((y_max - y_min) / grid_size);
        
        // 创建网格占用数组
        vector<vector<int>> grid_occupied(grid_x_count, vector<int>(grid_y_count, 0));
        
        // 处理深度图像点
        const int sampleStep = get_parameter("obstacle_avoidance.depth_sample_step").as_int();
        for (int y = 0; y < msg.height; y += sampleStep)
        {
            for (int x = 0; x < msg.width; x += sampleStep)
            {
                float depth = depthFloat.at<float>(y, x);
                if (depth > 0)
                {
                    // 转换到相机坐标系
                    double x_cam = (x - cx) * depth / fx;
                    double y_cam = (y - cy) * depth / fy;
                    double z_cam = depth;

                    // 转换到机器人坐标系
                    Eigen::Vector4d point_cam(x_cam, y_cam, z_cam, 1.0);
                    Eigen::Vector4d point_robot = data->camToRobot * point_cam;
                    
                    // 记录点用于可视化
                    points_robot.push_back(rerun::Vec3D{point_robot(0), point_robot(1), point_robot(2)});
                    
                    // 更新网格占用情况
                    const double Z_THRESHOLD = get_parameter("obstacle_avoidance.obstacle_min_height").as_double();
                    const double EXCLUDE_MAX_X = get_parameter("obstacle_avoidance.exclusion_x").as_double(); // 排除机器人自己的身体
                    const double EXCLUDE_MIN_X = -EXCLUDE_MAX_X;
                    const double EXCLUDE_MAX_Y = get_parameter("obstacle_avoidance.exclusion_y").as_double(); // 排除机器人自己的身体
                    const double EXCLUDE_MIN_Y = -EXCLUDE_MAX_Y;

                    auto isInRange = [&]() {
                        return point_robot(0) >= x_min && point_robot(0) < x_max
                            && point_robot(1) >= y_min && point_robot(1) < y_max;
                    };
                    auto isSelfBody = [&]() {
                        return point_robot(0) >= EXCLUDE_MIN_X && point_robot(0) <= EXCLUDE_MAX_X
                            && point_robot(1) >= EXCLUDE_MIN_Y && point_robot(1) <= EXCLUDE_MAX_Y;
                    };
                    auto isBall = [&]() {
                        double r = get_parameter("obstacle_avoidance.ball_exclusion_radius").as_double();
                        double h = get_parameter("obstacle_avoidance.ball_exclusion_height").as_double();
                        return fabs(point_robot(0) - data->ball.posToRobot.x) < r 
                            && fabs(point_robot(1) - data->ball.posToRobot.y) < r
                            && point_robot(2) < h;
                    };

                    if (
                        point_robot(2) > Z_THRESHOLD 
                        && isInRange()
                        &&!isSelfBody() 
                        &&!isBall()
                    )
                    {
                        int grid_x = static_cast<int>((point_robot(0) - x_min) / grid_size);
                        int grid_y = static_cast<int>((point_robot(1) - y_min) / grid_size);
                        grid_occupied[grid_x][grid_y] += 1;
                    }
                }
            }
        }

        auto obs_old = data->getObstacles();
        vector<GameObject> obs_new = {};

        // 本次看到的记入 obstables
        for (int i = 0; i < grid_x_count; i++) {
            for (int j = 0; j < grid_y_count; j++) {
                if (grid_occupied[i][j] > 0) {
                    GameObject obj;
                    obj.label = "Obstacle";
                    obj.timePoint = get_clock()->now();
                    obj.posToRobot.x = x_min + (i + 0.5) * grid_size;
                    obj.posToRobot.y = y_min + (j + 0.5) * grid_size;
                    obj.confidence = grid_occupied[i][j];
                    updateFieldPos(obj);
                    obs_new.push_back(obj);
                }
            }
        }

        // 清理旧 obstacle
        for (int i = 0; i < obs_old.size(); i++) {
           // 先把当前视野范围内的旧 obstacle 清空, 注意角度只是粗略计算, 并通过 offset 适当扩大了一些范围.
            double visionLeft = data->headYaw + config->camAngleX / 2;
            double visionRight = data->headYaw - config->camAngleX / 2;
            auto obs = obs_old[i];
            const double offset = 0.20;
            double obsYawLeft = atan2(obs.posToRobot.y - offset, obs.posToRobot.x + offset);
            double obsYawRight = atan2(obs.posToRobot.y + offset, obs.posToRobot.x + offset);
            if (obsYawLeft < visionLeft && obsYawRight > visionRight) continue; 

            // 如果旧的 obs 与新的 obs 太过接近, 则认为旧的 obs 已经不存在, 防止边界情况下 obs 堆积
            bool found = false;
            for (int j = 0; j < obs_new.size(); j++) {
                auto obs_n = obs_new[j];
                double dist = norm(obs.posToRobot.x - obs_n.posToRobot.x, obs.posToRobot.y - obs_n.posToRobot.y);
                if (dist < 0.5 * grid_size) {
                    found = true;
                    break;
                }
            }
            if (found) continue;

            // else
            obs_new.push_back(obs);
        }

        
        data->setObstacles(obs_new); // note: 此处不清空超时的旧 obstacles, 而在 tick 中清理
        log->setTimeSeconds(timePointFromHeader(msg.header).seconds());
        logDepth(grid_x_count, grid_y_count, grid_occupied, points_robot);
        logObstacles();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in depth image callback: %s", e.what());
    }
}

double Brain::distToObstacle(double angle) {
    auto obs = data->getObstacles();
    double minDist = 1e9;
    // double obstacleThreshold = static_cast<double>(config->obstacleThreshold);
    double obstacleThreshold = static_cast<double>(get_parameter("obstacle_avoidance.occupancy_threshold").as_int());

    double collisionThreshold = config->collisionThreshold;

    for (int i = 0; i < obs.size(); i++) {
        if (obs[i].confidence < obstacleThreshold) continue;

        auto o = obs[i];
        Line line = {
            0, 0,
            cos(angle) * 100, sin(angle) * 100
        };
        double perpDist = fabs(pointPerpDistToLine(Point2D{o.posToRobot.x, o.posToRobot.y}, line));
        if (perpDist < collisionThreshold) {
            double dist = innerProduct(vector<double>{o.posToRobot.x, o.posToRobot.y}, vector<double>{cos(angle), sin(angle)});
            if (dist > 0 && dist < minDist) {
                minDist = dist;
            }
        }
    }
    return minDist;
}

vector<double> Brain::findSafeDirections(double startAngle, double safeDist, double step) {
    double safeAngleLeft = startAngle;
    double safeAngleRight = startAngle;
    double leftFound = 0;
    double rightFound = 0;
    for (double angle = startAngle; angle < startAngle + M_PI; angle += step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleLeft = angle;
            leftFound = 1;
            break;
        }
    }
    for (double angle = startAngle; angle > startAngle - M_PI; angle -= step) {
        if (distToObstacle(angle) > safeDist) {
            safeAngleRight = angle;
            rightFound = 1;
            break;
        }
    }

    return vector<double>{leftFound, toPInPI(safeAngleLeft), rightFound, toPInPI(safeAngleRight)};
}

double Brain::calcAvoidDir(double startAngle, double safeDist) {
    auto res = findSafeDirections(startAngle, safeDist);
    bool leftFound = res[0] > 0.5;
    bool rightFound = res[2] > 0.5;
    double angleLeft = res[1];
    double angleRight = res[3]; 
    double determinedAngle = 0;
    if (leftFound && rightFound) {
        determinedAngle = fabs(angleLeft) < fabs(angleRight) ? angleLeft : angleRight;
    } else if (leftFound) {
        determinedAngle = angleLeft;
    } else if (rightFound) {
        determinedAngle = angleRight;
    } else {
        return 0;
    }
    return toPInPI(determinedAngle);
}

void Brain::updateLogFile() {
    if (config->rerunLogEnableFile && msecsSince(data->timeLastLogSave) > config->rerunLogMaxFileMins * 60000)
        log->updateLogFilePath();
}

// ------------------------------------------------------ 调试 log 相关 ------------------------------------------------------
void Brain::logObstacleDistance() {
    log->setTimeNow();

    // log obstacle distance for test
    vector<rerun::LineStrip2D> lines = {};   
    for (int i = 0; i < 180; i++) {
        double angle = i * M_PI / 90;
        double dist = min(5.0, distToObstacle(angle));
        double angle_f = toPInPI(data->robotPoseToField.theta + angle);
        lines.push_back(
            rerun::LineStrip2D({
                {data->robotPoseToField.x, -data->robotPoseToField.y}, 
                {data->robotPoseToField.x + cos(-angle_f) * dist, -data->robotPoseToField.y + sin(-angle_f) * dist}
            })
        );
    }
    log->log(
        "field/obstacle_distance",
        rerun::LineStrips2D(lines)
            .with_colors(0x666666FF)
            .with_radii(0.01)
            .with_draw_order(-10)
    );
}

void Brain::logLags() {
    log->setTimeNow();
    auto color = 0x00FF00FF;
    
    double detLag = msecsSince(data->timeLastDet);
    if (detLag > 500) color = 0xFF0000FF;
    else if (detLag > 100) color = 0xFFFF00FF;
    else  color = 0x00FF00FF;
    double MAX_LAG_LENGTH = config->camPixX;
    log->log(
        "image/detection_lag",
        rerun::LineStrips2D(rerun::LineStrip2D({{10., -150.}, {10. + min(detLag, MAX_LAG_LENGTH), -150.}}))
            .with_colors(color)
            .with_radii(2.0)
            .with_draw_order(10)
            .with_labels({format("Det Lag %.0fms", detLag)})
    );
    log->log(
        "performance/detection_lag_timeseries",
        rerun::Scalar(detLag)
    );

    // log fieldline detection delay
    double lineLag = msecsSince(data->timeLastLineDet);
    if (lineLag > 500) color = 0xFF0000FF;
    else if (lineLag > 100) color = 0xFFFF00FF;
    else  color = 0x00FF00FF;

    log->log(
        "image/fieldline_detection_lag",
        rerun::LineStrips2D(rerun::LineStrip2D({{10., -100.}, {10. + min(lineLag, MAX_LAG_LENGTH), -100.}}))
            .with_colors(color)
            .with_radii(2.0)
            .with_draw_order(10)
            .with_labels({format("Line Det Lag %.0fms", lineLag)})
    );
    log->log(
        "performance/fieldline_detection_lag_timeseries",
        rerun::Scalar(lineLag)
    );

    // log game control delay
    double gcLag = msecsSince(data->timeLastGamecontrolMsg);
    if (gcLag > 5000) color = 0xFF0000FF;
    else if (gcLag > 1000) color = 0xFFFF00FF;
    else color = 0x00FF00FF;

    log->log(
        "image/gamecontrol_lag",
            rerun::LineStrips2D(rerun::LineStrip2D({{10., -50.}, {10. + min(gcLag, MAX_LAG_LENGTH), -50.}}))
            .with_colors(color)
            .with_radii(2.0)
            .with_draw_order(10)
            .with_labels({format("GC Lag %.0fms", gcLag)})
    );
    log->log(
        "performance/gamecontrol_lag_timeseries",
        rerun::Scalar(gcLag)
    );
}

void Brain::statusReport() {
    if (!config->soundEnable || config->soundPack != "espeak") return;

    log->setTimeNow();
    static int reportInterval = 100;
    static string lastReport = "";
    string report;
    bool camOK = msecsSince(data->timeLastDet) < 1000;
    bool gcOK = msecsSince(data->timeLastGamecontrolMsg) < 1000;

    if (camOK && gcOK) {
        report = "Team" + to_string(config->teamId) + " Player " + to_string(config->playerId) + " " + tree->getEntry<string>("player_role") + " " + " OK";
    } else {
        report = "";
        if (!camOK) report += "camera lost";
        if (!gcOK) report += "gamecontrol lost";
    }
    if (lastReport != report) {
        speak(report);
        lastReport = report;
    }
}

void Brain::logStatusToConsole() {
    // 静态信息只在启动时打印一次
    static bool staticInfoPrinted = false;
    if (!staticInfoPrinted) {
        string staticMsg = format(
            "ROBOT:\n\tTeamID: %d\tPlayerID: %d\tNumberOfPlayers: %d\tRole: %s\tStartRole: %s\n\n"
            "DEBUG_CONFIG:\n\tcom: %s\tlogFile: %s\tlogTCP: %s\n\tvxFactor: %.2f\tyawOffset: %.2f",
            config->teamId,
            config->playerId,
            config->numOfPlayers,
            tree->getEntry<string>("player_role").c_str(),
            config->playerRole.c_str(),
            config->enableCom ? "YES" : "NO",
            config->rerunLogEnableFile ? "YES" : "NO",
            config->rerunLogEnableTCP ? "YES" : "NO",
            config->vxFactor,
            config->yawOffset
        );
        prtDebug(staticMsg);
        staticInfoPrinted = true;
    }

    // 动态状态信息每 30 个 tick 打印一次，只打印变化的量
    static int cnt = 0;
    const int LOG_INTERVAL = 30;

    // 上次打印的状态，用于比较是否有变化
    static string lastGameState = "";
    static string lastGameSubType = "";
    static string lastGameSubState = "";
    static int lastScore = -1;
    static int lastOppoScore = -1;
    static int lastLiveCount = -1;
    static int lastOppoLiveCount = -1;
    static bool lastPrimary = false;
    static int lastCmdId = -1;
    static double lastCost = -1;
    static bool lastImLead = false;
    static int lastCtrlState = -1;

    cnt++;
    if (cnt % LOG_INTERVAL == 0) {
        string gameState = tree->getEntry<string>("gc_game_state");
        gameState = gameState == "" ? "-----" : gameState;
        string gameSubType = tree->getEntry<string>("gc_game_sub_state_type");
        gameSubType = gameSubType == "" ? "-----" : gameSubType;
        string gameSubState = tree->getEntry<string>("gc_game_sub_state");
        gameSubState = gameSubState == "" ? "-----" : gameSubState;

        int score = data->score;
        int oppoScore = data->oppoScore;
        int liveCount = data->liveCount;
        int oppoLiveCount = data->oppoLiveCount;
        bool primary = isPrimaryStriker();
        int cmdId = data->tmCmdId;
        double cost = data->tmMyCost;
        bool imLead = data->tmImLead;
        int ctrlState = tree->getEntry<int>("control_state");
        int tickTime = (int)msecsSince(data->lastTick);

        // 检查是否有变化
        bool hasChange = (gameState != lastGameState) ||
                         (gameSubType != lastGameSubType) ||
                         (gameSubState != lastGameSubState) ||
                         (score != lastScore) ||
                         (oppoScore != lastOppoScore) ||
                         (liveCount != lastLiveCount) ||
                         (oppoLiveCount != lastOppoLiveCount) ||
                         (primary != lastPrimary) ||
                         (cmdId != lastCmdId) ||
                         (imLead != lastImLead) ||
                         (ctrlState != lastCtrlState);

        // 成本变化超过阈值也算变化
        if (!hasChange && fabs(cost - lastCost) > 0.05) hasChange = true;

        if (hasChange) {
            vector<string> changes;

            // 游戏状态变化
            string gameInfo = "";
            if (gameState != lastGameState) gameInfo += format("State=%s ", gameState.c_str());
            if (score != lastScore || oppoScore != lastOppoScore) gameInfo += format("Score=%d:%d ", score, oppoScore);
            if (liveCount != lastLiveCount || oppoLiveCount != lastOppoLiveCount) gameInfo += format("Live=%d:%d ", liveCount, oppoLiveCount);
            if (primary != lastPrimary) gameInfo += format("Primary=%s ", primary ? "YES" : "NO");
            if (!gameInfo.empty()) changes.push_back("GAME: " + gameInfo);

            // 子状态变化
            string subInfo = "";
            if (gameSubType != lastGameSubType) subInfo += format("SubType=%s ", gameSubType.c_str());
            if (gameSubState != lastGameSubState) subInfo += format("SubState=%s ", gameSubState.c_str());
            if (!subInfo.empty()) changes.push_back("SUB: " + subInfo);

            // 通信状态变化
            string comInfo = format("Cost=%.1f Lead=%s CmdId=%d", cost, imLead ? "YES" : "NO", cmdId);
            changes.push_back("COM: " + comInfo);

            // 控制状态变化
            if (ctrlState != lastCtrlState) changes.push_back(format("CtrlState=%d", ctrlState));

            // Tick 时间始终打印
            changes.push_back(format("Tick=%dms", tickTime));

            // 组装消息
            string msg = "";
            for (size_t i = 0; i < changes.size(); i++) {
                if (i > 0) msg += "  |  ";
                msg += changes[i];
            }
            prtDebug(msg);
        }

        // 更新上次状态
        lastGameState = gameState;
        lastGameSubType = gameSubType;
        lastGameSubState = gameSubState;
        lastScore = score;
        lastOppoScore = oppoScore;
        lastLiveCount = liveCount;
        lastOppoLiveCount = oppoLiveCount;
        lastPrimary = primary;
        lastCmdId = cmdId;
        lastCost = cost;
        lastImLead = imLead;
        lastCtrlState = ctrlState;
    }
    data->lastTick = get_clock()->now();
}

string Brain::getComLogStringBrief() {
    // 简洁版通信日志，只输出关键信息
    stringstream ss;
    int aliveCnt = 0;
    int selfIdx = config->playerId - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;
        if (data->tmStatus[i].isAlive) aliveCnt += 1;
    }
    ss << format("Alive: %d/%d  Cost: %.1f  Lead: %s  LastCmdChange: %.0fms ago",
        aliveCnt,
        config->numOfPlayers - 1,
        data->tmMyCost,
        data->tmImLead ? "YES" : "NO",
        msecsSince(data->tmLastCmdChangeTime)
    );
    return ss.str();
}

string Brain::getComLogString() {
    stringstream ss;
    int onFieldCnt = 0;
    int aliveCnt = 0;
    int selfIdx = config->playerId - 1;
    vector<int> onFieldIdxs = {};
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;

        if (data->penalty[i] == PENALTY_NONE) {
            onFieldCnt += 1;
            onFieldIdxs.push_back(i);
        }

        if (data->tmStatus[i].isAlive) aliveCnt += 1;
    }
    ss << CYAN_CODE << "COM: " << "\n";
    ss << "Teammates: OnField: " << onFieldCnt << "[";
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        ss << " P" << idx + 1 << " ";
    }
    ss << "]";
    ss << "  Alive: " << aliveCnt << "  TMCMDID: " << data->tmCmdId << "  ReceivedDMD: " << data->tmReceivedCmd << "\n";

    // Self info
    ss << "Self\tCost: " << format("%.1f", data->tmMyCost) << "\tLead: ";
    if (data->tmImLead)
        ss << GREEN_CODE << "YES" << CYAN_CODE;
    else
        ss << RED_CODE << "NO" << CYAN_CODE;
    ss << "    TMCMD: " << data->tmCmdId << format("\tCMD: [%d]%d", data->tmMyCmdId, data->tmMyCmd);
    ss << "\n";

    // Teammates info
    for (int i = 0; i < onFieldIdxs.size(); i++) {
        int idx = onFieldIdxs[i];
        auto status = data->tmStatus[idx];
        ss << "P" << idx + 1 << "[";
        if (status.isAlive)
            ss << GREEN_CODE << "★" << CYAN_CODE;
        else 
            ss << RED_CODE << "☆" << CYAN_CODE;
        ss << "]\tCost: " << format("%.1f", status.cost);
        ss << "\tLead: ";
        if (status.isLead)
            ss << GREEN_CODE << "YES" << CYAN_CODE;
        else
            ss << RED_CODE << "NO" << CYAN_CODE;
        ss << "\tCMD: " << format("[%d]%d", status.cmdId, status.cmd);
        ss << "\tLag: " << format("%.0f", msecsSince(status.timeLastCom)) << "ms" << "\n";
    }
    ss << "\n";
    
    return ss.str();
}

bool Brain::isFreekickStartPlacing() {
    return (tree->getEntry<string>("gc_game_sub_state_type") == "FREE_KICK" && tree->getEntry<string>("gc_game_state") == "PLAY" && tree->getEntry<string>("gc_game_sub_state") == "GET_READY");
}

void Brain::playSoundForFun() {
    string soundPack = config->soundPack;
    if (config->soundEnable && soundPack != "espeak") {
        static string gcGameState_last;
        string gcGameState = tree->getEntry<string>("gc_game_state");

        static bool gameStarted = false;
        if (gcGameState == "PLAY") gameStarted = true;
        if (gcGameState == "READY")
        {
            if (!gameStarted) playSound(soundPack + "-ready", 2000);
            else if (tree->getEntry<bool>("we_just_scored")) playSound(soundPack + "-celebrate", 2000);
            else playSound(soundPack + "-regret", 2000);   
        }

        if (gcGameState == "PLAY") {
            auto decision = tree->getEntry<string>("decision"); 
            if (decision == "chase") playSound(soundPack + "-chase", 5000);
            else if (decision == "adjust") playSound(soundPack + "-adjust", 2000);
            else if (decision == "kick") playSound(soundPack + "-kick", 2000);
        }
    }
}

// =============================================================================
//  请将以下代码追加到 src/brain/src/brain.cpp 的最末尾 (在最后的 } 之后或者之前均可，确保在 namespace 外或类定义外)
// =============================================================================

/**
 * 计算当前向 dir 方向踢球的价值的大小.
 */
double Brain::kickValue(double dir)
{
    // 简单的价值评估：如果是朝向对方球门方向，价值高
    // 这里是一个基础实现，你可以根据策略需要修改
    auto fd = config->fieldDimensions;
    double goalDir = atan2(0.0 - data->robotPoseToField.y, fd.length/2.0 - data->robotPoseToField.x);
    double diff = fabs(toPInPI(dir - goalDir));
    
    // 角度偏差越小，价值越高。范围 [0, 1]
    return max(0.0, 1.0 - diff / M_PI); 
}

/**
 * 计算当前的急迫度
 * 0: 安全, 1: 有威胁, 2: 危险
 */
double Brain::threatLevel()
{
    // 基础实现：如果有敌人在附近(2米内)，则认为有威胁
    double minOpponentDist = 100.0;
    auto robots = data->getRobots();
    for(const auto& r : robots) {
        if(r.label == "Opponent") {
            double dist = r.range;
            if(dist < minOpponentDist) minOpponentDist = dist;
        }
    }

    if (minOpponentDist < 0.8) return 2.0;  // 危险：敌人很近
    if (minOpponentDist < 1.5) return 1.0;  // 有威胁：敌人在 1.5m 内
    return 0.0;                              // 安全：敌人在 1.5m 外
}

/**
 * 判断是否适合定向踢球
 */
bool Brain::isAngleGoodForDirectionalKick(double goalPostMargin)
{
    // 复用通用的角度判断逻辑，或者添加特定逻辑
    return isAngleGood(goalPostMargin, "kick");
}

/**
 * 检查前方扇形区域是否有障碍物
 */
bool Brain::isFrontRangeClear(double startAngle, double endAngle, double safeDist, double step)
{
    // 遍历角度范围，检查每个角度上的最近障碍物距离
    for (double ang = startAngle; ang <= endAngle; ang += step) {
        if (distToObstacle(ang) < safeDist) {
            return false; // 发现障碍物，不空闲
        }
    }
    return true; // 区域内无障碍物
}

/**
 * 发布视觉校准参数
 */
/**
 * 发布视觉校准参数
 */
void Brain::pubCalParamMsg(double pitch, double yaw, double z)
{
    if (pubCalParam) {
        vision_interface::msg::CalParam msg;
        msg.pitch_compensation = pitch;
        msg.yaw_compensation = yaw;
        msg.z_compensation = z;
        
        pubCalParam->publish(msg);
    }
}
