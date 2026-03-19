#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include "game_controller_node.h"

GameControllerNode::GameControllerNode(string name) : rclcpp::Node(name)
{
    _socket = -1;

    // 声明 Ros2 参数，注意在配置文件中新加的参数需要在这里显示声明
    declare_parameter<int>("port", 3838);
    declare_parameter<bool>("enable_ip_white_list", false);
    declare_parameter<vector<string>>("ip_white_list", vector<string>{});

    // 从配置中读取参数，注意把读取到的参数打印到日志中方便查问题
    get_parameter("port", _port);
    RCLCPP_INFO(get_logger(), "[get_parameter] port: %d", _port);
    get_parameter("enable_ip_white_list", _enable_ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] enable_ip_white_list: %d", _enable_ip_white_list);
    get_parameter("ip_white_list", _ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] ip_white_list(len=%ld)", _ip_white_list.size());
    for (size_t i = 0; i < _ip_white_list.size(); i++)
    {
        RCLCPP_INFO(get_logger(), "[get_parameter]     --[%ld]: %s", i, _ip_white_list[i].c_str());
    }

    // 创建 publisher，发布到 /game_state
    _publisher = create_publisher<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 10);
}

GameControllerNode::~GameControllerNode()
{
    if (_socket >= 0)
    {
        // 关闭打开的文件描述符是个好习惯
        close(_socket);
    }

    if (_thread.joinable())
    {
        _thread.join();
    }
}

/**
 * 创建 Soket 并绑定到指定的端口
 */
void GameControllerNode::init()
{
    // 创建 socket，失败了直接抛异常
    _socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(get_logger(), "socket failed: %s", strerror(errno));
        throw runtime_error(strerror(errno));
    }

    // 初始化地址
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    // INADDR_ANY 将监听本机所有网络接口，默认情况这样就可以
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_port);

    // 绑定地址，失败了就抛异常
    if (bind(_socket, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "bind failed: %s (port=%d)", strerror(errno), _port);
        throw runtime_error(strerror(errno));
    }

    // bind 成功后就可以开始从 socket 里接收数据了
    RCLCPP_INFO(get_logger(), "Listening for UDP broadcast on 0.0.0.0:%d", _port);

    // 启用一个新的线程来接收数据，主线程进入 Node 自身的 spin，处理一些 Node 自己的服务
    _thread = thread(&GameControllerNode::spin, this);
}

void GameControllerNode::spin()
{
    // 用来获取远程地址
    sockaddr_in remote_addr;
    socklen_t remote_addr_len = sizeof(remote_addr);

    // data 和 msg 在循环内是复用的，后续更新代码需要注意一下这个点
    HlRoboCupGameControlData data;
    game_controller_interface::msg::GameControlData msg;

    // 进入循环
    while (rclcpp::ok())
    {
        // 从 socket 中接收数据包，期望的是接收完整的数据包
        ssize_t ret = recvfrom(_socket, &data, sizeof(data), 0, (sockaddr *)&remote_addr, &remote_addr_len);
        if (ret < 0)
        {
            RCLCPP_ERROR(get_logger(), "receiving UDP message failed: %s", strerror(errno));
            continue;
        }

        // 获取远端 IP
        string remote_ip = inet_ntoa(remote_addr.sin_addr);

        // 接收到不完整的包或其它非法的包，忽略掉
        if (ret != sizeof(data))
        {
            RCLCPP_INFO(get_logger(), "packet from %s invalid length=%ld", remote_ip.c_str(), ret);
            continue;
        }

        if (data.version != HL_GAMECONTROLLER_STRUCT_VERSION)
        {
            RCLCPP_INFO(get_logger(), "packet from %s invalid version: %d", remote_ip.c_str(), data.version);
            continue;
        }

        // 过滤 IP 白名单
        if (!check_ip_white_list(remote_ip))
        {
            RCLCPP_INFO(get_logger(), "received packet from %s, but not in ip white list, ignore it", remote_ip.c_str());
            continue;
        }

        // 处理消息，把 data 数据 copy 到 msg
        handle_packet(data, msg);

        // 将消息发布到 Topic 中
        _publisher->publish(msg);

        RCLCPP_INFO(get_logger(), "handle packet successfully ip=%s, packet_number=%d", remote_ip.c_str(), data.packetNumber);
    }
}

/**
 * 检查 IP 是否在白名单里，如果未开启白名单或者在白名单里，返回 true，其它情况返回 false
 */
bool GameControllerNode::check_ip_white_list(string ip)
{
    // 没有开启或在白名单内，返回 true
    if (!_enable_ip_white_list)
    {
        return true;
    }
    for (size_t i = 0; i < _ip_white_list.size(); i++)
    {
        if (ip == _ip_white_list[i])
        {
            return true;
        }
    }
    return false;
}

/**
 * 将 UDP 数据格式转成自定交 Ros2 message 格式（逐字段复制）
 * 如需更改，一定要仔细各字段
 */
void GameControllerNode::handle_packet(HlRoboCupGameControlData &data, game_controller_interface::msg::GameControlData &msg)
{

    // header 是固定长度 4
    for (int i = 0; i < 4; i++)
    {
        msg.header[i] = data.header[i];
    }
    msg.version = data.version;
    msg.packet_number = data.packetNumber;
    msg.players_per_team = data.playersPerTeam;
    msg.game_type = data.gameType;
    msg.state = data.state;
    msg.first_half = data.firstHalf;
    msg.kick_off_team = data.kickOffTeam;
    msg.secondary_state = data.secondaryState;
    // secondary_state_info 是固定长度 4
    for (int i = 0; i < 4; i++)
    {
        msg.secondary_state_info[i] = data.secondaryStateInfo[i];
    }
    msg.drop_in_team = data.dropInTeam;
    msg.drop_in_time = data.dropInTime;
    msg.secs_remaining = data.secsRemaining;
    msg.secondary_time = data.secondaryTime;

    // RCLCPP_INFO(get_logger(), "-------------------- GameController Data -------------------------");
    
    // RCLCPP_INFO(get_logger(), "header: %02x %02x %02x %02x, "
    //             "version=%d, packet_number=%d, players_per_team=%d",
    //             msg.header[0], msg.header[1], msg.header[2], msg.header[3],
    //             msg.version, msg.packet_number, msg.players_per_team);
    // RCLCPP_INFO(get_logger(), "game_type=%d, state=%d, "
    //             "first_half=%d, kick_off_team=%d, secondary_state=%d, "
    //             "drop_in_team=%d, drop_in_time=%d, secs_remaining=%d, secondary_time=%d",
    //             msg.game_type, msg.state, msg.first_half,
    //             msg.kick_off_team, msg.secondary_state,
    //             msg.drop_in_team, msg.drop_in_time,
    //             msg.secs_remaining, msg.secondary_time);
    // RCLCPP_INFO(get_logger(), "secondary_state_info: %d %d %d %d",
    //             msg.secondary_state_info[0], msg.secondary_state_info[1],
    //             msg.secondary_state_info[2], msg.secondary_state_info[3]);

    // teams 是固定长度 2
    for (int i = 0; i < 2; i++)
    {
        msg.teams[i].team_number = data.teams[i].teamNumber;
        msg.teams[i].field_player_colour = data.teams[i].fieldPlayerColour;
        msg.teams[i].score = data.teams[i].score;
        msg.teams[i].penalty_shot = data.teams[i].penaltyShot;
        msg.teams[i].single_shots = data.teams[i].singleShots;
        msg.teams[i].coach_sequence = data.teams[i].coachSequence;
        // RCLCPP_INFO(get_logger(), "team[%d]: team_number=%d, field_player_colour=%d, score=%d, penalty_shot=%d, single_shots=%d, coach_sequence=%d",
        //             i, msg.teams[i].team_number, msg.teams[i].field_player_colour,
        //             msg.teams[i].score, msg.teams[i].penalty_shot,
        //             msg.teams[i].single_shots, msg.teams[i].coach_sequence);

        // msg.teams[i].players 定义为不定长的数组，注意跟定长数组有所区分
        int coach_message_len = sizeof(data.teams[i].coachMessage) / sizeof(data.teams[i].coachMessage[0]);
        msg.teams[i].coach_message.clear(); // 因为 msg 是利用的，切记这里要 clear()
        for (int j = 0; j < coach_message_len; j++)
        {
            msg.teams[i].coach_message.push_back(data.teams[i].coachMessage[j]);
        }

        // msg.teams[i].cocah
        msg.teams[i].coach.penalty = data.teams[i].coach.penalty;
        msg.teams[i].coach.secs_till_unpenalised = data.teams[i].coach.secsTillUnpenalised;
        msg.teams[i].coach.number_of_warnings = data.teams[i].coach.numberOfWarnings;
        msg.teams[i].coach.yellow_card_count = data.teams[i].coach.yellowCardCount;
        msg.teams[i].coach.red_card_count = data.teams[i].coach.redCardCount;
        msg.teams[i].coach.goal_keeper = data.teams[i].coach.goalKeeper;

        // msg.teams[i].coach_message 定义为不定长的数组，注意跟定长数组有所区分
        int players_len = sizeof(data.teams[i].players) / sizeof(data.teams[i].players[0]);
        msg.teams[i].players.clear(); // 因为 msg 是利用的，切记这里要 clear()
        for (int j = 0; j < players_len; j++)
        {
            game_controller_interface::msg::RobotInfo rf;
            rf.penalty = data.teams[i].players[j].penalty;
            rf.secs_till_unpenalised = data.teams[i].players[j].secsTillUnpenalised;
            rf.number_of_warnings = data.teams[i].players[j].numberOfWarnings;
            rf.yellow_card_count = data.teams[i].players[j].yellowCardCount;
            rf.red_card_count = data.teams[i].players[j].redCardCount;
            rf.goal_keeper = data.teams[i].players[j].goalKeeper;
            msg.teams[i].players.push_back(rf);
            // RCLCPP_INFO(get_logger(), "team[%d].player[%d]: penalty=%d, secs_till_unpenalised=%d, number_of_warnings=%d, yellow_card_count=%d, red_card_count=%d, goal_keeper=%d",
            //             i, j, rf.penalty, rf.secs_till_unpenalised,
            //             rf.number_of_warnings, rf.yellow_card_count,
            //             rf.red_card_count, rf.goal_keeper);
        }
    }
}