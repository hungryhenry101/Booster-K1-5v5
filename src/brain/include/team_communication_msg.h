#pragma once

#include "types.h"

#define VALIDATION_COMMUNICATION 31202
#define VALIDATION_DISCOVERY 41203
struct TeamCommunicationMsg
{
    int validation = VALIDATION_COMMUNICATION; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
    int playerRole; // 1: striker, 2: goal_keeper, 3: unknown
    bool isAlive; // 是否在场上, 且没有在罚时中
    bool isLead; // 是否在控球状态
    bool ballDetected;
    bool ballLocationKnown;
    double ballConfidence;
    double ballRange;
    double cost; // 计算从当前状态到能踢到球的成本
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir;
    double thetaRb;
    int cmdId; // 每个 player 发布时, 需要将 cmdId + 1. 用来代表发布的顺序. 
    int cmd; // 百位为 1 时, 代表自己要球控球. 十位为 1 时, 代表守门员要求另一个球员接替守门员角色, 此时个位数字代表接替球员的 playerId. 例如: 100, 代表自己要球控球, 另一个 striker 进入辅助角色; 011, 代表守门员要出击, 要求 1 号球员接替守门.  
};

struct TeamDiscoveryMsg
{
    int validation = VALIDATION_DISCOVERY; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
};
