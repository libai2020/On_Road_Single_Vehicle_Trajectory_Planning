%  MATLAB Source Codes for the book "Cooperative Decision and Planning for
%  Connected and Automated Vehicles" published by Mechanical Industry Press
%  in 2020. 《智能网联汽车协同决策与规划技术》书籍配套代码
%  Copyright (C) 2020 Bai Li
%  2020.02.14
% ==============================================================================
%  第四章 4.3节. 结构化道路单一车辆轨迹优化方法
% ==============================================================================
%  备注：
%  1. 求解成功后会动态显示规划的轨迹
%  2. 求解成功率低，可能的原因是局部隧道建设方式浪费了一定可用的空间，另外道路宽度设置有些窄..
% ==============================================================================
clear all; close all; clc
% % 参数设置
global vehicle_geometrics_ % 车辆轮廓几何尺寸
vehicle_geometrics_.vehicle_wheelbase = 2.8;
vehicle_geometrics_.vehicle_front_hang = 0.96;
vehicle_geometrics_.vehicle_rear_hang = 0.929;
vehicle_geometrics_.vehicle_width = 1.942;
vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase + ...
    vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_rear_hang;
vehicle_geometrics_.radius = hypot(0.25 * vehicle_geometrics_.vehicle_length, 0.5 * vehicle_geometrics_.vehicle_width);
vehicle_geometrics_.r2x = 0.25 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;
vehicle_geometrics_.f2x = 0.75 * vehicle_geometrics_.vehicle_length - vehicle_geometrics_.vehicle_rear_hang;
global vehicle_kinematics_ % 车辆运动能力参数
vehicle_kinematics_.vehicle_v_max = 20.0;
vehicle_kinematics_.vehicle_a_max = 5.0;
vehicle_kinematics_.vehicle_phy_max = 0.7;
vehicle_kinematics_.vehicle_w_max = 1.0;
global dp_ % 搜索空间参数
dp_.num_t_grids = 5;
dp_.num_s_grids = 7;
dp_.num_l_grids = 8;
dp_.unit_time = 2.0;
dp_.max_unit_s = dp_.unit_time * vehicle_kinematics_.vehicle_v_max;
dp_.min_unit_s = 0;
dp_.ds = linspace(dp_.min_unit_s, dp_.max_unit_s, dp_.num_s_grids);
dp_.dl = linspace(0, 1, dp_.num_l_grids);

global obstacles_ Nobs precise_timeline precise_timeline_index % 障碍物随机生成
Nobs = 2;
delta_t_precise = 0.05;
precise_timeline = [0 : delta_t_precise : (dp_.unit_time * dp_.num_t_grids)];
precise_timeline_index = cell(1,dp_.num_t_grids);
ind = round(linspace(1, length(precise_timeline), dp_.num_t_grids + 1));
for ii = 1 : dp_.num_t_grids
    elem.ind1 = ind(ii); elem.ind2 = ind(ii+1);
    precise_timeline_index{1,ii} = elem;
end
obstacles_ = GenerateObstacles(0.5 * vehicle_kinematics_.vehicle_v_max);

global BV_ % 边值
BV_.s0 = 0;
BV_.l0 = 0.78;
[BV_.x0, BV_.y0, BV_.theta0] = ConvertFrenetToCartesian(BV_.s0, BV_.l0);
BV_.v0 = 20;
BV_.phy0 = 0.18;
global road_barriers_ % 道路边界散点
road_barriers_ = GenerateRoadBarrierGrids();

% % DP搜索中的参数设置
dp_.Ncollision = 10000;
dp_.w_relative = 1.0;
dp_.w1 = 1.0;
dp_.w2 = 1.0;
dp_.w3 = 10.0;
% % 决策轨迹
tic; [x, y, theta] = SearchDecisionTrajectoryViaDp(); toc

% % 轨迹规划部分
gamma_rate = 0.8; % 只保留决策轨迹的前cutting_rate * 100 %时域，并仅在该时域上做轨迹规划
tf = dp_.unit_time * dp_.num_t_grids * gamma_rate;
temp = abs(precise_timeline - tf);
ind_end = find(temp == min(temp)); ind_end = ind_end(1);
Nfe = ind_end; x = x(1 : Nfe); y = y(1 : Nfe); theta = theta(1 : Nfe);
[v, a, phy, w] = FormInitialGuess(x, y, theta, tf);
tic; [~, ~, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta); toc;
WriteInitialGuessForFirstTimeNLP(x, y, theta, xr, yr, xf, yf, v, a, phy, w);
WriteParameters(tf, Nfe);
WriteGuidingLineFile(x, y, theta);
!ampl rr.run
[cur_x, cur_y, cur_theta] = LoadStates();
SpecifyLocalBoxes(cur_x, cur_y, cur_theta);
!ampl rr2.run

if (exist('opti_flag.txt','file'))
    load opti_flag.txt
    if (opti_flag)
        [cur_x, cur_y, cur_theta] = LoadStates();
        Dynamics();
    end
end