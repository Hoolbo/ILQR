function[] = arguments()
global arg 
%% 障碍函数约束参数
arg.is_cal_obs_cost = 1;
arg.is_cal_lane_cost = 1;
arg.is_cal_steer_cost = 1;
arg.steer_max_q1 = 1;
arg.steer_max_q2 = 1;
arg.steer_min_q1 = 1;
arg.steer_min_q2 = 1;
arg.lane_q1 = 1;
arg.lane_q2 = 1;
arg.obs_q1 = 1;
arg.obs_q2 = 1;
%% Q状态代价矩阵 | R控制代价矩阵
arg.Q = [1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0, 0, 0;
         0, 0, 0, 0.5 ];
arg.R = [0.5, 0;
         0, 3 ];
%%仿真参数
arg.dt = 0.1;
arg.tf = 1000;
arg.num_states = 4;
arg.num_ctrl = 2;
%% 地图参数
load('map_info');
map_start_index = 300;
map_end_index = 1200;
% map_end_index = length(xcoord);
arg.trace_width = 6;
arg.trace_safe_width_left = 3;
arg.trace_safe_width_right = 3;
arg.xcoord  = xcoord(map_start_index:map_end_index);
arg.ycoord  = ycoord(map_start_index:map_end_index);
arg.slength = slength(map_start_index:map_end_index);
arg.Curv    = Curv(map_start_index:map_end_index);
arg.theta   = theta(map_start_index:map_end_index);

%% ilqr参数
arg.N = 40; %Horizon
arg.rel_tol = 1e-3;
arg.tol = 1e-3;
arg.max_iter = 200;
arg.lamb_factor = 1.2;
arg.lamb_init = 5;
arg.lamb_max = 2000;
arg.totalBarrierCost = 0;
arg.startSpeed = 10;
arg.desireSpeed = 10;
arg.preX = zeros(arg.N+1,arg.num_states) * nan;
arg.preU = zeros(arg.N,arg.num_ctrl) * nan;
arg.preX_nan = zeros(arg.N+1,arg.num_states) * nan;
arg.preU_nan = zeros(arg.N,arg.num_ctrl) * nan;
arg.error_count = 0;

     
%%障碍物参数

arg.obs_x = [270,340,380,420,460];   % 障碍物 x 坐标
arg.obs_y = [1.24,0.5,2,1.2,-4];   % 障碍物 y 坐标
arg.obs_radius = [1,1,1,1,1]; % 障碍物半径
% arg.obs_x = [320];   % 障碍物 x 坐标
% arg.obs_y = [3];   % 障碍物 y 坐标
% arg.obs_radius = [1]; % 障碍物半径
arg.obs_dx = 0.3;

%%车辆几何参数
arg.ego_rad = 4;
arg.lf = 1.597;
arg.lr = 1.133;
arg.l = 2.73;
arg.width = 1.5;
arg.steer_angle_max = 1;
arg.steer_angle_min = -1;

%%纯跟踪参数
arg.Kv = 0.3; %前视距离系数
arg.Kp = 0.8; %速度P控制器系数
arg.Ld0 = 3;  %Ld0是预瞄距离的下限值
arg.Ld_min = 3;
arg.Ld_max = 20;
end