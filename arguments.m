function[] = arguments()
global arg 
%%仿真参数
arg.dt = 0.1;
arg.tf = 1000;
arg.num_states = 4;
arg.num_ctrl = 2;
%% 地图参数
load('map_info');
map_start_index = 500;
map_end_index = 1500;
% map_end_index = length(xcoord);
arg.xcoord  = xcoord(map_start_index:map_end_index);
arg.ycoord  = ycoord(map_start_index:map_end_index);
arg.slength = slength(map_start_index:map_end_index);
arg.Curv    = Curv(map_start_index:map_end_index);
arg.theta   = theta(map_start_index:map_end_index);

%% ilqr参数
arg.N = 30; %Horizon
arg.rel_tol = 1e-3;
arg.tol = 1e-2;
arg.max_iter = 200;
arg.lamb_factor = 1.5;
arg.lamb_init = 5;
arg.lamb_max = 2000;
arg.totalBarrierCost = 0;
arg.startSpeed = 0;
arg.desireSpeed = 5;
arg.preX = zeros(arg.N+1,arg.num_states) * nan;
arg.preU = zeros(arg.N,arg.num_ctrl) * nan;
%% 障碍函数约束参数
arg.steer_max_q1 = 20;
arg.steer_max_q2 = 5;
arg.steer_min_q1 = 20;
arg.steer_min_q2 = 5;
arg.obs_q1 = 1;
arg.obs_q2 = 1;
%% Q状态代价矩阵 | R控制代价矩阵
arg.Q = [2, 0, 0, 0;
         0, 2, 0, 0;
         0, 0, 0, 0;
         0, 0, 0, 0.5 ];
arg.R = [2, 0;
         0, 6 ];
     
%%障碍物参数
% arg.obs_x = [300,340,380,420,460];   % 障碍物 x 坐标
% arg.obs_y = [2,0.5,1,1.2,1.6];   % 障碍物 y 坐标
% arg.obs_radius = [1,1,1,1,1]; % 障碍物半径
arg.obs_x = [300];   % 障碍物 x 坐标
arg.obs_y = [1];   % 障碍物 y 坐标
arg.obs_radius = [1.5]; % 障碍物半径


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