function arg = arguments()
arg = struct();
%% 障碍函数约束参数
arg.is_cal_obs_cost = 1;
arg.is_cal_lane_cost = 1;
arg.is_cal_steer_cost = 1;
arg.steer_max_q1 = 1;
arg.steer_max_q2 = 1;
arg.steer_min_q1 = 1;
arg.steer_min_q2 = 1;
arg.lane_q1 = 2;
arg.lane_q2 = 1;
arg.obs_q1 = 1;
arg.obs_q2 = 1;
%% Q状态代价矩阵 | R控制代价矩阵
arg.Q = [1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0, 0, 0;
         0, 0, 0, 1 ];
arg.R = [0.5, 0;
         0, 2 ];

%%仿真参数
arg.dt = 0.1;
arg.tf = 1000;
arg.num_states = 4;
arg.num_ctrl = 2;
%% 地图参数
load('map_info');
% load('map_info2');
map_start_index = 300;
map_end_index = 1800;
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
arg.N = 50; %Horizon
arg.rel_tol = 1e-3;
arg.tol = 1e-3;
arg.max_iter = 200;
arg.lamb_factor = 1.25;
arg.lamb_init = 10;
arg.lamb_max = 2000;
arg.totalBarrierCost = 0;
arg.startSpeed = 0;
arg.desireSpeed = 5;
arg.preX = zeros(arg.N+1,arg.num_states) * nan;
arg.preU = zeros(arg.N,arg.num_ctrl) * nan;
arg.preX_nan = zeros(arg.N+1,arg.num_states) * nan;
arg.preU_nan = zeros(arg.N,arg.num_ctrl) * nan;

%%障碍物参数
arg.obs_num = 6;
arg.obs_radius = 1;
arg.obs_traj = zeros(arg.N+1, arg.obs_num, 4);
%% state x y theta v
arg.obs_traj(1,1,:) = [300,0,0,0.1];
arg.obs_traj(1,2,:) = [250,3,0,-0.2];
arg.obs_traj(1,3,:) = [330,3,0,0];
arg.obs_traj(1,4,:) = [400,-1,0,0];
arg.obs_traj(1,5,:) = [420,1,0,0];
arg.obs_traj(1,6,:) = [600,82,0,0];
for i = 2 : arg.N + 1
    for j = 1 : arg.obs_num
        arg.obs_traj(i,j,1) = arg.obs_traj(i-1,j,1) +arg.obs_traj(i-1,j,4) * cos(arg.obs_traj(i-1,j,3)) ;
        arg.obs_traj(i,j,2) = arg.obs_traj(i-1,j,2) +arg.obs_traj(i-1,j,4) * sin(arg.obs_traj(i-1,j,3)) ;
        arg.obs_traj(i,j,3) = arg.obs_traj(i-1,j,3);
        arg.obs_traj(i,j,4) = arg.obs_traj(i-1,j,4);
    end
end

%%车辆参数
arg.ego_rad = 3;
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