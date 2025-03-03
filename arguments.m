function[] = arguments()
global arg 
%障碍物参数
arg.obs_x = 301;   % 障碍物 x 坐标
arg.obs_y = 0.5;   % 障碍物 y 坐标

arg.obs_x2 = 310;
arg.obs_y2 = 2;
arg.obs_radius = 1; % 障碍物半径
arg.t_safe = 1;   % 安全时间
arg.Q_obs=1;
arg.R_obs=1;
arg.w_obs = 10;

%车辆几何参数
arg.ego_rad = 3;
arg.lf = 1.597;
arg.lr = 1.133;
arg.l = 2.73;
arg.width = 1.5;
arg.steer_angle_max = 1;
arg.steer_angle_min = -1;

%仿真参数
arg.dt = 0.1;
arg.tf = 1000;
arg.num_states = 4;
arg.num_ctrl = 2;

%地图参数
load('map_info');
arg.xcoord  = xcoord;
arg.ycoord  = ycoord;
arg.slength = slength;
arg.Curv    = Curv;
arg.theta   = theta;

%纯跟踪参数
arg.Kv = 0.3; %前视距离系数
arg.Kp = 0.8; %速度P控制器系数
arg.Ld0 = 3;  %Ld0是预瞄距离的下限值
arg.Ld_min = 3;
arg.Ld_max = 20;



%ilqr参数
arg.N = 20; %Horizon
arg.rel_tol = 1e-3;
arg.tol = 1e-2;
arg.max_iter = 200;
arg.lamb_factor = 1.5;
arg.lamb_init = 5;
arg.lamb_max = 2000;
% arg.preU = zeros(arg.N,arg.num_ctrl) * nan;
% arg.preX = zeros(arg.N + 1,arg.num_states) * nan;
arg.totalBarrierCost = 0;

arg.startSpeed = 0;
arg.desireSpeed = 10;
arg.steer_max_q1 = 20;
arg.steer_max_q2 = 5;
arg.steer_min_q1 = 20;
arg.steer_min_q2 = 5;
arg.obs_q1 = 1;
arg.obs_q2 = 1;
arg.Q = [2, 0, 0, 0;
         0, 2, 0, 0;
         0, 0, 1, 0;
         0, 0, 0, 1 ];

arg.R = [2, 0;
         0, 4 ];

end