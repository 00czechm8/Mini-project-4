clear
%% Constants
x_lim = 10;
y_lim = 10;
res = 2; %number of decimals
num_obs = 10;
collision_radius = 0.5;

%% Set up playing field
%creates random obstacles
obs = obstacle_field(x_lim, y_lim, res, num_obs);
%creates binary occupancy map and populates obstacles
map = binaryOccupancyMap(y_lim, x_lim, 10^(res));
setOccupancy(map, [obs.locs(:,1), obs.locs(:,2)], ones(10,1));

%inflate obstacles to safe size so robots stay clear
inflate(map, collision_radius);

%% Run Offline Planner
startPose = [0 0 pi/2];
goalPose = [9.9 9.9 pi/4];

%create validator map
validator = validatorOccupancyMap;
validator.Map = map;

%declare and run planner with nonholonomic constraints
hybridPlanner = plannerHybridAStar(validator,MinTurningRadius=1,MotionPrimitiveLength=1);
refpath = plan(hybridPlanner,startPose,goalPose);

%show plan
show(hybridPlanner)


%% Me playing with formation things

% th = [pi/2; pi; 3*pi/2];
% r = [1;2;3];
% 
% x_0 = [0 0;
%        5 0;
%        10 5
%        15 0];
% T = size(x_0,2);
% 
% gamma = 0
% th_t
% for t = 1:T-2
%     for i = 1:3
%         gamma = atan2(x_0(t+2,2)-x_0(t+1,2), x_0(t+2,1)-x_0(t+1,1))-gamma
%         th_t = atan2(x_0(t+1,2)-x_0(t,2), x_0(t+1,1)-x_0(t,1))
% 
%         x_i(i,1,t) = r(i)*cos(th(i)+gamma)+r_t*cos(th_t)+x_0(t,1)
%         x_i(i,2,t) = r(i)*sin(th(i)+gamma)+r_t*sin(th_t)+x_0(t,2)
%     end
% end