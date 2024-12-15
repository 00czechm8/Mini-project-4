clear
%% Constants
x_lim = 10;
y_lim = 10;
res = 2; %number of decimals
num_obs = 5;
collision_radius = 0.5;

%% Set up playing field
%creates random obstacles
obs = obstacle_field(x_lim, y_lim, res, num_obs);
%creates binary occupancy map and populates obstacles
map = binaryOccupancyMap(y_lim, x_lim, 10^(res));
setOccupancy(map, [obs.locs(:,1), obs.locs(:,2)], ones(num_obs,1));

%inflate obstacles to safe size so robots stay clear
inflate(map, collision_radius);

%% Run Offline Planner
startPose = [0 0 0];
goalPose = [9.9 9.9 pi/4];

%create validator map
validator = validatorOccupancyMap;
validator.Map = map;

%declare and run planner with nonholonomic constraints
hybridPlanner = plannerHybridAStar(validator,MinTurningRadius=1,MotionPrimitiveLength=1);
refpath = plan(hybridPlanner,startPose,goalPose);

%show plan
% show(hybridPlanner)


%% Move Swarm with Collision Avoidance
g = eye(2);
f = zeros(2);
theta_star = [0.656; 2.7; 4; 5.8];
r_star = [0.5; 1; 2; 1];
gamma = startPose(3);
x_0 = startPose(1:2);
r_si = [0.25; 0.25; 0.25; 0.25];

N = length(r_star);
drone_pos = zeros(N, 2);
for drone = 1:length(theta_star)
    drone_pos(drone,:) = r_star(drone)*[cos(theta_star(drone)+gamma) sin(theta_star(drone)+gamma)]+x_0;
end
hold on
plot(x_0(1), x_0(2),  "o", LineWidth=2)
plot([drone_pos(:,1);drone_pos(1,1)], [drone_pos(:,2);drone_pos(1,2)], "r-o",LineWidth=2)
plt(obs)
hold off
xlim([-1 10])
ylim([-1 10])
pause(1)
clf;


FormCBF = ZCBF(g, f, r_si, drone_pos, r_star, theta_star, gamma, x_0);
FormCBF = FormCBF.update_obs(0.5*ones(length(obs.locs),1), drone_pos, obs.locs, gamma, x_0);

for n = 2:size(refpath.States,1)
    x_0 = refpath.States(n, 1:2);
    gamma = refpath.States(n,3);
    FormCBF = FormCBF.update_obs(0.5*ones(length(obs.locs),1), drone_pos, obs.locs, gamma, x_0);
    for drone = 1:N
        drone_pos(drone, :) = FormCBF.QP(drone);
        FormCBF = FormCBF.update_obs(0.5*ones(length(obs.locs),1), drone_pos, obs.locs, gamma, x_0);
    end
    hold on
    plot(x_0(1), x_0(2),  "o", LineWidth=2)
    plot([drone_pos(:,1);drone_pos(1,1)], [drone_pos(:,2);drone_pos(1,2)], "r-o",LineWidth=2)
    plt(obs)
    hold off
    xlim([-1 10])
    ylim([-1 10])
    pause(1)
    clf;
end