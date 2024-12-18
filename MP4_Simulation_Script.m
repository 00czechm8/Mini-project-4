clear
%% Distance Function Calc
disFun = @(x,y,p) ((p(1)-x).^2+(p(2)-y).^2).^.5;
%% Constants
x_lim = 14;
y_lim = 14;
res = 2; %number of decimals
num_obs = 8;
collision_radius = 0.75;

%% Set up playing field
%creates random obstacles
obs = obstacle_field(x_lim, y_lim, res, num_obs);
%creates binary occupancy map and populates obstacles
map = binaryOccupancyMap(y_lim, x_lim, 10^(res));
setOccupancy(map, [obs.locs(:,1), obs.locs(:,2)], ones(num_obs,1));

%inflate obstacles to safe size so robots stay clear
inflate(map, collision_radius);

%% Run Offline Planner
startPose = [2 2 0];
goalPose = [11 11 pi/4];

%create validator map
validator = validatorOccupancyMap;
validator.Map = map;

%declare and run planner with nonholonomic constraints
hybridPlanner = plannerHybridAStar(validator,MinTurningRadius=1,MotionPrimitiveLength=1);
refpath = plan(hybridPlanner,startPose,goalPose);

%show plan
show(hybridPlanner)


%% Move Swarm with Collision Avoidance
v = VideoWriter('ObstacleFormationPath5');
v.FrameRate = 5;  % Set frames per second
open(v);  % Open video file

theta_star = [0 0 pi/4 2*pi/3 pi -2*pi/3];
r_star = 2*[0.5 0  0.75 0.5 0.75 0.5];
r_si = 0.25;
gamma = 0;
x_0 = [2 2];
for drone = 1:length(theta_star)
    drone_pos(drone,:) = r_star(drone)*[cos(theta_star(drone)+gamma) sin(theta_star(drone)+gamma)]+x_0;
end
clf;
hold on
plt(obs, y_lim, x_lim, res, collision_radius, num_obs)
plot(x_0(1), x_0(2),  "o", LineWidth=3)
plot([drone_pos(:,1);drone_pos(1,1)], [drone_pos(:,2);drone_pos(1,2)],"r-o");
viscircles([[drone_pos(:,1);drone_pos(1,1)], [drone_pos(:,2);drone_pos(1,2)]], r_si(1), "Color","#EDB120");

hold off
xlim([0 14])
ylim([0 14])
axis square
frame = getframe(gcf);
writeVideo(v, frame);  % Write frame to video file
pause(0.5)

deviation = 0;
ind_deviation = zeros(4,1); 
N = length(r_star);
for n = 2:size(refpath.States,1)
    deviation(n) = 0; 
    gamma_prev = gamma;
    x_0 = refpath.States(n, 1:2);
    gamma = refpath.States(n,3);
    for drone = 1:N
        map = binaryOccupancyMap(y_lim, x_lim, 10^(res));
        setOccupancy(map, [obs.locs(:,1), obs.locs(:,2)], ones(num_obs,1));
        inflate(map,collision_radius-r_si+0.01)
        setOccupancy(map,[drone_pos(setdiff(1:N,drone),1) drone_pos(setdiff(1:N,drone),2)], ones(length(setdiff(1:N,drone)),1 ))
        inflate(map,2*r_si+0.02)
        ideal_location = r_star(drone)*[cos(theta_star(drone)+gamma) sin(theta_star(drone)+gamma)]+x_0;
        map_matrix = getOccupancy(map);
        [x,y] = ndgrid(0.0:10^-res:x_lim-0.01, 0.0:10^-res:y_lim-0.01);

        z = disFun(x,y,ideal_location);
        map_matrix = getOccupancy(map);
        map_matrix = ~map_matrix;
        map_matrix = flipud(map_matrix)';
        
        values = map_matrix.*z;       
        
        obs_pos = find(values == 0);
        values(obs_pos) = NaN;
        x(obs_pos) = NaN;
        y(obs_pos) = NaN;
        [val,loc] = min(values, [],"all","omitnan","linear");
        Goal_loc = [x(loc), y(loc)];
        
        startPose = [drone_pos(drone,:), gamma_prev];
        goalPose = [Goal_loc, gamma];
        
        map = binaryOccupancyMap(y_lim, x_lim, 10^(res));
        setOccupancy(map, [obs.locs(:,1), obs.locs(:,2)], ones(num_obs,1));
        inflate(map,collision_radius-r_si)
        setOccupancy(map,[drone_pos(setdiff(1:N,drone),1) drone_pos(setdiff(1:N,drone),2)], ones(length(setdiff(1:N,drone)),1 ))
        inflate(map,2*r_si)
        %create validator map
        validator = validatorOccupancyMap;
        validator.Map = map;
        
        %declare and run planner with nonholonomic constraints
        hybridPlanner = plannerHybridAStar(validator, MinTurningRadius=0.0095, MotionPrimitiveLength=0.0148);
        dronepath = plan(hybridPlanner,startPose,goalPose);
        dronepath = [dronepath.States;goalPose];
        for step = 1:size(dronepath,1)
            clf;
            hold on
            plt(obs, y_lim, x_lim, res, collision_radius, num_obs)
            plot(x_0(1), x_0(2),  "o", LineWidth=3)
            viscircles([[drone_pos(setdiff(1:N,drone),1)], ...
                [drone_pos(setdiff(1:N,drone),2)]], r_si(1), "Color","#EDB120");
            plot(dronepath(step,1),dronepath(step,2), 'go', MarkerSize=5, LineWidth=2)
            plot(dronepath(step:end,1),dronepath(step:end,2), '--',Color="#77AC30", LineWidth=1.5)
            
            hold off
            xlim([0 14])
            ylim([0 14])
            axis square
            frame = getframe(gcf);
            writeVideo(v, frame);  % Write frame to video file
            if n == 5 && drone == 3
                disp("")
            end
            pause(1)
        end
        drone_pos(drone,:) = dronepath(end,1:2);
        deviation(n) = deviation(n)+norm(drone_pos(drone,:)-ideal_location);
        ind_deviaton(drone, n) = norm(drone_pos(drone,:)-ideal_location);
    end
    clf;
    hold on
    plt(obs, y_lim, x_lim, res, collision_radius, num_obs)
    plot(x_0(1), x_0(2),  "o", LineWidth=3)
    plot([drone_pos(:,1);drone_pos(1,1)], [drone_pos(:,2);drone_pos(1,2)],"r-o", MarkerSize=4);
    viscircles([[drone_pos(:,1);drone_pos(1,1)], [drone_pos(:,2);drone_pos(1,2)]], r_si(1), "Color","#EDB120");
    
    hold off
    xlim([0 14])
    ylim([0 14])
    axis square
    frame = getframe(gcf);
    writeVideo(v, frame);  % Write frame to video file
    pause(1)
    deviation(n) = deviation(n)/N;
    
end

close(v)
%%
t = tiledlayout(N,1);
t.TileSpacing = "tight";
% t.Padding = "compact";

for i = 1:N
    nexttile;
    
    hold on
    plot(ind_deviation(i, :), LineWidth=2)
    plot(deviation,LineWidth=2)
    set(gca, "FontSize", 14)
    ylabel(i, Rotation=0, FontSize=20)
    if i == 1
       legend(["Agent Dev.", "Mean Form. Dev."], Location="Northwest")
    end
    hold off
    if i ~= N
        set(gca, "XTick", [])
        % set(gca, "YTick")
    end
    
end

ylabel(t,"Deviation", "FontWeight", "bold", "FontSize", 23)
xlabel(t,"Time Stamp [n]","FontWeight", "bold", "FontSize", 23)
% fontsize(18, "points")
fontname("Times New Roman")
% g = eye(2);
% f = zeros(2);
% % theta_star = [0.656; 2.7; 4; 5.8];
% % r_star = [2; 1; 2; 3];
% 
% theta_star = pi/2;
% r_star = 1;
% gamma = startPose(3);
% x_0 = startPose(1:2);
% r_si = [0.25];
% obs_r = 0.25;
% 
% N = length(r_star);
% drone_pos = zeros(N, 2);
% for drone = 1:length(theta_star)
%     drone_pos(drone,:) = r_star(drone)*[cos(theta_star(drone)+gamma) sin(theta_star(drone)+gamma)]+x_0;
% end
% figure
% hold on
% plot(x_0(1), x_0(2),  "o", LineWidth=2)
% plot([drone_pos(:,1);drone_pos(1,1)], [drone_pos(:,2);drone_pos(1,2)], "r-o",LineWidth=2)
% plt(obs)
% hold off
% xlim([-2 13])
% ylim([-2 13])
% pause(1)
% 
% 
% FormCBF = ZCBF(g, f, r_si, drone_pos, r_star, theta_star, gamma, x_0);
% FormCBF = FormCBF.update_obs(obs_r*ones(length(obs.locs),1), drone_pos, obs.locs, gamma, x_0);
% 
% for n = 2:size(refpath.States,1)
%     x_0 = refpath.States(n, 1:2);
%     gamma = refpath.States(n,3);
%     FormCBF = FormCBF.update_obs(obs_r*ones(length(obs.locs),1), drone_pos, obs.locs, gamma, x_0);
%     for drone = 1:N
%         drone_pos(drone, :) = FormCBF.QP(drone);
%         FormCBF = FormCBF.update_obs(obs_r*ones(length(obs.locs),1), drone_pos, obs.locs, gamma, x_0);
%     end
%     clf;
%     hold on
%     plt(obs)
%     plot(x_0(1), x_0(2),  "o", LineWidth=3)
%     viscircles([[drone_pos(:,1);drone_pos(1,1)], [drone_pos(:,2);drone_pos(1,2)]], r_si(1), "Color","r");
% 
%     hold off
%     xlim([-2 13])
%     ylim([-2 13])
%     pause(1)
% 
% end