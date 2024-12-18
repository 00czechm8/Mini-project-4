classdef obstacle_field
    properties
        locs = round(10*rand([10,2]), 3)
    end
    methods
        %constructor that creates an obstacle based on decimal res, bounds
        %of playing field, and the number obstacles desired
        function obj = obstacle_field(x_lim, y_lim, res, num_obs)

            obj.locs = [round((10-4.2)*rand([num_obs,1])+4.2, res), round((10-4.2)*rand([num_obs,1])+4.2, res)];
        end

        %plots black dots where the obstacles are
        function plt(obj, y_lim,x_lim, res, collision_radius, num_obs)
          % viscircles([obj.locs(:,1) obj.locs(:,2)], 0.75, "Color", 'k', 'EnhanceVisibility',1);
          map = binaryOccupancyMap(y_lim, x_lim, 10^(res));
          setOccupancy(map, [obj.locs(:,1), obj.locs(:,2)], ones(num_obs,1))
          inflate(map, collision_radius);
          show(map)
        end
    end
end