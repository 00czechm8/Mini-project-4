classdef obstacle_field
    properties
        locs = round(10*rand([10,2]), 3)
    end
    methods
        %constructor that creates an obstacle based on decimal res, bounds
        %of playing field, and the number obstacles desired
        function obj = obstacle_field(x_lim, y_lim, res, num_obs)

            obj.locs = [round(x_lim*rand([num_obs,1]), res), round(y_lim*rand([num_obs,1]), res)];
        end

        %plots black dots where the obstacles are
        function plt(obj)
          plot(obj.locs(:,1), obj.locs(:,2), 'ko', MarkerSize=10, MarkerFaceColor='k')
          
        end
    end
end