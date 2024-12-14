classdef ZCBF
    properties
        g = eye(2);
        f = zeros(2);
        r_si = zeros(10,1);
        r_sig = [];
        drone_pos = zeros(10,1)
        obs_pos = [];
    end
    methods
        function obj = ZCBF(g, f, r_si, drone_pos)
            obj.g = g;
            obj.f = f;
            obj.r_si = r_si;
            obj.drone_pos = drone_pos;
        end

        function obj = update_obs(obj, r_sig, drone_pos, obs_pos)
            obj.r_sig = r_sig;
            obj.drone_pos = drone_pos;
            obj.obs_pos = obs_pos;
        end

        function h_si = h(obj, current_drone)
            dr_pos = obj.drone_pos(set_diff(1:end,current_drone),:);
            o_pos = obj.obs_pos;

            j_pos = [dr_pos;o_pos];
            i_pos = obj.drone_pos(current_drone,:);
            r_sj = [obj.r_si(set_diff(1:end,current_drone));obj.r_sig];
            r_s = obj.r_si(current_drone);

            h_si = log(sum(exp( (r_s+r_sj)-vecnorm(i_pos-j_pos,2,2) )));
        end

        function bottleneck = find_minimizer(obj, current_drone)
            dr_pos = obj.drone_pos(set_diff(1:end,current_drone),:);
            o_pos = obj.obs_pos;
            i_dr = obj.drone_pos(current_drone,:);

            bottleneck = inf;
            bottleneck_norm = inf;
            for drone = 1:size(dr_pos, 1)
                if norm(i_dr-dr_pos(drone,:)) <= bottleneck_norm
                    bottleneck = drone;
                    bottleneck_norm = norm(i_dr-dr_pos(drone,:));
                else
                    continue
                end
  
            end
            for obstacle = 1:size(o_pos, 1)
                if norm(i_dr-o_pos(obstacle,:)) <= bottleneck_norm
                    bottleneck = obstacle;
                    bottleneck_norm = norm(i_dr-dr_pos(drone,:));
                else
                    continue
                end
            end
        end

        function val = LgHs(obj, current_drone)
            dr_pos = obj.drone_pos;
            o_pos = obj.obs_pos;
            i_dr = current_drone;


            for drone = 1:size(dr_pos, 1)

            end
            for obstacle = 1:size(o_pos, 1)
            end
        end

    end
end
