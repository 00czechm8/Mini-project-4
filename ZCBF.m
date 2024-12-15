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

        function h_si = calc_h(obj, r_s, r_sj, i_pos, j_pos)
            % dr_pos = obj.drone_pos(set_diff(1:end,current_drone),:);
            % o_pos = obj.obs_pos;
            % 
            % j_pos = [dr_pos;o_pos];
            % i_pos = obj.drone_pos(current_drone,:);
            % r_sj = [obj.r_si(set_diff(1:end,current_drone));obj.r_sig];
            % r_s = obj.r_si(current_drone);

            h_si = log(sum(exp( (r_s+r_sj)-vecnorm(i_pos-j_pos,2,2) )));
        end

        function [bottleneck, r_sj] = find_minimizer(obj, current_drone)
            dr_pos = obj.drone_pos;
            o_pos = obj.obs_pos;
            i_dr = obj.drone_pos(current_drone,:);
            
            bottleneck = [inf,inf];
            bottleneck_norm = inf;
            for drone = 1:size(dr_pos, 1)
                if current_drone == drone
                    continue
                elseif norm(i_dr-dr_pos(drone,:)) <= bottleneck_norm
                    bottleneck = dr_pos(drone,:);
                    bottleneck_norm = norm(i_dr-dr_pos(drone,:));
                    r_sj = obj.r_si(drone);
                else
                    continue
                end
  
            end

            for obstacle = 1:size(o_pos, 1)
                if norm(i_dr-o_pos(obstacle,:)) <= bottleneck_norm
                    bottleneck = o_pos(obstacle,:);
                    bottleneck_norm = norm(i_dr-dr_pos(drone,:));
                    r_sj = obj.r_sig(obstacle);
                else
                    continue
                end
            end
        end

        function val = LgHs(obj, current_drone)
            dr_pos = obj.drone_pos;
            o_pos = obj.obs_pos;
            i_pos = dr_pos(current_drone,:);
            [x_j, r_sj] = find_minimizer(obj, current_drone);
            r_s = obj.r_si(current_drone);

            dr_pos = dr_pos(setdiff(1:end, current_drone),:);
            pos = [dr_pos;o_pos];
            r_sig_j = [obj.r_si(setdiff(1:end, current_drone));obj.r_sig];
            
            x_i = i_pos;
            sum = 0;
            for i = 1:size(pos,1)
                sum = sum + exp(calc_h(obj, r_s, r_sig_j(i), i_pos, pos(i,:)));
            end

            val = (-(exp(calc_h(obj, r_s, r_sj, i_pos, x_j))/sum)*[(x_i(1)-x_j(1))/norm(x_i-x_j), (x_i(2)-x_j(2))/norm(x_i-x_j)])*obj.g;
            

        end

    end
end
