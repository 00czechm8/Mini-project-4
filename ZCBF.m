classdef ZCBF
    properties
        g = eye(2);
        f = zeros(2);
        r_si = zeros(10,1);
        r_sig = [];
        drone_pos = zeros(10,1)
        obs_pos = [];
        R_star = [];
        Theta_star = [];
        Gamma = [];
        X_0 = [0 0];
    end
    methods
        function obj = ZCBF(g, f, r_si, drone_pos, r_star, theta_star, gamma, x_0)
            obj.g = g;
            obj.f = f;
            obj.r_si = r_si;
            obj.drone_pos = drone_pos;
            obj.R_star = r_star;
            obj.Theta_star = theta_star;
            obj.Gamma = gamma;
            obj.X_0 = x_0;
        end

        function rel_coord = calc_rel_coord(obj, r_star, theta_star, gamma, drone_center, world_pos)
            rel_coord = world_pos'-(r_star*[cos(theta_star+gamma);sin(theta_star+gamma)]+drone_center');
        end

        function obj = update_obs(obj, r_sig, drone_pos, obs_pos, gamma, x_0)
            obj.r_sig = r_sig;
            obj.drone_pos = drone_pos;
            obj.obs_pos = obs_pos;
            obj.Gamma = gamma;
            obj.X_0 = reshape(x_0, [1 2]);
        end

        function h_si = calc_h(obj, r_s, r_sj, i_pos, j_pos)
            % dr_pos = obj.drone_pos(set_diff(1:end,current_drone),:);
            % o_pos = obj.obs_pos;
            % 
            % j_pos = [dr_pos;o_pos];
            % i_pos = obj.drone_pos(current_drone,:);
            % r_sj = [obj.r_si(set_diff(1:end,current_drone));obj.r_sig];
            % r_s = obj.r_si(current_drone);

            h_si = log(sum(exp( -(r_s+r_sj)+vecnorm(i_pos-j_pos,2,2) )));
        end

        function [bottleneck, r_sj] = find_minimizer(obj, current_drone)
            dr_pos = obj.drone_pos;
            o_pos = obj.obs_pos;
            i_dr = obj.drone_pos(current_drone,:);
            r_s = obj.r_si(current_drone);
            R_si = obj.r_si;
            R_sig = obj.r_sig;


            bottleneck = [inf,inf];
            bottleneck_norm = inf;
            for drone = 1:size(dr_pos, 1)
                if current_drone == drone
                    continue
                elseif norm(i_dr-dr_pos(drone,:))-(r_s+R_si(drone)) <= bottleneck_norm
                    bottleneck = dr_pos(drone,:);
                    bottleneck_norm = norm(i_dr-dr_pos(drone,:))-(r_s+R_si(drone));
                    r_sj = obj.r_si(drone);
                else
                    continue
                end
  
            end

            for obstacle = 1:size(o_pos, 1)
                if norm(i_dr-o_pos(obstacle,:))-(r_s+R_sig(obstacle)) <= bottleneck_norm
                    bottleneck = o_pos(obstacle,:);
                    bottleneck_norm = norm(i_dr-dr_pos(drone,:))-(r_s+R_sig(drone));
                    r_sj = obj.r_sig(obstacle);
                else
                    continue
                end
            end
        end

        function val = LgHs(obj, current_drone)
            r_star = obj.R_star(current_drone);
            theta_star = obj.Theta_star(current_drone);
            gamma = obj.Gamma;
            x_0 = obj.X_0;
            dr_pos = obj.drone_pos;
            o_pos = obj.obs_pos;
            [x_j, r_sj] = find_minimizer(obj, current_drone);
            r_s = obj.r_si(current_drone);

            
            for i = 1:size(dr_pos,1)
                dr_pos(i,:) = calc_rel_coord(obj, r_star, theta_star, gamma, x_0, dr_pos(i,:));
            end

            for i = 1:size(o_pos,1)
                o_pos(i,:) = calc_rel_coord(obj, r_star, theta_star, gamma, x_0, o_pos(i,:));
            end
            i_pos = dr_pos(current_drone,:);

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

        function u = QP(obj, current_drone)
            r_s = obj.r_si(current_drone);
            r_sig_j = [obj.r_si(setdiff(1:end, current_drone));obj.r_sig];
            x_j = [obj.drone_pos(setdiff(1:end, current_drone), :);obj.obs_pos];
            x_i = obj.drone_pos(current_drone,:);
            
            options = optimoptions("quadprog","Display", "none");
            H = [2 0 0; 0 2 0; 0 0 0];
            c = [0 0 1];
            A = [-LgHs(obj, current_drone) -calc_h(obj, r_s, r_sig_j, x_i, x_j)];
            b = 0;

            z = quadprog(H,c,A,b, [], [],[],[],[],options);
            u = z(1:2);
        end
    end
end
