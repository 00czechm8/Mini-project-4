close all
clearvars -except refpath
theta_star = [0.656 2.7 4 5.8];
r_star = [0.5 1 2 3];
x_0 = refpath.States(:,1:2);
gamma = refpath.States(:,3);


% Set up video writer
v = VideoWriter('Sample Formation path');
v.FrameRate = 2;  % Set frames per second
open(v);  % Open video file
for i = 1:size(x_0,1)
    for drone = 1:length(theta_star)
        x(drone,:) = r_star(drone)*[cos(theta_star(drone)+gamma(i)) sin(theta_star(drone)+gamma(i))]+x_0(i,:);
    end
    x(end+1,:) = x(1,:);
    hold on
    xlim([-2, 13])
    ylim([-2, 13])
    plot(x_0(1:i,1), x_0(1:i,2), 'o')
    plot(x(:,1), x(:,2),'r-o');
    hold off
    frame = getframe(gcf);
    writeVideo(v, frame);  % Write frame to video file
    pause(1)
    clf;
    clear x
end
close(v)
