% clear
clc
% close all

% Input
length = 4;
width = 7.5;
Pos = [0 5-1];
% Output
[P1, n] = CreateMapPoints(length, width, Pos);
Points  = P1;
n_points = size(Points);
% plot(Points(:,1),Points(:,2),'o','MarkerSize',10)
figure; % create a new figure
grid on;
hold on; % retain the current plot
axis([-10 1 -1 10]); % adjust the limits as needed
% Initialize an empty plot for previous points
prev_plot = [];

for i = 1:n_points(1)
    % Plot the current point
    plot(Points(i, 1), Points(i, 2), 'o', 'MarkerSize', 10);
    
    % Add the order number next to the point
    text(Points(i, 1), Points(i, 2), [' ' num2str(i)], 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    
    % Pause to show the plot
    pause(0.5); % adjust this value to change speed of plotting
    
    % Clear the plot (except for the axes) before moving to the next point
    % cla;
end

hold off;