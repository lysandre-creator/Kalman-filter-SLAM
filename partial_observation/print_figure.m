function print_figure(traj_odom, traj_corr, X, P)
% displays a single summary figure: odometry trajectory, Kalman-corrected
% trajectory, and final landmark positions with their uncertainty circles
% (radius = standard deviation, from the diagonal of P).

N = (numel(X) - 2 ) / 2 ; % number of detected landmarks so far

ux = traj_odom(1:2:end) ;
uy = traj_odom(2:2:end) ;

ux_corr = traj_corr(1:2:end) ;
uy_corr = traj_corr(2:2:end) ;

theta = linspace(0, 2*pi, 100) ;

figure

% left panel - final map
subplot(1,2,1)
hold on

text(0.02, 0.98, sprintf('N = %d', N), 'Units', 'normalized', 'VerticalAlignment', 'top', 'FontWeight', 'bold', 'BackgroundColor', 'w', 'Margin', 3) ;


% Uncertainty circles around the final estimated landmark positions
for i = 1:N
    xl = X(2*i+1) ;
    yl = X(2*i+2) ;
    rl = sqrt((P(2*i+1,2*i+1) + P(2*i+2,2*i+2)) / 2) ;
    plot(rl*cos(theta) + xl, rl*sin(theta) + yl, 'b--')
end

h_land = plot(X(3:2:end), X(4:2:end), 'bs', 'MarkerSize', 8, 'LineWidth', 1.5) ;
h_odom = plot(ux, uy, 'go-', 'LineWidth', 2) ;
h_corr = plot(ux_corr, uy_corr, 'ro-', 'LineWidth', 2) ;

legend([h_odom h_land h_corr], {'trajectory with Odometry', 'Landmarks', 'corrected trajectory'}, 'Location', 'best')
title('Drone trajectory and landmark map with uncertainty')
xlabel('Position X')
ylabel('Position Y')
axis equal
grid on

% right panel : final covariance matrix
subplot(1,2,2)
imagesc(P)
colorbar
axis square
title('Covariance matrix P')
xlabel('State index')
ylabel('State index')

end

