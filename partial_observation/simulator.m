% Simple simulator: generates a data file for a drone moving along a square loop and observing landmarks within a fixed detection radius.
%
% Also displays the ground truth landmarks and trajectory,for visual comparison with the filter's output.


% true fixed landmarks (ground truth)
landmarks_true = [1 1 ;
    7 1 ;
    4 4 ;
    1 7 ;
    7 7 ;
    9 3 ] ;

% simulation parameters 
side = 8 ; 
steps_per_side = 5 ; 
R = 5 ; % detection radius
a_odom = 0.2 ; % same noise coefficients as main.m
a_obs = 0.1 ;

% corners of the square trajectory, starting at the origin 
corners = [0 0 ; side 0 ; side side ; 0 side ; 0 0 ] ;

% build the list of true drone positions along the square
true_traj = [0 0] ;  % first position: the origin
for c = 1:4
    step_vector = (corners(c+1, :) - corners(c, :)) / steps_per_side ;
    for s = 1:steps_per_side
        last_pos = true_traj(end, :) ;
        true_traj = [true_traj ; last_pos + step_vector] ;
    end
end

nb_steps = size(true_traj, 1) - 1 ;

% open the output file
fid = fopen('data/SimulatedObservation.data', 'w') ;
if fid == -1
    error('Could not create the output file.') ;
end

% first perception, from the origin
y = simulate_perception(true_traj(1, :), landmarks_true, R, a_obs) ;
fprintf(fid, 'percep : ') ;
fprintf(fid, '%f %f ', y) ;
fprintf(fid, '\n') ;

% odometry/perception pairs for the rest of the trajectory
for k = 1:nb_steps
    true_step = true_traj(k+1, :) - true_traj(k, :) ;

    % noisy odometry, same noise model as cov_odometry.m
    Q = cov_odometry(true_step, a_odom) ;
    u_noisy = true_step' + sqrt(diag(Q)) .* randn(2, 1) ;
    fprintf(fid, 'odom : %f %f\n', u_noisy(1), u_noisy(2)) ;

    y = simulate_perception(true_traj(k+1, :), landmarks_true, R, a_obs) ;
    fprintf(fid, 'percep : ') ;
    fprintf(fid, '%f %f ', y) ;
    fprintf(fid, '\n') ;
end

fclose(fid) ;
fprintf('Simulated data written to data/SimulatedObservation.data (%d steps)\n', nb_steps) ;

% ground truth figure
figure
hold on
plot(landmarks_true(:,1), landmarks_true(:,2), 'bs', 'MarkerSize', 8, 'LineWidth', 1.5) ;
plot(true_traj(:,1), true_traj(:,2), 'ko-', 'LineWidth', 2) ;
legend('True landmarks', 'True trajectory', 'Location', 'best')
title('Ground truth used by the simulator')
xlabel('Position X')
ylabel('Position Y')
axis equal
grid on

