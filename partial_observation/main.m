% Kalman filter (SLAM, 2D case, partially observed landmarks, in unknown order, with nearest-neighbor matching).
%
% The input text file must alternate odometry and perception lines, and start and end with a perception line
%
% It displays a single summary figure at the end 


% open the data file
fid = fopen('data/FullObservation.data', 'r'); 
if fid == -1
    error('Could not open the file.');
end

nb_line = 0 ;

% read the first line: first perception
line1 = fgetl(fid);
assert(ischar(line1), 'Could not read the first line') ;
data = textscan(strrep(line1, 'percep :', ''), '%f') ;
y_init = data{1} ;
nb_line = nb_line + 1 ;

% for plotting :
traj_odom = [0 0];    % stores the odometry-predicted states 
traj_corr = [0 0];   % stores the Kalman-corrected states

a_odom = 0.2; % proportionality coefficient for odometry measures
a_obs = 0.1 ; % proportionality coefficient for observation measures
epsilon = 0.01 ; % variance of the drone's initial position - around [0 0]
thresh = 1.5 ; % distance threshold beyond which a landmark is considered new

% initialize the filter with the first perception
[X, P, A, B] = init(y_init, a_obs, epsilon);


while ~feof(fid)

    % extract odometry vector u
    odom_line = fgetl(fid) ;
    if isequal(odom_line, -1) || isempty(strtrim(odom_line)) % end of file
        break ;
    end 
    data = textscan(odom_line, 'odom : %f %f');
    u = cell2mat(data)';
    nb_line = nb_line + 1 ;

    % compute the covariance matrix of the odometry u
    Q = cov_odometry(u, a_odom) ;

    % predict the new state based on odometry
    [X_pred, P_pred] = pred_odometry(X, u, A, B, P, Q) ;

    % store the predicted state
    traj_odom = [ traj_odom [X_pred(1) X_pred(2)] ] ;
    
    % extract real perception vector y from the new state
    obs_line = fgetl(fid);
    assert(ischar(obs_line), 'Could not read an observation line') ;
    data = textscan(strrep(obs_line, 'percep :', ''), '%f');
    y = data{1} ;
    nb_line = nb_line + 1 ;
    
    % match detected with known landmarks
    [H, Y_known, Y_new] = match_landmarks(X_pred,y, thresh) ;
    
    if ~isempty(Y_known)
        % predict the observation from this predicted state
        Y_pred = pred_observation(X_pred, H) ;

        % compute the covariance matrix of the observation Y_known
        Py = cov_observation(Y_known, a_obs) ;

        % apply Kalman filter to get the corrected state 
        [X_corr, P_corr] = corr_observation(X_pred, P_pred, Y_pred, Y_known, H, Py) ; 
    else
        X_corr = X_pred ;
        P_corr = P_pred ;
    end

    % store the corrected state
    traj_corr = [ traj_corr [X_corr(1) X_corr(2)] ] ;

    X = X_corr ; 
    P = P_corr ;

    % add newly detected landmarks to the state 
    if ~isempty(Y_new)
        [X, P, A, B] = update_state(X, P, A, B, Y_new, a_obs);
    end

    
end

fclose(fid);
fprintf('Number of lines read : %d\n', nb_line) ;

print_figure(traj_odom, traj_corr, X, P) ;

