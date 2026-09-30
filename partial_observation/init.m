function [X0, P0, A, B] = init(y, a_obs, epsilon)
% Initializes the state, covariance and filter matrices from the first
% perception
%
% X0, P0 : initial state and its covariance matrix
% A, B : constant matrices needed for steps of the Kalman filter

    y = y(:);
    N = size(y, 1) / 2; % Number of detected landmarks so far

    % build P0
    py = cov_observation(y,a_obs);
    C = zeros(2*N + 2,2);
    D = zeros(2,2*N) ; 
    E = [D;
         py] ;
    P0 = [C E] ;
    P0(1,1) = epsilon ;
    P0(2,2) = epsilon ;

    % build A
    A = eye(2*N + 2);

    % build B
    B = zeros(2*N + 2,2) ;
    B(1,1) = 1 ;
    B(2,2) = 1 ;

    % build X0
    I = [0 0]';  % drone starts at the origin
    X0 = [I ; y];
end

