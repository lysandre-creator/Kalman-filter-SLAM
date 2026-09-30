function Q = cov_odometry(u,a_odom)
% Builds the odometry covariance matrix Q

    u = u(:) ;
    assert(numel(u) == 2, 'The odometry input must contain exactly 2 elements') ;

    Q = eye(2); 
    variance = ( a_odom * norm([u(1) u(2)]) )^2 ;
    Q(1,1) = variance ;
    Q(2,2) = variance;

