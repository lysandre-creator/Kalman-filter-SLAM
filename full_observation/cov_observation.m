function Py = cov_observation(y,a_obs)
% Builds the observation covariance matrix Py

    y = y(:) ;
    N = size(y , 1) / 2 ; % Number of landmarks
    Py = eye(2*N) ; 
    for i = 0:N-1
        variance = ( a_obs*norm([y(2*i+1) y(2*i+2)]) )^2 ;
        Py(2*i+1,2*i+1) = variance ;
        Py(2*i+2,2*i+2) = variance ;
    end

