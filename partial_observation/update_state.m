function [X, P, A, B] = update_state(X, P, A, B, Y_new, a_obs)
% Updates the current variables giving the new detected landmarks
%
% X, P : new state and covariance taking new landmarks into account
% A, B : updated for size consistency

    X = X(:) ;
    Y_new = Y_new(:) ;

    assert(mod(numel(Y_new), 2) == 0, "Y_new must be even") ;
    
    nb_new_lm = numel(Y_new) / 2 ;
    
    % updates state vector X
    Y_new_abs = Y_new + repmat( [ X(1) ; X(2) ] , nb_new_lm, 1) ; % repmat duplicates the column vector nb_new_lm times
    X = [X ; Y_new_abs] ;

    % updates covariance matrix P
    py_new = cov_observation(Y_new, a_obs) ;
    P = blkdiag(P, py_new) ;
    
    % updates A
    A = blkdiag(A, eye(2*nb_new_lm, 2*nb_new_lm)) ;

    % updates B
    B = [ B ; zeros(2*nb_new_lm, 2)] ;

end
