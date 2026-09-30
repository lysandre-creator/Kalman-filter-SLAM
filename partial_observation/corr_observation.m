function [X_corr, P_corr] = corr_observation(X_pred, P_pred, Y_pred, Y, H, Py)
% Correction step of the Kalman filter.
% X_corr, P_corr : corrected state and its covariance matrix 

    K = P_pred*H'/(H*P_pred*H' + Py) ;
    X_corr = X_pred + K*(Y - Y_pred) ; 
    P_corr = P_pred - K*H*P_pred ;

