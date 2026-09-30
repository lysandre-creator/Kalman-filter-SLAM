function [X_pred, P_pred] = pred_odometry(X_prev, ut, A, B, P_prev, Q)
% Prediction of the new state thanks to odometry measures
% X_pred, P_pred : predicted state and its covariance matrix

    X_pred = A*X_prev + B*ut ;

    P_pred = A*P_prev*A' + B*Q*B' ;

