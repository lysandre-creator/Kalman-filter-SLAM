function Y_pred = pred_observation(X_pred, H)
% Predicts the observation expected from the odometry-predicted state
% Y_pred : observation vector

    Y_pred = H * X_pred ; 


