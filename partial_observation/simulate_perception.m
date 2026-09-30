function y = simulate_perception(drone_pos, landmarks_true, R, a_obs)
% Simulates what the drone perceives from its true position: landmarks
% within radius R are seen, in random order, with noise added using the
% same noise model as cov_observation.m
%
% y : relative landmark positions (empty if no landmark is within range)

nb_landmarks = size(landmarks_true, 1) ;
visible = [] ;  % indices of the landmarks within range

for i = 1:nb_landmarks
    relative_pos = landmarks_true(i, :) - drone_pos ;
    if norm(relative_pos) < R
        visible = [visible i] ;
    end
end

order = visible(randperm(numel(visible))) ;  % shuffle: unknown order

y = [] ;
for i = order
    relative_pos = (landmarks_true(i, :) - drone_pos)' ;
    Py = cov_observation(relative_pos, a_obs) ;  % reuse the project's own noise model
    noisy_pos = relative_pos + sqrt(diag(Py)) .* randn(2, 1) ;
    y = [y ; noisy_pos] ;
end
end

