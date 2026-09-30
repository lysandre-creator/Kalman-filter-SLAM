function [H, Y_known, Y_new] = match_landmarks(X, Y, threshold)
    % Associates observed landmarks with known landmarks or new ones.
    % Builds the matrix H for prediction of observation
    % 
    % Y_known, Y_new : relative coordinates of perceived landmarks from Y
    % separated in 2 clusters (known and new ones).
    % H : matrix used for predicting observation
    
    assert(mod(numel(X), 2) == 0, 'state vector must be even') ;
    assert(mod(numel(Y), 2) == 0, 'observation vector must be even') ;
    
    X = X(:) ;
    Y = Y(:) ;

    Y_known = [] ;
    Y_new = [] ;
    
    N = ( numel(X) - 2 ) / 2 ; % number of  known landmarks
    M = numel(Y) / 2 ; % number of observed landmarks

    links = [] ;

    for j = 0:M-1 % iterates on observed landmarks

        perceived_lm = Y(2*j+1 : 2*j+2) ;

        min_dist = inf ;
        index_min = -1 ;
        
        for i = 0:N-1 % iterates on known landmarks
            if ismember(i, links)
                continue
            end

            lm_relative = X(2*i + 3 : 2*i + 4) - X(1:2);

            dist = norm(lm_relative - perceived_lm) ;
            
            if dist < min_dist 
                min_dist = dist ;
                index_min = i ;
            end
        end

        if min_dist < threshold 
            Y_known = [ Y_known ; perceived_lm ] ;
            links = [ links index_min ] ;

        else 
            Y_new = [Y_new ; perceived_lm ] ;
        end
        
    end

    assert(numel(Y_known) + numel(Y_new) == 2*M, 'Y_known and Y_new sizes do not match Y size') ;
    assert(numel(Y_known) == 2 * numel(links)) ;
    

   % compute H
    nb_links = numel(links) ; % number of linked landmarks

    H = zeros(2*nb_links, 2*N + 2) ;
    
    for k = 0 : nb_links - 1
        i = links(k + 1) ;

        H(2*k + 1:2*k + 2, 1:2) = -eye(2, 2);
        H(2*k + 1:2*k + 2, 2*i + 3:2*i + 4) = eye(2);
    end

end
