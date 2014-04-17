function [pose] = dense_tracking_inv_compositional(K, img1_pyr, img2_pyr, depth1_pyr, initial_guess)
    k_max = 30;
    epsilon = 1;

    guess = initial_guess;
    
    levels = size(img1_pyr,1);
    
    levels = size(img1_pyr,1);
    
    for level = levels:-1:1
        increment = double([0;0;0;0;0;0]);
    
        level
        
        img1 = img1_pyr{level};
        img1_dbl = double(img1);
        img1_depth = depth1_pyr{level};
        img2 = img2_pyr{level};

        [ points2d, points3d ] = get_3d_points(K, img1_depth, size(img1,1), size(img1,2));
        transformed_points3d = points3d;
        
        J_img = compute_template_jacobians(K, img1, points2d, points3d);
    
        k = 1;
        
        error = inf;
        error_last = 0;
        accept = true;
        while accept && (k == 1 || (error_last - error) > epsilon) && (k < k_max)
            increment = twistexp(increment)
            new_guess = increment * guess
            
            tic
            % successively warp the 3d points by increment
            [transformed_points2d, transformed_points3d] = ...
                warpAndProject(points3d, K, new_guess); 
            toc 
            
            tic
            warped_img2 = warpimgIntensityWithDepth(img2, img1_depth, points2d, ...
                                            transformed_points2d, transformed_points3d);
            toc
                                        
            all_residuals = warped_img2 - img1_dbl;
            
            error_last = error;
            [ increment, error ] = compute_increment(J_img, all_residuals); 
            
            error
            
            if error > error_last 
                % the error is worse, try the next pyramid level
                accept = false;
            else
                guess = new_guess;
            end
            
            k = k+1;
        end
    end
    
    pose = guess;
end

function [J_img] = compute_template_jacobians(K, template, points2d, points3d)
    [ gradx, grady ] = imgradientxy(template, 'CentralDifference'); 
    n = size(points2d,2);
    J_img = NaN(size(template, 1), size(template,2), 6);
    for i = 1:n
        x = points2d(1,i);
        y = points2d(2,i);
        
        x_world = points3d(1, i);
        y_world = points3d(2, i);
        z_world = points3d(3, i);
        
        Jwi = get_warp_jacobian(K, x_world, y_world, z_world);
        
        gradx_val = gradx(y,x);
        grady_val = grady(y,x);
        JIi = [ gradx_val, grady_val ];
        
        Ji = JIi * Jwi;
        J_img(y, x,:) = Ji';
    end
end

function [increment, error ] = compute_increment(J_img, all_residuals)
    % iterate over residuals
    % and J_img at the same time, remove NaN entries on both sides
    empty_test = ones(1,6);
    n = size(J_img, 1) * size(J_img, 2);
    J = reshape(J_img, n, 6);
    residuals = reshape(all_residuals, n, 1);
    J_final = NaN(n, 6);
    residuals_final = NaN(n, 1);
    idx = 0;
    for i = 1:n
        if ~isfinite(residuals(i,1))
            continue;
        end
        
        if isnan(J(i, :)) == empty_test
            continue;
        end
        
        idx = idx + 1;
        J_final(idx,:) = J(i, :);
        residuals_final(idx,1) = residuals(i,1);
    end
    
    % drop the NaNs at the end 
    J = J_final(1:idx-1,:);
    residuals = residuals_final(1:idx-1,:);
    
    % error 
    error = (residuals'*residuals) / size(residuals,1);
    
    % Hessian matrix 
    H = J' * J;
    
    % steepest descent
    steepest_descent = -J'*residuals;
    
    % compute the increment :
    increment = H \ steepest_descent     
end