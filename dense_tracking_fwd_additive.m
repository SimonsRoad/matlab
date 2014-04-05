function [pose] = dense_tracking_fwd_additive(K, img1_pyr, img2_pyr, depth1_pyr, initial_guess)
    % TODO (mtourne): eventually expose this.
    k_max = 30;
    epsilon = 0.2;

    guess = initial_guess;
    
    levels = size(img1_pyr,1);
    
    for level = levels:-1:1
        increment = double([0;0;0;0;0;0]);
    
        level
        
        img1 = img1_pyr{level};
        img1_depth = depth1_pyr{level};
        img2 = img2_pyr{level};

        [ points2d, points3d ] = get_3d_points(K, img1_depth, size(img1,1), size(img1,2));

        k = 1;
        
        error = inf;
        error_last = 0;
        accept = true;
        while accept && (k == 1 || (error_last - error) > epsilon) && (k < k_max)
            increment = twistexp(increment)
            new_guess = increment * guess
            
            [transformed_points2d, transformed_points3d] = warpAndProject(points3d, K, new_guess);      

            [J, residuals] = compute_jacobian_forward_additive(K, img1, img2, img1_depth, points2d, ...
                                                               transformed_points2d, transformed_points3d);

            error_last = error;
            error = (residuals'*residuals) / size(residuals,1)

            if error > error_last 
                % the error is worse, try the next pyramid level
                accept = false;
            else
                guess = new_guess;
                % hessian equation with a matrix operation
                % same as :
                % H = zeros(6,6); for j=1:size(J,1), H = H + (J(j,:)' * J(j,:)); end              
                H = J'*J;
                % same as :
                % steepest_grad = zeros(6,1); for j=1:size(J,1), steepest_grad = steepest_grad - J(j,:)' * residuals(j); end
                steepest_descent = -J'*residuals;
                % compute the increment :
                increment = H \ steepest_descent                
            end

            k = k + 1;
        end        
    end
    
    pose = guess;
end

function [JWi] = get_warp_jacobian(K, x, y, z)
    focals = [K(1,1); K(2,2)];
    
    JWi = [ 1/z, 0, -x/z^2, -(x*y)/z^2, 1+(x^2/z^2), -y/z; ...
            0, 1/z, -y/z^2, -(1+(y^2/z^2)), (x*y)/z^2, x/z];
    JWi = bsxfun(@times, JWi, focals);
end


function [J, residuals] = compute_jacobian_forward_additive(K, img1, img2, img1_depth, ...
                                                             points2d, transformed_points2d, transformed_points3d)
    %warped_img2 = warpimgIntensity(img2, points2d, transformed_points2d);
    warped_img2 = warpimgIntensityWithDepth(img2, img1_depth, points2d, ...
                                            transformed_points2d, transformed_points3d);
    %subplot(1,3,1), imshow(uint8(img2)); 
    %subplot(1,3,2), imshow(uint8(warped_img2));
    %subplot(1,3,3), imshow(uint8(img1));
    [ img2_gradx, img2_grady ] = imgradientxy(warped_img2, 'CentralDifference');
    all_residuals = warped_img2 - double(img1);
    residuals = [];
    J = [];
    for i = 1:size(points2d,2)
        x = points2d(1,i);
        y = points2d(2,i);

        if ~isfinite(x) || ~isfinite(y)
            continue
        end
        
        residual = all_residuals(y,x);
        
        gradx_val = img2_gradx(y,x);
        grady_val = img2_grady(y,x);
        
        x_world = transformed_points3d(1, i);
        y_world = transformed_points3d(2, i);
        z_world = transformed_points3d(3, i);
        
        if ~isfinite(residual)
            continue;
        end
            
        if ~isfinite(gradx_val) || ~isfinite(grady_val)
            %warning('grad point is not finite');
            continue
        end
        
        if ~isfinite(x_world) || ~isfinite(y_world) || ~isfinite(z_world)
           warning('world point is not finite');
           continue;
        end
        
        % TODO (mtourne): would be better to remove all nans at once and
        % reshape
        residuals = [residuals; residual];
   
        JIi = [ gradx_val, grady_val ];
                
        Jwi = get_warp_jacobian(K, x_world, y_world, z_world);
        Ji = JIi * Jwi;
        J = [J; Ji];
    end
end

% function [J, residuals] = compute_jacobian_forward_additive(K, img1, img2, img2_gradx, img2_grady, ...
%                                                             points2d, transformed_points2d, transformed_points3d)
%     n = size(points2d, 2);
%     x_size = size(img2, 1);
%     y_size = size(img2, 2);
%     bad_transformed = 0;
%     residuals = [];
%     J = [];
%     for i = 1:n
%         x = points2d(1,i);
%         y = points2d(2,i);
%         x_trans = transformed_points2d(1, i);
%         y_trans = transformed_points2d(2, i);
%         
%         if x_trans+1 > x_size || y_trans+1 > y_size ...
%             || x_trans < 1 || y_trans < 1
%             bad_transformed = bad_transformed + 1;
%             continue;
%         end
%         
%         img1_val = img1(x,y,:);
%         % x_trans and y_trans are not integers
%         % use bilinear interpolation 
%         img2_val = interpImg(img2, [x_trans, y_trans]);
%         residual = img2_val - img1_val;
%         % get the mean over the 3 channels (if rgb)
%         residual = mean(residual);
%         residuals = [residuals; residual];
%         gradx_val = interpImg(img2_gradx, [x_trans, y_trans]);
%         grady_val = interpImg(img2_grady, [x_trans, y_trans]);
%         JIi = [ gradx_val, grady_val];
%         
%         x_world = transformed_points3d(1, i);
%         y_world = transformed_points3d(2, i);
%         z_world = transformed_points3d(3, i);
%         
%         Jwi = get_warp_jacobian(K, x_world, y_world, z_world);
%         Ji = JIi * Jwi;
%         J = [J; Ji];
%     end
%end