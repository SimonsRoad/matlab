function [pose] = sparse_tracking_ic(K, img1_pyr, img2_pyr, depth1_pyr, initial_guess)
    k_max = 50;
    epsilon = 1;
    % fixed # patches
    patches_centers = 100;

    guess = initial_guess;
    
    levels = size(img1_pyr,1);
            
    for level = levels:-1:1
        guess
        
        increment = double([0;0;0;0;0;0]);
    
        level
        
        % variable # patches / level
        %patches_centers = 400 / level;
        
        img1 = img1_pyr{level};
        img1_dbl = double(img1);
        img1_depth = depth1_pyr{level}; 
        img2 = img2_pyr{level};
        
        % get regularly spaced out centers for now.
        centers = get_patches_centers(img1, patches_centers);
        [ patches, patch_img ] = get_patches(K, img1_dbl, img1_depth, centers);
        
        k = 1;
        
        error = inf;
        error_last = 0;
        accept = true;
        while accept && (k == 1 || (error_last - error) > epsilon) && (k < k_max)
            increment = twistexp(increment);
            new_guess = increment * guess;
            
            J = [];
            all_residuals = [];
            
            for l = 1:size(patches, 2)
                transformed_p2d = warpAndProject(patches(l).points3d, K, new_guess);
                warped_patch = get_warped_patch(img2, transformed_p2d);
                
                residuals = warped_patch - patches(l).rpatch;
                
                % keep only the Jacobians and Residuals that
                % are both finite (non-NaN)
                
                for i = 1:size(residuals)
                    if sum(isfinite(patches(l).jacobi(i,:))) == 6 ...
                        && isfinite(residuals(i))
                        J = [J; patches(l).jacobi(i,:) ];
                        all_residuals = [all_residuals; residuals(i)];
                    end
                end
            end
            
            % error 
            error_last = error;
            error = (all_residuals'*all_residuals) / size(all_residuals,1)
    
            % Hessian matrix 
            H = J' * J;
    
            % steepest descent
            steepest_descent = -J'*all_residuals;
    
            % compute the increment :
            increment = H \ steepest_descent;
            
            if error > error_last 
                % the error is worse, try the next pyramid level
                accept = false;
            else
                guess = new_guess;
            end
            
            k = k+1;            
        end        
    end
    
    error
    pose = guess;
end

function [ points2d ] = get_patches_centers(img, n_patches)
    % linearly separated 2d points in an image
    img_height = size(img, 1);
    img_width = size(img, 2);
    area = (img_height * img_width) / n_patches;
    side = sqrt(area);
    height = side * img_height/img_width;
    width = side * img_width/img_height;
    startx = width/2;
    starty = height/2;
    [x, y] = meshgrid(startx:width:img_width, starty:height:img_height);
    points2d = NaN(3, size(x, 1) * size(x,2));
    k = 1;
    for i=1:size(x,1)
        for j=1:size(x,2)
            points2d(:, k) = [x(i,j); y(i,j); 1];
            k = k + 1;
        end
    end
end

function [ patch_list, patch_img ] = get_patches(K, img, img_depth, patches_centers)
    patch_img = zeros(size(img));
    k = 1;
    height = size(img, 1);
    width = size(img, 2);
    for p = 1:size(patches_centers, 2)
        % XXX (mtourne): +0.1 to make sure we don't land on an integer.
        % (otherwise floor and ceil give the same pixel).
        x = patches_centers(1, p) + 0.1;
        y = patches_centers(2, p) + 0.1;
        x0 = floor(x);
        x1 = ceil(x);
        y0 = floor(y);
        y1 = ceil(y);
        if (x0 - 1) < 1 || (x1 + 1) > width || (y0 - 1) < 1 || (y1 + 1) > height
            continue
        end
        points2d = NaN(3, 16);
        l = 1;
        % XXX(mtourne) : Super innefficient !
        for i = (x0-1):(x1+1)
            for j = (y0-1):(y1+1)
                points2d(:, l) = [ i; j; 1];
                l = l + 1;
            end
        end
        patch = img(y0-1:y1+1,x0-1:x1+1);
        patch_img(y0-1:y1+1,x0-1:x1+1) = patch;
        patch_list(k).patch = patch;
        patch_list(k).rpatch = reshape(patch, 16, 1);
        patch_list(k).points2d = points2d;
        points3d = get_3d_from_2d(K, img_depth, points2d);
        patch_list(k).points3d = points3d;
        patch_list(k).jacobi = compute_template_jacobians(K, patch, points2d, points3d);
        k = k+1;
    end
end

function [ points3d, i ] = get_3d_from_2d(K, depth_img, points2d)
    % get 3d points p from a list of 2d points + depth image
    fx = K(1,1);
    cx = K(1,3);
    fy = K(2,2);
    cy = K(2,3);
    n = size(points2d, 2);
    points3d = NaN(4, n);
    for i = 1:n
        x = points2d(1, i);
        y = points2d(2, i);
        % y is rows, x is columns
        depth = depth_img(y, x);
        if depth == 0
            continue;
        end
        % similar to K^-1 * (point2d, z(point))
        point3d = [(x - cx)/fx ; (y - cy)/fy; 1; 0] .* depth;
        point3d(4) = 1;
        points3d(:, i) = point3d;
    end
end

function [J] = compute_template_jacobians(K, patch, points2d, points3d) 
    [ gradx, grady ] = imgradientxy(patch, 'CentralDifference'); 
    orig_x = points2d(1,1);
    orig_y = points2d(2,1);
    
    n = size(points2d,2);
    J = NaN(16,6);
    for i = 1:n
        x = points2d(1,i);
        y = points2d(2,i);
        
        x_world = points3d(1, i);
        y_world = points3d(2, i);
        z_world = points3d(3, i);
        
        if ~isfinite(x_world) || ~isfinite(y_world) || ~isfinite(z_world)
            continue;
        end
        
        Jwi = get_warp_jacobian(K, x_world, y_world, z_world);
        
        gradx_val = gradx(y - orig_y + 1, x - orig_x + 1);
        grady_val = grady(y - orig_y + 1, x - orig_x + 1);
        JIi = [ gradx_val, grady_val ];
        
        Ji = JIi * Jwi;
        J(i,:) = Ji';
    end
end

function [ warped_patch ] = get_warped_patch(img, points2d)
    % get patch reshaped 16, 1 (intstead of 4x4)
    warped_patch = NaN(16,1);
    for i = 1:size(points2d,2)
        if ~isfinite(points2d(1,i)) || ~isfinite(points2d(2,i))
            continue;
        end
        warped_patch(i,1) = bilinearInterpol(img, points2d(:,i));
    end
end