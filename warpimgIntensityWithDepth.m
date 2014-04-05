function [warped_img] = warpimgIntensityWithDepth(img, img_ref_depth, points2d, ...
                                                  transformed_points2d, transformed_points3d)
    x_size = size(img,2);
    y_size = size(img,1);
    bad_transformed = 0;
    warped_img = NaN(y_size, x_size);
    
    for i = 1:size(points2d, 2)
        x = points2d(1,i);
        y = points2d(2,i);
        x_trans = transformed_points2d(1,i);
        y_trans = transformed_points2d(2,i);
        z_trans = transformed_points3d(3,i);
        if ~isfinite(x_trans) || ~isfinite(y_trans) || ~isfinite(z_trans)
            bad_transformed = bad_transformed + 1;
            continue;
        end
       
        %img_val = bilinearInterpolWithDepth(img, img_ref_depth, [x_trans, y_trans], z_trans);
        img_val = bilinearInterpol(img, [x_trans, y_trans]);
        if ~isfinite(img_val)
            bad_transformed = bad_transformed +1;
            continue
        end
        warped_img(y,x) = img_val; 
    end
    warning('unable to warp %d pixels.', bad_transformed);
    
end

function [img_val] = bilinearInterpol(img, point2d)
    % same as below, except don't use depth
    x_size = size(img,2);
    y_size = size(img,1);
    
    x = point2d(1);
    y = point2d(2);
    
    x0 = floor(x);
    y0 = floor(y);
    x1 = x0 + 1;
    y1 = y0 + 1;
    
    x0_weight = x - x0;
    y0_weight = y - y0;
    x1_weight = 1 - x0_weight;
    y1_weight = 1 - y0_weight;
    
    val = 0;
    sum = 0;
    
    if (x0 < 1) || (x1 > x_size) ...
       || (y0 < 1) || (y1 > y_size)
        img_val = NaN;
        return
    end
    
    val = val + img(y0, x0) * x0_weight * y0_weight;
    sum = sum + x0_weight * y0_weight;
    
    val = val + img(y0, x1) * x1_weight * y0_weight;
    sum = sum + x1_weight * y0_weight;
    
    val = val + img(y1, x0) * x0_weight * y1_weight;
    sum = sum + x0_weight * y1_weight;
    
    val = val + img(y1, x1) * x1_weight * y1_weight;
    sum = sum + x1_weight * y1_weight;
    
    img_val = double(val) / sum;
end

function [img_val] = bilinearInterpolWithDepth(img, img_depth, point2d, z)
    x_size = size(img,2);
    y_size = size(img,1);
    
    x = point2d(1);
    y = point2d(2);
    
    x0 = floor(x);
    y0 = floor(y);
    x1 = x0 + 1;
    y1 = y0 + 1;
    
    x0_weight = x - x0;
    y0_weight = y - y0;
    x1_weight = 1 - x0_weight;
    y1_weight = 1 - y0_weight;
    z_eps = z - 0.05;
    
    val = 0;
    sum = 0;
    
    if (x0 < 1) || (x1 > x_size) ...
       || (y0 < 1) || (y1 > y_size)
        img_val = NaN;
        return
    end
    
    if isfinite(img_depth(y0, x0)) && img_depth(y0, x0) > z_eps
        val = val + img(y0, x0) * x0_weight * y0_weight;
        sum = sum + x0_weight * y0_weight;
    end
    
    if isfinite(img_depth(y0, x1)) && img_depth(y0, x1) > z_eps
        val = val + img(y0, x1) * x1_weight * y0_weight;
        sum = sum + x1_weight * y0_weight;
    end
    
    if isfinite(img_depth(y1, x0)) && img_depth(y1, x0) > z_eps
        val = val + img(y1, x0) * x0_weight * y1_weight;
        sum = sum + x0_weight * y1_weight;
    end
    
    if isfinite(img_depth(y1, x1)) && img_depth(y1, x1) > z_eps
        val = val + img(y1, x1) * x1_weight * y1_weight;
        sum = sum + x1_weight * y1_weight;
    end
    
    if sum > 0
        img_val = double(val) / sum;
    else
        img_val = NaN;
    end
end