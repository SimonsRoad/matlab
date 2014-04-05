function [warped_img] = warpimgIntensity(img, points2d, transformed_points2d)
    x_size = size(img,2);
    y_size = size(img,1);
    bad_transformed = 0;
    warped_img = NaN(y_size, x_size);
    for i = 1:size(points2d, 2)
        x = points2d(1,i);
        y = points2d(2,i);
        x_trans = transformed_points2d(1,i);
        y_trans = transformed_points2d(2,i);
        if ~isfinite(x_trans) || ~isfinite(y_trans)
            bad_transformed = bad_transformed + 1;
            continue;
        end
        if x_trans > x_size || y_trans > y_size ...
            || x_trans < 1 || y_trans < 1
            bad_transformed = bad_transformed + 1;
            continue;
        end
      
        img_val = interpImg(img, [y_trans, x_trans]);
        warped_img(y,x) = img_val; 
    end
    warning('unable to warp %d pixels.', bad_transformed);
end

