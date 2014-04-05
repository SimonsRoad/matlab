function [ transformed_points2d, transformed_points3d ] = warpAndProject(points3d, K, P)
    % warp 3d points and project onto an image, using a 4x4 matrix P
    % convert K into a 3*4 matrix, to get a 3*n result (2d homogeneous
    % points)
    K = [K [0; 0; 0;]];
    % transform points 3d with a rotation and translation 
    transformed_points3d = P * points3d;
    % project 3d points onto the image plane.
    transformed_points2d = K * transformed_points3d;
    % get homogeneous coordinates
    homogeneize = @(col) col(:)/col(3);
    transformed_points2d = apply_to_columns(homogeneize, transformed_points2d);
end