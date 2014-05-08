% dense tracking test %

close all;

K = [ 525, 0, 319.5;
      0, 525, 239.5;
      0, 0, 1];

%% XYZ test %%
img1 = imread('data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.211214.png');
% depth for img1
% 1305031102.211214 rgb/1305031102.211214.png 1305031102.226738 depth/1305031102.226738.png
depth_img = imread('data/rgbd_dataset_freiburg1_xyz/depth/1305031102.226738.png');


img2 = imread('data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.275326.png');
%img2 = imread('data/rgbd_dataset_freiburg1_xyz/rgb/1305031102.411258.png');

%figure, imshow(img1);
%figure, imshow(img2);


%% ground truth
% pose1
% 1305031102.211214 rgb/1305031102.211214.png 1305031102.215900 1.3303 0.6256 1.6464 0.6579 0.6161 -0.2932 -0.3189
q1 = [ 0.6579; 0.6161; -0.2932; -0.3189 ];
t1 = [ 1.3303; 0.6256; 1.6464 ];

% pose2
% 1305031102.275326 rgb/1305031102.275326.png 1305031102.275800 1.3160 0.6254 1.6302 0.6609 0.6199 -0.2893 -0.3086
q2 = [ 0.6609; 0.6199; -0.2893; -0.3086 ];
t2 = [ 1.3160; 0.6254; 1.6302 ];
% 1305031102.411258 rgb/1305031102.411258.png 1305031102.415900 1.2830 0.6257 1.5933 0.6614 0.6249 -0.2903 -0.2962
%q2 = [ 0.6614; 0.6249; -0.2903; -0.2962 ];
%t2 = [ 1.2830; 0.6257; 1.5933 ];

initial_pose = quaternion2matrix(q1);
initial_pose(1:3, 4) = t1;

new_pose = quaternion2matrix(q2);
new_pose(1:3, 4) = t2;

rel_pose = get_relative_pose(initial_pose, new_pose);
inv_rel_pose = get_relative_pose(new_pose, initial_pose);

%depth_factor = 1/5000;
%depth_factor = 1;
%depth_factor = 0.001;

% normalize depth so it ranges from 0 to 255 like intensity
depth_factor = 255 / double(max(max(depth_img)));

depth_img = double(depth_img) * depth_factor;

pyr1 = get_pyramid(img1, 'median', 4);
pyr2 = get_pyramid(img2, 'median', 4);
%pyr1 = get_pyramid(img1, 'mean', 4);
%pyr2 = get_pyramid(img2, 'mean', 4);

depth_pyr = get_pyramid(depth_img, 'median', 4);
%depth_pyr = get_pyramid(depth_img, 'median2', 4);


%% test warp 
% tic
% [points2d, points3d] = get_3d_points(K, depth_pyr{2}, size(pyr1{2}, 1), size(pyr1{2}, 2));
% toc
% tic
% [transformed_points2d, transformed_points3d ] = warpAndProject(points3d, K, inv_rel_pose);
% toc
% tic
% warped_img2_on_img1 = warpimgIntensity(pyr2{2}, points2d, transformed_points2d);
% toc
% error_img = uint8(abs(warped_img2_on_img1 - double(pyr1{2})));
% tic
% warped_img2_on_img1_depth = warpimgIntensityWithDepth(pyr2{2}, depth_pyr{2}, points2d, ...
%                                         transformed_points2d, transformed_points3d);
% toc
% 
% error_img2 = uint8(abs(warped_img2_on_img1_depth - double(pyr1{2})));


%error_img_initial = uint8(abs(double(pyr2{2}) - double(pyr1{2})));
%figure, imshow(error_img);
%figure, imshow(error_img_initial);

% identity test
%[transformed_points2d, transformed_points3d ] = warpAndProject(points3d, K, eye(4,4));
%warped_img1_on_img1 = warpimgIntensity(pyr1{2}, points2d, transformed_points2d);
%error_img_identity = uint8(abs(warped_img1_on_img1 - double(pyr1{2})));
%figure, imshow(error_img_identity);

%% run dense tracking
%grad_pyr = get_grad_pyr(pyr1);

% initial transformation
initial_guess = eye(4,4);

tic
guessed_pose_sic = sparse_tracking_ic(K,  pyr1, pyr2, depth_pyr, initial_guess)
guessed_pose = guessed_pose_sic;
toc

%guessed_pose_ic = dense_tracking_inv_compositional(K, pyr1, pyr2, depth_pyr, initial_guess)
%guessed_pose = guessed_pose_ic;

%guessed_pose_fwd = dense_tracking_fwd_additive(K, pyr1, pyr2, depth_pyr, initial_guess)
%guessed_pose = guessed_pose_fwd;

% guess the identity function
% guessed_pose = dense_tracking_fwd_additive(K, pyr1, pyr1, depth_pyr, initial_guess)


pyr_idx = 1;

[points2d, points3d] = get_3d_points(K, depth_pyr{pyr_idx}, size(pyr1{pyr_idx}, 1), size(pyr1{pyr_idx}, 2));
[transformed_points2d, transformed_points3d ] = warpAndProject(points3d, K, guessed_pose);
warped_img2_on_img1_guessed = warpimgIntensityWithDepth(pyr2{pyr_idx}, depth_pyr{pyr_idx}, ...
                                                        points2d, transformed_points2d,...
                                                        transformed_points3d);
error_img = abs(warped_img2_on_img1_guessed - double(pyr1{pyr_idx}));
%norm(error_img)
    
imshow(uint8(error_img));


