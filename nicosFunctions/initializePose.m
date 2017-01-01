function [state] = initializePose(img_0, img_1, K )
%INITIALIZE creates initial state and pose when given first two images of
%dataset
%   input two images img0 and img1
% initialize(img0, img1) finds common harris corners and uses the 8-point
% algorithm with RANSAC in order to recover the world coordinate points of
% the landmarks


%% Parameters form exercise 3.
harris_patch_size = 9;
harris_kappa = 0.08;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 5;
num_keypoints = 1000;
num_it_ransac = 200;
pixel_tolerance = 1;

%% find harris corners and descriptors in both images
harris_0 = harris(img_0, harris_patch_size, harris_kappa);
keypoints_0 = selectKeypoints( harris_0, num_keypoints, nonmaximum_supression_radius);
descriptors_0 = describeKeypoints(img_0, keypoints_0, descriptor_radius);
harris_1 = harris(img_1, harris_patch_size, harris_kappa);
keypoints_1 = selectKeypoints( harris_1, num_keypoints, nonmaximum_supression_radius);
descriptors_1 = describeKeypoints(img_1, keypoints_1, descriptor_radius);

%% Match keypoints
all_matches = matchDescriptors(descriptors_0, descriptors_1, match_lambda);
[~, i_0, i_1] = find(all_matches);
keypoints_0 = flipud(keypoints_0);
keypoints_1 = flipud(keypoints_1);
p_0 = homogenize(keypoints_0(:,i_0));
p_1 = homogenize(keypoints_1(:,i_1));
max_num_inliers = 0; % start with zero inliers


%% Apply RANSAC
for i = 1:num_it_ransac
    
    % randomly pick 8 keypoints
    idx = datasample(1:length(p_0), 8, 2, 'Replace', false);
    keypoint_sample_0 = p_0(:, idx);
    keypoint_sample_1 = p_1(:, idx);
    
    % estimate fund Mat. from those 8 points
    F = fundamentalEightPoint_normalized(keypoint_sample_0, keypoint_sample_1);
    
    % calculate epipolar line distance
    errors = epipolarDistance(F,p_0,p_1);
    
    % count inliers
    is_inlier = errors < pixel_tolerance^2;
    if nnz(is_inlier) > max_num_inliers && nnz(is_inlier) >= 8
        max_num_inliers = nnz(is_inlier);
        inlier_mask = is_inlier;
        bestF=F;
    end
    max_num_inliers_history(i) = max_num_inliers;
end

%% Compute essential matrix from all inliers and recover R,T
if max_num_inliers == 0
    R = eye(3);
    T = [0;0;0];
    disp('could not initialize [R|T] matrix. Set to zero');
else
    E = estimateEssentialMatrix(p_0(:,inlier_mask), p_1(:,inlier_mask),K,K);
    [R,u] = decomposeEssentialMatrix(E);
    [R,T] = disambiguateRelativePose(R, u, p_0(:,inlier_mask), p_1(:,inlier_mask), K, K);
end

%% Initialize state
state = struct;
state.pose = [R', -R*T];
state.landmarks = linearTriangulation(p_0(:,inlier_mask), p_1(:,inlier_mask),K*eye(3,4),K*[R,T]);
state.keypoints = p_1(1:2,inlier_mask);
state.candidate_keypoints = [];
state.candidate_keypoints_1=[];
state.candidate_pose_1=[];
state.K = K;
state.discard = zeros(1,size(state.landmarks,2));
state.candidate_discard = zeros(1,size(state.candidate_keypoints,2));

%% Debug statements
% state.pose
% disp('found R and T with num inliers: ');
% max_num_inliers
% figure
% plot(max_num_inliers_history)
% figure
% imshow(img_0);
% hold on
% x_from = p_0(1,inlier_mask);
% x_to = p_1(1,inlier_mask);
% y_from = p_0(2,inlier_mask);
% y_to = p_1(2,inlier_mask);
% plot([x_from; x_to], [y_from; y_to], 'g-', 'Linewidth', 3);

end

