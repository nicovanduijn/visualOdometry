function [state] = initializePose(img_0, img_1, K )
%INITIALIZEPOSE creates initial state and pose when given first two images of
%dataset
%   input two images img0 and img1 and calibration matrix
% initialize(img0, img1, K) finds common harris corners and uses the 8-point
% algorithm with RANSAC in order to recover the world coordinate points of
% the landmarks

%% Parameters
global params;
harris_patch_size = params.init_harris_patch_size;
harris_kappa = params.init_harris_kappa;
nonmaximum_supression_radius = params.init_nonmaximum_supression_radius;
descriptor_radius = params.init_descriptor_radius;
match_lambda = params.init_match_lambda;
num_keypoints = params.init_num_keypoints;
num_it_ransac = params.init_num_it_ransac;
pixel_tolerance = params.init_pixel_tolerance;

%% Find harris corners and descriptors in both images
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
        % bestF=F;
    end
    % max_num_inliers_history(i) = max_num_inliers;
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
state.pose = [R', -R'*T];
state.landmarks = linearTriangulation(p_0(:,inlier_mask), p_1(:,inlier_mask),K*eye(3,4),K*[R,T]);
del =  state.landmarks(3,:)<0; % delete points behind camera
state.landmarks(:,del) = [];
state.keypoints = p_1(1:2,inlier_mask);
state.keypoints(:,del)=[];
state.candidate_keypoints = zeros(2,0);
state.candidate_keypoints_1 = zeros(2,0);
state.candidate_pose_1 = zeros(12,0);
state.K = K;
state.discard =zeros(1,size(state.landmarks,2));
state.candidate_discard = zeros(1,size(state.candidate_keypoints,2));
state.previous_keypoints = zeros(2,0);
state.new_candidate_keypoints = zeros(2,0);
state.init_counter = 0;

end

