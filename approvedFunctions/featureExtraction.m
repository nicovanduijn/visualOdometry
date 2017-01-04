function [new_candidate_keypoints,new_candidate_keypoints_1,new_candidate_pose_1] = featureExtraction(...
    current_image,current_keypoints,new_keypoints,candidate_keypoints,current_pose,discard,candidate_discard)

%% Parameters
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 80;
nonmaximum_supression_radius = 8;
min_distance = 8; % Minimum distance new keypoints must have w.r.t. existing ones

%% Code
% tic
% harris_scores = harris(current_image, harris_patch_size, harris_kappa);
% harris_keypoints = flipud(selectKeypoints(harris_scores, num_keypoints, nonmaximum_supression_radius));
% 
% existing_keypoints = [current_keypoints(:,discard == 0) new_keypoints candidate_keypoints(:,candidate_discard == 0)];
% 
% distances = pdist2(harris_keypoints',existing_keypoints');
% 
% new_candidate_keypoints = harris_keypoints(:,min(distances,[],2) > min_distance);
% new_candidate_keypoints_1 = new_candidate_keypoints;
% new_candidate_pose_1 = repmat(current_pose(:),1,size(new_candidate_keypoints,2));
% toc
%% Code with Matlab function
tic
harris_scores = detectHarrisFeatures(current_image);

strongest_harris_scores_struct = harris_scores.selectStrongest(num_keypoints);

strongest_harris_scores = double(strongest_harris_scores_struct.Location');

existing_keypoints = [current_keypoints(:,discard == 0) new_keypoints candidate_keypoints(:,candidate_discard == 0)];

distances = pdist2(strongest_harris_scores',existing_keypoints');

new_candidate_keypoints = strongest_harris_scores(:,min(distances,[],2) > min_distance);
new_candidate_keypoints_1 = new_candidate_keypoints;
new_candidate_pose_1 = repmat(current_pose(:),1,size(new_candidate_keypoints,2));
toc
end