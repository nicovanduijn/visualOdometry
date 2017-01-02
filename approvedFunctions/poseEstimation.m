function [ pose, discard] = poseEstimation(curr_keyp, landmarks, K, discard)
%POSEESTIMATION Estimates the pose of the camera using P3P and RANSAC
%   Takes in
% - curr_keyp: 2xN [u;v] describing keypoint location
% - landmarks: 4xN [x;y;z] describing corresponding world points
% - K: [3x3] kamera matrix
% - discard 1xN voting array

num_iterations = 200;
pixel_tolerance = 10;

matched_query_keypoints = curr_keyp(:,discard==0);
corresponding_landmarks = landmarks(1:3,discard==0); %de-homogenize

% Initialize RANSAC.
inlier_mask = zeros(1, size(matched_query_keypoints, 2));
max_num_inliers_history = zeros(1, num_iterations);
max_num_inliers = 0;

% RANSAC
for i = 1:num_iterations
    [landmark_sample, idx] = datasample(...
        corresponding_landmarks, 3, 2, 'Replace', false);
    keypoint_sample = matched_query_keypoints(:, idx);
    
    normalized_bearings = K\[keypoint_sample; ones(1, 3)];
    for ii = 1:3
        normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
            norm(normalized_bearings(:, ii), 2);
    end
    poses = p3p(landmark_sample, normalized_bearings);
    R_C_W_guess = zeros(3, 3, 2);
    t_C_W_guess = zeros(3, 1, 2);
    for ii = 0:1
        R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
        t_W_C_ii = real(poses(:, (1+ii*4)));
        R_C_W_guess(:,:,ii+1) = R_W_C_ii';
        t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
    end
    
    
    % Count inliers:
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,1) * corresponding_landmarks) + ...
        repmat(t_C_W_guess(:,:,1), ...
        [1 size(corresponding_landmarks, 2)]), K);
    difference = matched_query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;
    
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,2) * corresponding_landmarks) + ...
        repmat(t_C_W_guess(:,:,2), ...
        [1 size(corresponding_landmarks, 2)]), K);
    difference = matched_query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    alternative_is_inlier = errors < pixel_tolerance^2;
    if nnz(alternative_is_inlier) > nnz(is_inlier)
        is_inlier = alternative_is_inlier;
    end
    
    
    if nnz(is_inlier) > max_num_inliers && nnz(is_inlier) >= 6
        max_num_inliers = nnz(is_inlier);
        inlier_mask = is_inlier;
    end
    
    max_num_inliers_history(i) = max_num_inliers;
end

if max_num_inliers == 0
    R_C_W = [];
    t_C_W = [];
else
    M_C_W = estimatePoseDLT(...
        matched_query_keypoints(:, inlier_mask>0)', ...
        corresponding_landmarks(:, inlier_mask>0)', K);
    R_C_W = M_C_W(:, 1:3);
    t_C_W = M_C_W(:, end);
end

pose = [R_C_W', -R_C_W*t_C_W];
discard(~inlier_mask)=discard(~inlier_mask)+inf;

end

