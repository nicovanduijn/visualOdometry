function [new_keypoints,new_landmarks,updated_candidate_keypoints,...
    updated_candidate_keypoints_1,updated_candidate_pose_1,updated_candidate_discard,new] = landmarkTriangulation(...
    K,current_pose,candidate_pose_1,candidate_keypoints,candidate_keypoints_1,candidate_discard)

%% Parameters
global params;
min_angle = params.triang_min_angle; % Degrees 
max_angle = params.triang_max_angle; % Degrees
min_iterations = params.triang_min_iterations; 
max_reprojection_error = params.triang_max_reprojection_error; %
penalty = params.triang_penalty; % Penalty for points triangulated behind the camera

%% Code

num_points = size(candidate_keypoints,2);

p1 = [candidate_keypoints_1; ones(1,num_points)];
p2 = [candidate_keypoints; ones(1,num_points)];

current_pose_inv = invertPose(current_pose);
M2 = K*current_pose_inv;
 
P = zeros(4,num_points);
behind_camera = false(1,num_points);

repro_reliable= false(1,num_points);
for j=1:num_points
    pose_1 = reshape(candidate_pose_1(:,j),[3,4]);
    pose_1_inv = invertPose(pose_1);
    M1 = K*pose_1_inv;
     
    % Build matrix of linear homogeneous system of equations
    A1 = cross2Matrix(p1(:,j))*M1;
    A2 = cross2Matrix(p2(:,j))*M2;
    A = [A1; A2];
    
    % Solve the linear homogeneous system of equations
    [~,~,v] = svd(A,0);
    P(:,j) = v(:,4)/v(4,4); % Extract and dehomogenize (P is expressed in homogeneous coordinates)
      
    % Check for points triangulated behind the camera
    behind_camera(1,j) = current_pose_inv(3,:)*P(:,j) < 0 | pose_1_inv(3,:)*P(:,j) < 0;
    
    if reprojectionError(p1(1:2,j),P(:,j),M1) < max_reprojection_error &&...
            reprojectionError(p2(1:2,j),P(:,j),M2) < max_reprojection_error
        repro_reliable(j) = true;
    end
end

% Compute angle and compare with min_angle
a = P(1:3,:) - repmat(current_pose(1:3,4),1,num_points);
b = P(1:3,:) - candidate_pose_1(10:12,:);
angle = acos(dot(a,b)./sqrt(dot(a,a).*dot(b,b)));
angle_reliable = (min_angle < abs(angle)*180/pi) & (max_angle > abs(angle)*180/pi);

new = angle_reliable & repro_reliable & ~behind_camera & ~isinf(candidate_discard) & candidate_discard > min_iterations;

% Output
new_keypoints = candidate_keypoints(:,new);
new_landmarks = P(:,new);
updated_candidate_keypoints = candidate_keypoints(:,~new);
updated_candidate_keypoints_1 = candidate_keypoints_1(:,~new);
updated_candidate_pose_1 = candidate_pose_1(:,~new);
updated_candidate_discard = candidate_discard;
updated_candidate_discard(:,behind_camera) = penalty;
updated_candidate_discard = updated_candidate_discard(:,~new);

end