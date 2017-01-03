function [new_keypoints,new_landmarks,updated_candidate_keypoints,...
    updated_candidate_keypoints_1,updated_candidate_pose_1,updated_candidate_discard] = landmarkTriangulation(...
    K,current_pose,candidate_pose_1,candidate_keypoints,candidate_keypoints_1,candidate_discard)

%% Parameters
min_angle = 0.1; % Degrees 1
max_angle = 1.8; % Degrees 1.8
min_iterations = 0; % 2

penalty = inf; % Penalty for points triangulated behind the camera

%% Code

num_points = size(candidate_keypoints,2);

p1 = [candidate_keypoints_1; ones(1,num_points)];
p2 = [candidate_keypoints; ones(1,num_points)];

current_pose_inv = [current_pose(:,1:3)' -current_pose(:,1:3)'*current_pose(:,4)];

P = zeros(4,num_points);
behind_camera = false(1,num_points);

for j=1:num_points
    pose_1 = reshape(candidate_pose_1(:,j),[3,4]);
    pose_1_inv = [pose_1(:,1:3)' -pose_1(:,1:3)'*pose_1(:,4)];
    
    M2 = K*[eye(3) zeros(3,1)]*[current_pose_inv; zeros(1,3) 1]*[pose_1; zeros(1,3) 1];
    M1 = K*[eye(3) zeros(3,1)];
    
    % Build matrix of linear homogeneous system of equations
    A1 = cross2Matrix(p1(:,j))*M1;
    A2 = cross2Matrix(p2(:,j))*M2;
    A = [A1; A2];
    
    % Solve the linear homogeneous system of equations
    [~,~,v] = svd(A,0);
    P(:,j) = [pose_1; zeros(1,3) 1]*v(:,4)/v(4,4); % Extract and dehomogeneize (P is expressed in homogeneous coordinates)
    
    % Check for points triangulated behind the camera
    behind_camera(1,j) = current_pose_inv(3,:)*P(:,j) < 0 | pose_1_inv(3,:)*P(:,j) < 0;
end

% Compute angle and compare with min_angle
a = P(1:3,:) - repmat(current_pose(1:3,4),1,num_points);
b = P(1:3,:) - candidate_pose_1(10:12,:);
angle = acos(dot(a,b)./sqrt(dot(a,a).*dot(b,b)));
new = (min_angle < abs(angle)*180/pi) & (max_angle > abs(angle)*180/pi) & ~behind_camera & ~isinf(candidate_discard) & candidate_discard > min_iterations;

disp('New landmarks with associated angle (last row): ')
disp(num2str([P(:,new); 180/pi*angle(:,new)]))
disp(' ')

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