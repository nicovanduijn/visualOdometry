function [adjusted_landmarks,current_landmarkBundleAdjustment_struct] = landmarkBundleAdjustment(...
    K,current_pose,current_keypoints,new_keypoints,landmarks,new_landmarks,previous_landmarkBundleAdjustment_struct,del)
% Inputs:
%   - previous_landmark_keypoints: 2 x num_landmarks x max _observations
%   - previous_landmark_poses : 12 x num_landmarks x max_observations


%% Parameters

min_observations = 1;
max_observations = 10;

%% Code

num_landmarks = size(landmarks,2) - sum(del);
num_new_landmarks = size(new_landmarks,2);
num_adjusted_landmarks = size(landmarks,2) - sum(del) + size(new_landmarks,2);

if isempty(fieldnames(previous_landmarkBundleAdjustment_struct))
    previous_landmarkBundleAdjustment_struct.previous_landmark_poses = NaN(12,num_landmarks + sum(del),max_observations);
    previous_landmarkBundleAdjustment_struct.previous_keypoints = NaN(2,num_landmarks + sum(del),max_observations);
    previous_landmarkBundleAdjustment_struct.counter = zeros(1,num_landmarks + sum(del));
end

previous_landmark_poses = previous_landmarkBundleAdjustment_struct.previous_landmark_poses(:,~del,:);
previous_keypoints = previous_landmarkBundleAdjustment_struct.previous_keypoints(:,~del,:);
counter = previous_landmarkBundleAdjustment_struct.counter(:,~del);

p1 = [current_keypoints(:,~del); ones(1,num_landmarks)];
current_pose_inv = invertPose(current_pose);
M1 = K*current_pose_inv;

landmarks = landmarks(:,~del);

P = NaN(4,num_landmarks);
adjusted = false(1,num_landmarks);
for j=1:num_landmarks
    if counter(j) >= min_observations
        A1 = cross2Matrix(p1(:,j))*M1;
        
        A2 = zeros(3*counter(j),4);
        for i = 1:min(counter(j),max_observations)
            M2 = K*invertPose(reshape(previous_landmark_poses(:,j),[3,4]));
            A2((i-1)*3+1:i*3,:) = cross2Matrix([previous_keypoints(:,j,i); 1])*M2;
        end
        
        A = [A1; A2];
        [~,~,v] = svd(A,0);
        P(:,j) = v(:,4)/v(4,4); % Extract and dehomogenize (P is expressed in homogeneous coordinates)
        
        if current_pose_inv(3,:)*P(:,j) < 0
            P(:,j) = landmarks(:,j);
        else
            adjusted(j) = true;
        end
    else
        P(:,j) = landmarks(:,j);
    end
end

adjusted_landmarks = [P new_landmarks];

figure(10)
plot(landmarks(1,~adjusted),landmarks(3,~adjusted), 'go')
hold on
plot(landmarks(1,adjusted),landmarks(3,adjusted), 'mo')
hold on
x_from = landmarks(1,adjusted);
x_to = P(1,adjusted);
z_from = landmarks(3,adjusted);
z_to = P(3,adjusted);
plot([x_from; x_to], [z_from; z_to], 'm-', 'Linewidth', 1);
hold on
plot(P(1,adjusted),P(3,adjusted), 'bx')
axis equal
hold off

%del
%adjusted
disp(['Maximum value in counter: ' num2str(max(counter))])

% Output
current_landmarkBundleAdjustment_struct.previous_landmark_poses = cat(3, repmat(current_pose(:),[1,num_adjusted_landmarks,1]), cat(2,previous_landmark_poses(:,:,1:max_observations-1),NaN(12,num_new_landmarks,max_observations-1)));
current_landmarkBundleAdjustment_struct.previous_keypoints = cat(3, cat(2,current_keypoints(:,~del),new_keypoints), cat(2,previous_keypoints(:,:,1:max_observations-1),NaN(2,num_new_landmarks,max_observations-1)));
current_landmarkBundleAdjustment_struct.counter = [counter zeros(1,num_new_landmarks)] + 1;

end