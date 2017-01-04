function [current_state] = processFrame(previous_state, previous_image, current_image)
%PROCESSFRAME Processes a frame
%   Inputs:
%   - previous_state: Struct with fields
%       * landmarks: 4xN (homogeneous coordinates)
%       * keypoints: 2xN
%       * candidate_keypoints: 2xM
%       * candidate_keypoints_1: 2xM
%       * candidate_pose_1: 12xM
%       * pose: 3x4
%       * K: 3x3
%       * discard 1xN
%       * candidate_discard 1xM
%       * init_counter 1x1
%   - previous_image: XxY
%   - current_image: XxY
%
%   Outputs:
%   - current_state: Struct
%   - current_pose: 4x3 non-homogenous matrix representing pose [R|T]

%% Paramters
global params;
discard_max = params.proc_discard_max; % Points with a higher vote are discarded (currently: random choice)
candidate_discard_max = params.proc_candidate_discard_max; % Points with a higher vote are discarded (currently: random choice)
min_keypoint_threshold = params.proc_min_keypoint_threshold;
re_init_after = params.proc_re_init_after;

%% Preliminary stuff

discard = previous_state.discard;
candidate_discard = previous_state.candidate_discard;
K = previous_state.K;

%% Apply KLT on current_image

% disp(['Number of inf values in discard at beginning: ' num2str(sum(discard == inf))])

[current_keypoints,current_candidate_keypoints,discard,candidate_discard] = keypointTracking(previous_state.keypoints,...
    previous_state.candidate_keypoints,previous_image,current_image,discard,candidate_discard);

% disp(['Number of inf values in discard after KLT: ' num2str(sum(discard == inf))])

%% Apply P3P + RANSAC on keypoints with an associated landmark
[current_pose,discard] = poseEstimation(current_keypoints, previous_state.landmarks, previous_state.K, discard);

% disp(['Number of inf values in discard after P3P: ' num2str(sum(discard == inf))])

%% Check if number of keypoints too low
if(sum(discard==0)<=min_keypoint_threshold || previous_state.init_counter >= re_init_after)
    current_state = initializePose(previous_image, current_image, K);
    current_state.pose = previous_state.pose* [current_state.pose; 0 0 0 1];
    current_state.landmarks = [previous_state.pose; 0 0 0 1] * current_state.landmarks;
    disp('lost tracking! re-initializing VO pipeline...');
    
    current_state.landmarkBundleAdjustment_struct = struct();
    
    % Look for new candidate_keypoints and keep old candidate_keypoints

    [new_candidate_keypoints,new_candidate_keypoints_1,new_candidate_pose_1] = featureExtraction(...
        current_image,current_state.keypoints,zeros(2,0),previous_state.candidate_keypoints,current_state.pose,current_state.discard,candidate_discard);
    
    candidate_discard = candidate_discard + 1; % Penalty for 'old' candidate features
    candidate_del = candidate_discard > candidate_discard_max;
    current_state.new_candidate_keypoints = new_candidate_keypoints; % For plotting only
    current_state.candidate_keypoints = [current_candidate_keypoints(:,~candidate_del) new_candidate_keypoints];
    current_state.candidate_keypoints_1 = [previous_state.candidate_keypoints_1(:,~candidate_del) new_candidate_keypoints_1];
    current_state.candidate_pose_1 = [previous_state.candidate_pose_1(:,~candidate_del) new_candidate_pose_1];
    current_state.candidate_discard = [candidate_discard(:,~candidate_del) zeros(1,size(new_candidate_keypoints,2))];
else

%% Apply linear triangulation on keypoints without associated landmark

[new_keypoints,new_landmarks,updated_candidate_keypoints,updated_candidate_keypoints_1,...
    updated_candidate_pose_1,candidate_discard] = landmarkTriangulation(...
    K,current_pose,previous_state.candidate_pose_1,current_candidate_keypoints,...
    previous_state.candidate_keypoints_1,candidate_discard);

%% Find new features
%  - Perform suppression around existing features?
%  - How about FAST instead of Harris?
%  - Do this in every iteration or only once in a while?
%  - Note: new_candidate_keypoints = new_candidate_keypoints_1!
[new_candidate_keypoints,new_candidate_keypoints_1,new_candidate_pose_1] = featureExtraction(...
    current_image,current_keypoints,new_keypoints,updated_candidate_keypoints,current_pose,discard,candidate_discard);

%% What is left to do
disp(['Number of new keypoints: ' num2str(size(new_keypoints,2))])
disp(['Number of keypoints: ' num2str(size(current_keypoints,2))])
% disp(['Any NaN values in candidate_discard: ' num2str(any(isnan(candidate_discard)))])
% disp(['Any NaN values in discard: ' num2str(any(isnan(discard)))])

candidate_discard = candidate_discard + 1; % Penalty for 'old' candidate features

del = discard > discard_max;
candidate_del = candidate_discard > candidate_discard_max;

% ------
% Landmarks only bundle adjustment
[current_state.landmarks,current_landmarkBundleAdjustment_struct] = landmarkBundleAdjustment(...
    K,current_pose,current_keypoints,new_keypoints,previous_state.landmarks,new_landmarks,previous_state.landmarkBundleAdjustment_struct,del);

current_state.landmarkBundleAdjustment_struct = current_landmarkBundleAdjustment_struct;
% ------

% current_state.landmarks = [previous_state.landmarks(:,~del) new_landmarks];
current_state.keypoints = [current_keypoints(:,~del) new_keypoints];
current_state.previous_keypoints = previous_state.keypoints(:,~del); % For plotting only
current_state.discard = [discard(:,~del) zeros(1,size(new_keypoints,2))];
current_state.candidate_keypoints = [updated_candidate_keypoints(:,~candidate_del) new_candidate_keypoints];
current_state.new_candidate_keypoints = new_candidate_keypoints; % For plotting only
current_state.candidate_keypoints_1 = [updated_candidate_keypoints_1(:,~candidate_del) new_candidate_keypoints_1];
current_state.candidate_pose_1 = [updated_candidate_pose_1(:,~candidate_del) new_candidate_pose_1];
current_state.candidate_discard = [candidate_discard(:,~candidate_del) zeros(1,size(new_candidate_keypoints,2))];
current_state.pose = current_pose;
current_state.K = K;
current_state.init_counter = previous_state.init_counter + 1;

%  disp(['Any current_keypoints smaller than zero: ', num2str(any(current_state.keypoints(:) < 0))])
%  disp(['Any candidate_keypoints smaller than zero: ', num2str(any(current_state.candidate_keypoints(:) < 0))])
 disp('Current pose: ')
 disp(num2str(current_pose))
end

end


