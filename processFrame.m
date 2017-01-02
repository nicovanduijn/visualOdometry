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
%   - previous_image: XxY
%   - current_image: XxY
%
%   Outputs:
%   - current_state: Struct
%   - current_pose: 4x3 non-homogenous matrix representing pose [R|T]

%% Paramters

discard_max = 10; % Points with a higher vote are discarded (currently: random choice)
candidate_discard_max = 10; % Points with a higher vote are discarded (currently: random choice)

%% Preliminary stuff

discard = previous_state.discard;
candidate_discard = previous_state.candidate_discard;
K = previous_state.K;

%% Apply KLT on current_image

[current_keypoints,current_candidate_keypoints,discard,candidate_discard] = keypointTracking_Matlab(previous_state.keypoints,...
    previous_state.candidate_keypoints,previous_image,current_image,discard,candidate_discard);


current_state.keypoints = current_keypoints;
current_state.candidate_keypoints = current_candidate_keypoints;
current_state.discard = discard;
current_state.candidate_discard = candidate_discard;

%% Apply P3P + RANSAC on keypoints with an associated landmark
% to test on zero-frame without keypoint tracker working (correctly gives
% identity as pose)

[current_pose,discard] = poseEstimation(current_keypoints, previous_state.landmarks, previous_state.K, discard);

%% Apply linear triangulation on keypoints without associated landmark

[new_keypoints,new_landmarks,updated_candidate_keypoints,updated_candidate_keypoints_1,...
    updated_candidate_pose_1,candidate_discard] = landmarkTriangulation(...
    K,current_pose,previous_state.candidate_pose_1,current_candidate_keypoints,...
    previous_state.candidate_keypoints_1,candidate_discard);

%% Find new features
%  - Perform suppression around existing features?
%  - How about FAST instaed of Harris?
%  - Do this in every iteration or only once in a while?
%  - Note: new_candidate_keypoints = new_candidate_keypoints_1!

[new_candidate_keypoints,new_candidate_keypoints_1,new_candidate_pose_1] = featureExtraction(...
    current_image,current_keypoints,new_keypoints,updated_candidate_keypoints,current_pose,discard,candidate_discard);

%% What is left to do

candidate_discard = candidate_discard + 1; % Penalty for 'old' candidate features

del = discard > discard_max;
candidate_del = candidate_discard > candidate_discard_max;

current_state.landmarks = [previous_state.landmarks(:,~del) new_landmarks];
current_state.keypoints = [current_keypoints(:,~del) new_keypoints];
current_state.discard = [discard(:,~del) zeros(1,size(new_keypoints,2))];
current_state.candidate_keypoints = [updated_candidate_keypoints(:,~candidate_del) new_candidate_keypoints];
current_state.candidate_keypoints_1 = [updated_candidate_keypoints_1(:,~candidate_del) new_candidate_keypoints_1];
current_state.candidate_pose_1 = [updated_candidate_pose_1(:,~candidate_del) new_candidate_pose_1];
current_state.candidate_discard = [candidate_discard(:,~candidate_del) zeros(1,size(new_candidate_keypoints,2))];
current_state.pose = current_pose;
current_state.K = K;

end


