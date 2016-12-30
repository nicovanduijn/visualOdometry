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

%% Setup
%  - Parameters?

discard = previous_state.discard;
candidate_discard = previous_state.candidate_discard;

%% Apply KLT on current_image

[current_keypoints,current_candidate_keypoints,discard,candidate_discard] = keypointTracking(previous_state.keypoints,...
    previous_state.candidate_keypoints,previous_image,current_image,discard,candidate_discard);

%% Apply P3P + RANSAC on keypoints with an associated landmark

[current_pose,discard] = poseEstimation(previous_image,current_image,...
    previous_state.keypoints,current_keypoints,discard);

%% Apply linear triangulation on keypoints without associated landmark

[new_keypoints,new_landmarks,updated_candidate_keypoints,...
    updated_candidate_keypoints_1,updated_candidate_pose_1,candidate_discard] = triangulation(...
    previous_state.K,current_pose,previous_state.candidate_pose_1,current_candidate_keypoints,...
    previous_state.candidate_keypoints_1,candidate_discard);


%% Find new features
%  - Perform suppression around existing features?
%  - How about FAST instaed of Harris?
%  - Do this in every iteration or only once in a while?
%  - Note: new_candidate_keypoints = new_candidate_keypoints_1!

[new_candidate_keypoints,new_candidate_keypoints_1,new_candidate_pose_1] = featureExtraction(...
    current_image,current_keypoints,new_keypoints,updated_candidate_keypoints);

%% What is left to do
%  - TODO: Kick out old/bad candidate_keypoints (using candidate_discard?)

current_state.landmarks = [current_landmarks new_landmarks]; % TODO: Applly discard on current_landmarks
current_state.keypoints = [current_keypoints new_keypoints]; % TODO: Apply discard on current_keypoints
current_state.discard = discard; % TODO Apply discard on itself
current_state.candidate_keypoints = [updated_candidate_keypoints new_candidate_keypoints];
current_state.candidate_keypoints_1 = [updated_candidate_keypoints_1 new_candidate_keypoints_1];
current_state.candidate_pose_1 = [updated_candidate_pose_1 new_candidate_pose_1];
current_state.candidate_discard = candidate_discard;
current_state.K = previous_state.K;

end


