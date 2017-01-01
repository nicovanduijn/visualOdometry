function [current_state,M,a] = processFrame_Marc(previous_state, previous_image, current_image)
%PROCESSFRAME Processes a frame
%   Inputs:
%   - previous_state: Struct with fields
%       * landmarks: 4xN (homogeneous coordinates)
%       * keypoints: 2xN
%       * candidate_keypoints: 2xM
%       * candidate_keypoints_1: 2xM
%       * candidate_pose_1: 12xM
%       * pose: 12x1
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


current_state.keypoints = current_keypoints;
current_state.candidate_keypoints = current_candidate_keypoints;
current_state.discard = discard;
current_state.candidate_discard = candidate_discard;

end

