function [current_state, current_pose] = processFrame(previous_state, previous_image, current_image )
%PROCESSFRAME Processes a frame
%   Inputs:
%   - previous_state: Struct with fields
%       * landmarks: 4xN (homogeneous coordinates)
%       * keypoints: 2xN
%       * keypoints_nl: 2xM
%       * keypoints_nl_1: 2xM
%       * pose_nl_1: 12xM
%   - previous_image: XxY
%   - current_image: XxY
%
%   Outputs:
%   - current_state: Struct
%   - current_pose: 4x3 non-homogenous matrix representing pose [R|T]
%   
%   Internal variables:
%   - discard: 1xN (array of logicals, where true denotes "discard")

%% Setup

% N = ?;
discard = zeros(1,N);

%% Apply KLT on current_image

[current_keypoints,discard] = keypointTracking(previous_state.keypoints,...
    previous_image,current_image,discard);

%% Apply P3P + RANSAC on keypoints with an associated landmark

[current_pose,discard] = poseEstimation(previous_image,current_image,...
    previous_state.keypoints,current_keypoints,discard);

%% Apply linear triangulation on keypoints without associated landmark

[new_keypoints,new_landmarks,current_keypoints_nl] = triangulation(...
    current_pose,previous_state.pose_nl_1,current_keypoints,...
    previous_state.keypoints_nl,previous_state.keypoints_nl_1,discard);

%% Find new features
%  - Perform suppression around existing features?
%  - How about FAST instaed of Harris?
%  - Do this in every iteration or only once in a while?

[new_keypoints_nl_1] = featureExtraction(current_image,current_keypoints,current_keypoints_nl,discard);

%% What is left to do
%  - Stitch together current_state.



end


