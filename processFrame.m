function [current_state] = processFrame(previous_state, previous_image, current_image)
%PROCESSFRAME Processes a frame
%   Inputs:
%   - previous_state: Struct with fields
%       * landmarks: 4xN (homogeneous coordinates)
%       * keypoints: 2xN
%       * candidate_keypoints: 2xM
%       * candidate_keypoints_1: 2xM
%       * candidate_pose_1: 12xM
%       * pose: 12x1
%   - previous_image: XxY
%   - current_image: XxY
%
%   Outputs:
%   - current_state: Struct
%   - current_pose: 4x3 non-homogenous matrix representing pose [R|T]
%   
%   Internal variables:
%   - discard: 1xN (voting array)

%% Setup

% N = ?;
discard = zeros(1,N);

%% Apply KLT on current_image

[current_keypoints,current_candidate_keypoints,discard] = keypointTracking(previous_state.keypoints,...
    previous_state.candidate_keypoints,previous_image,current_image,discard);

%% Apply P3P + RANSAC on keypoints with an associated landmark

[current_pose,discard] = poseEstimation(previous_image,current_image,...
    previous_state.keypoints,current_keypoints,discard);

%% Apply linear triangulation on keypoints without associated landmark

[new_keypoints,new_landmarks,updated_candidate_keypoints,...
    updated_candidate_keypoints_1,updated_candidate_pose_1] = triangulation(...
    current_pose,previous_state.candidate_pose_1,current_keypoints,...
    current_candidate_keypoints,previous_state.candidate_keypoints_1);


%% Find new features
%  - Perform suppression around existing features?
%  - How about FAST instaed of Harris?
%  - Do this in every iteration or only once in a while?
%  - Variable "discard" really needed?
%  - Note: new_candidate_keypoints = new_candidate_keypoints_1!

[new_candidate_keypoints,new_candidate_keypoints_1,new_candidate_pose_1] = featureExtraction(...
    current_image,current_keypoints,new_keypoints,updated_candidate_keypoints,discard);

%% What is left to do
%  - Kick out old/bad candidate_keypoints

current_state.landmarks = [current_landmarks new_landmarks]; % TODO: Applly discard on current_landmarks
current_state.keypoints = [current_keypoints new_keypoints]; % TODO: Apply discard on current_keypoints
current_state.candidate_keypoints = [updated_candidate_keypoints new_candidate_keypoints];
current_state.candidate_keypoints_1 = [updated_candidate_keypoints_1 new_candidate_keypoints_1];
current_state.candidate_pose_1 = [updated_candidate_pose_1 new_candidate_pose_1];

end


