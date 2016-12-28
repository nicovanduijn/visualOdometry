function [current_state, current_pose] = processFrame(previous_state, previous_image, current_image )
%PROCESSFRAME Processes a frame
%   takes in the following inputs:
% previous_state: XxY
% previous_image: XxY
% current_image: XxY
%
% and returns:
% current_state: XxY
% current_pose: 4x3 non-homogenous matrix representing pose [R|T]

harris_scores = harris(current_image); %compute harris scores to find corners


end


