function [current_keypoints,current_candidate_keypoints,discard,candidate_discard] = keypointTracking_Matlab(previous_state_keypoints,previous_state_candidate_keypoints,previous_image,current_image,discard,candidate_discard)
%Track the keypoints from the previous frame to the new frame
%   Inputs:
%   - previous_state_keypoints:2xN
%   - previous_state_candidate_keypoints: 2xM
%   - previous_image: XxY
%   - current_image: XxY
%   - discard: 1xN
%   - candidate_discard: 1xM
%
%   Outputs:
%   - current_keypoints: 2xN
%   - current_candidate_keypoints: 2xM
%   - discard: 1xN
%   - candidate_discard: 1xM


%% Keypoints tracking
points = (previous_state_keypoints)';

% width = size(previous_image,2);
% height = size(previous_image,1);

pointTracker = vision.PointTracker;
initialize(pointTracker,points,previous_image);

[new_points,point_validity] = step(pointTracker,current_image);

% new_points = new_points.*point_validity;
% discard = discard.*(point_validity)';

% Delete points which are not in the image anymore
% new_points(new_points<0) = 0;
% new_points(new_points(:,1)>width,:) = 0;
% new_points(new_points(:,2)>height,:) = 0;

% new_points(sum((new_points==0),2)>0,:) = [];
discard(~point_validity) = inf;

%% Candidate keypoints tracking
if ~isempty(previous_state_candidate_keypoints),
    
    candidate_points = (previous_state_candidate_keypoints)';
    pointTracker = vision.PointTracker;
    initialize(pointTracker,candidate_points,previous_image);

    [new_candidate_points,candidate_point_validity] = step(pointTracker,current_image);
    candidate_discard(~point_validity) = inf;
%      current_candidate_keypoints = new_candidate_points.*candidate_point_validity;
    current_candidate_keypoints = (new_candidate_points)';
else
    current_candidate_keypoints = previous_state_candidate_keypoints;
    
end

current_keypoints = (new_points)';


discard = discard;
candidate_discard = candidate_discard;

end