function [current_keypoints,current_candidate_keypoints,discard,candidate_discard] = keypointTracking(previous_state_keypoints,previous_state_candidate_keypoints,previous_image,current_image,discard,candidate_discard)
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

%% Parameters
global params;
min_dist_edge_of_the_screen = params.track_min_kp_dist_to_edge;

%% Keypoints tracking
points = (previous_state_keypoints)';

%Set width and height of the frame
width = size(previous_image,2);
height = size(previous_image,1);

%Configure the tracker object properties
pointTracker = vision.PointTracker;

%Initialize Tracking Process
initialize(pointTracker,points,previous_image);

%Track keypoints in new frame
[new_points,point_validity] = step(pointTracker,current_image);

% Set discard of points which are not in the image anymore to infinity
discard(new_points(:,1)<min_dist_edge_of_the_screen | new_points(:,2)<min_dist_edge_of_the_screen)  = inf;
discard(new_points(:,2)>(height-min_dist_edge_of_the_screen) | new_points(:,1)>(width-min_dist_edge_of_the_screen)) = inf;
discard(~point_validity) = inf;

%% Candidate keypoints tracking
if ~isempty(previous_state_candidate_keypoints)
    
    candidate_points = (previous_state_candidate_keypoints)';
    
    %Configure the tracker object properties
    pointTracker = vision.PointTracker;
    
    %Initialize Tracking Process
    initialize(pointTracker,candidate_points,previous_image);
    
    %Track keypoints in new frame
    [new_candidate_points,candidate_point_validity] = step(pointTracker,current_image);
    candidate_discard(new_candidate_points(:,1)<0 | new_candidate_points(:,2)<0)  = inf;
    candidate_discard(new_candidate_points(:,2)>height | new_candidate_points(:,1)>width)  = inf;
    
    % Set discard of points which are not in the image anymore to infinity
    candidate_discard(~candidate_point_validity) = inf;
    current_candidate_keypoints = (new_candidate_points)';
else
    current_candidate_keypoints = previous_state_candidate_keypoints;
    
end


current_keypoints = (new_points)';

end