function [current_keypoints,current_candidate_keypoints,discard,candidate_discard,M,a] = keypointTracking(previous_state_keypoints,previous_state_candidate_keypoints,previous_image,current_image,discard,candidate_discard)
%Track the keypoints from the previous frame to the new frame
%   Inputs:
%   - previous_state.keypoints:2xN
%   - previous_state.candidate_keypoints: 2xM
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


%% Set templates around keypoints and pad templates with extra boundary pixels

previous_image = double(previous_image)*(1/255);
r = 10;

for i = 1%:length(previous_state_keypoints)
    keypoint_frame{i} = [previous_state_keypoints(1,i+312)-r,previous_state_keypoints(1,i+312)+r;...
                         previous_state_keypoints(2,i+312)-r,previous_state_keypoints(2,i+312)+r];
end

% b = 5;
% padding = [-b,b;-b,b];
% for k = 1:numel(keypoint_frame),
%     keypoint_frame{k} = keypoint_frame{k} + padding;
%     
% end
    
%% Struct to store templates

TemplateData = struct;
for k = 1:numel(keypoint_frame),
    w = keypoint_frame{k};
%     center = [w(1,1)+w(1,2)-1 w(2,1)+w(2,2)-1]/2; % center of the template, in image coordinates.
    TemplateData(k).p = [0 0 0 0 previous_state_keypoints(1,k+312) previous_state_keypoints(2,k+312)];
    TemplateData(k).image = previous_image(w(1,1):w(1,2), w(2,1):w(2,2)); % pixels of the template
    
    figure(3)
    imshow(TemplateData(k).image);
    a = TemplateData(k).image;
end

%%
% LK Tracking Options (default values for other options)
Options.TranslationIterations = 15;
Options.AffineIterations = 5;

current_image = double(current_image)*(1/255);
[TemplateData(1).p,~,~] = LucasKanadeAffine(current_image,TemplateData(1).p,TemplateData(1).image,Options);

p = TemplateData(1).p;

M = [ 1+p(1) p(3) p(5); p(2) 1+p(4) p(6)];

         
% current_keypoints = M*[previous_state_keypoints(:,313);1];
current_keypoints = [p(5);p(6)]
current_candidate_keypoints = previous_state_candidate_keypoints;
discard = discard;
candidate_discard = candidate_discard;

end


