function [current_keypoints,current_candidate_keypoints,discard,candidate_discard] = keypointTracking(previous_state_keypoints,previous_state_candidate_keypoints,previous_image,current_image,discard,candidate_discard)
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
r = 20;

for i = 1:length(previous_state_keypoints)
    
    if previous_state_keypoints(1,i)>(r+1) && previous_state_keypoints(1,i)<size(previous_image,1)-(r+1)...
            && previous_state_keypoints(2,i)>(r+1) && previous_state_keypoints(2,i)<size(previous_image,2)-(r+1)
        
         keypoint_frame{i} =     [previous_state_keypoints(1,i)-r,previous_state_keypoints(1,i)+r;...
                                            previous_state_keypoints(2,i)-r,previous_state_keypoints(2,i)+r];
    else
        
        discard(i) = discard(i)+1;
        
    end
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
    
    if ~isempty(keypoint_frame{k})
    
            w = keypoint_frame{k};
            %     center = [w(1,1)+w(1,2)-1 w(2,1)+w(2,2)-1]/2; % center of the template, in image coordinates.
            TemplateData(k).p = [0 0 0 0 previous_state_keypoints(1,k) previous_state_keypoints(2,k)];
            TemplateData(k).image = previous_image(w(1,1):w(1,2), w(2,1):w(2,2)); % pixels of the template
    end
    
%     figure(3)
%     imshow(TemplateData(k).image);
%     a = TemplateData(k).image;
end

%%
% LK Tracking Options (default values for other options)
Options.TranslationIterations = 15;
Options.AffineIterations = 5;

current_image = double(current_image)*(1/255);

for i=1:numel(keypoint_frame),
    if ~isempty(keypoint_frame{i})
        
        [TemplateData(i).p,~,~] = LucasKanadeAffine(current_image,TemplateData(i).p,TemplateData(i).image,Options);
        p = TemplateData(i).p;
        %M = [ 1+p(1) p(3) p(5); p(2) 1+p(4) p(6)];
        current_keypoints(:,i) = [p(6);p(5)];
        
    else
        
        current_keypoints(:,i) =[-1,-1];
    end
end

current_candidate_keypoints = previous_state_candidate_keypoints;
discard = discard;
candidate_discard = candidate_discard;

end


