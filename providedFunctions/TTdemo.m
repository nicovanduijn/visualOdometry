% Example of Lucas-Kanade template tracker
%
% Guillermo Gallego

% You can run the entire script by pressing the F5 key
% Or, you can evaluate one or more lines by selecting them and pressing the
% F9 key (without needing to copy them to the prompt in the Command Window)

clear all % clear all variables in the workspace
close all % close all figures
clc       % clear the command window

% Load a Traffic Movie, frames in stack with 3th dimension time
%
% (Only differences between frames are stored, for 3x smaller .mat filesize.
% Integrate to get the approximated original movie back)
load('TTdemo_packed_movie');
Vmovie = uint8(cumsum(single(Vmovie),3)+128);

% Get the first movie frame
I = double(Vmovie(:,:,1))*(1/255);

% Show the movie frame
figure, imshow(I,[])
title(['Frame ' num2str(1)]) % Display the frame number on top of the image
set(gcf,'Color','w'); axis on,

% Select the coordinates of several templates
% rect=getrect; TemplateRect{1}=round([rect(2) rect(2)+rect(4);rect(1) rect(1)+rect(3);]);
TemplateRect{1} = [086,111; 100,134];
TemplateRect{2} = [054,091; 224,256];
TemplateRect{3} = [093,103; 285,312];
TemplateRect{4} = [154,171; 250,266];

% Plot templates
color = rand(4,3);
for k=1:numel(TemplateRect),
    v = flipud(TemplateRect{k});
    v = [v(1,1) v(1,1) v(1,2) v(1,2);
         v(2,1) v(2,2) v(2,2) v(2,1)];
    hold on,
    plot_quadrilateral(v, color(k,:));
end

% Save image and to disk
set(gcf,'PaperPositionMode','auto')
print('-dpng','-r0',['picture' num2str(1)])

% Pad templates with extra boundary pixels (not used for the actual template
% tracking, but for smooth image derivatives).
b = 5; % pixels
padding = [-b,b;-b,b];
for k=1:numel(TemplateRect),
    TemplateRect{k} = TemplateRect{k} + padding;
end


% -Set initial parameters of the templates
% -Set padded template image.
% 
% Affine Transformation Matrix is used in Lucas Kanade Tracking
% with 6 parameters
% M    = [ 1+p(1) p(3)   p(5); 
%          p(2)   1+p(4) p(6); 
%          0      0      1];
%

% Make a struct to store templates
TemplateData = struct;
for k=1:numel(TemplateRect)
    w = TemplateRect{k};
    center = [w(1,1)+w(1,2)-1 w(2,1)+w(2,2)-1]/2; % center of the template, in image coordinates.
    TemplateData(k).p = [0 0 0 0 center(1) center(2)]; % parameters for the affine warp of the template
    TemplateData(k).image = I(w(1,1):w(1,2), w(2,1):w(2,2)); % pixels of the template
end

%%
% LK Tracking Options (default values for other options)
Options.TranslationIterations = 15;
Options.AffineIterations = 5;

% Make a colormap
cmap = hot(256);

% Matrix to store squared pixel error between template and ROI in
% movieframe after template tracking.
T_error = zeros(size(Vmovie,3), length(TemplateData));


% Loop through the movie frames
% L = size(Vmovie,3);
L = 50;
for i=1:L
    % Get the a movie frame
    I = double(Vmovie(:,:,i))*(1/255);
    
    % Do the tracking for all templates, using the Lucas-Kanade method
    for k=1:length(TemplateData)
        [TemplateData(k).p,ROIimage,T_error(i,k)] = ...
            LucasKanadeAffine(I,TemplateData(k).p,TemplateData(k).image,Options);
    end
    
    
    % Show the location of the templates in the movie frame
    figure(1), imshow(I); % Show the movie frame
    title(['Frame ' num2str(i)]) % Display the frame number on top of the image
    set(gcf,'Color','w'); axis on,
    hold on
    
    % Display the tracked templates
    for k=1:length(TemplateData)
        plot(TemplateData(k).p(6),TemplateData(k).p(5),'go',...
            'MarkerFaceColor',cmap(round(255 * k/length(TemplateData))+1,:));

        % Get vertices of the template
        [template_height, template_width] = size(TemplateData(k).image);
        halfw = template_width/2 - b;
        halfh = template_height/2 - b;
        % Compute the position of the vertices of the (warped) template...
        p = TemplateData(k).p;

        % Choose one of the two next lines:
        % If we want to plot the translation part only, uncomment:
        %M = [ 1 0 p(5); 0 1 p(6); 0 0 1];
        % If we want to plot the full affine transformation, uncomment:
        M = [ 1+p(1) p(3) p(5); p(2) 1+p(4) p(6); 0 0 1];
        
        v = [halfh, -halfh, -halfh,  halfh; % x-axis is vertical, downward
             halfw,  halfw, -halfw, -halfw; % y-axis is horizontal, to the right
             ones(1,4)];
         
        v = M*v; % Apply transformation
        
        % Plot vertices of (warped) template
        plot_quadrilateral(v([2,1],:), color(k,:));
    end
    drawnow
    
    if(mod(i,10)==0)
	% Save image and to disk
        set(gcf,'PaperPositionMode','auto')
        print('-dpng','-r0',['picture' num2str(i)])
    end
end
