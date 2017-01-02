clear all;
close all;
clc

kitti_path = 'data/kitti'; % path for kitti dataset
img0 = imread([kitti_path '/00/image_0/' sprintf('%06d.png',0)]);
img1 = imread([kitti_path '/00/image_0/' sprintf('%06d.png',1)]);

% Read a video frame and run the face detector.
% videoFileReader = vision.VideoFileReader('tilted_face.avi');


points = detectHarrisFeatures(img0);
pointTracker = vision.PointTracker;
initialize(pointTracker,points.Location,img0);

oldpoints = points.Location;

[new_points,point_validity] = step(pointTracker,img1);

% Draw the returned bounding box around the detected face.
% videoFrame = insertShape(videoFrame, 'Rectangle', bbox);
figure(1); imshow(img0); title('Detected face');
hold on
for i = 1:length(oldpoints),
    plot(oldpoints(i,1),oldpoints(i,2),'r+')
end


% % Convert the first box into a list of 4 points
% % This is needed to be able to visualize the rotation of the object.
% bboxPoints = bbox2points(bbox(1, :));
% 
% % Detect feature points in the face region.
% points = detectMinEigenFeatures(img0);
% 
% % Display the detected points.
figure(2), imshow(img1), hold on, title('Detected features');
for i = 1:length(new_points),
    plot(new_points(i,1),new_points(i,2),'r+')
end
