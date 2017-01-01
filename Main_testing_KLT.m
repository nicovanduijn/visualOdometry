%% Setup
clear all % clean up
clc
close all
  %rng(1);

% set the dataset to use
ds = 0; % 0: KITTI, 1: Malaga, 2: parking 
parking_path = 'data/parking'; % path for parking dataset
kitti_path = 'data/kitti'; % path for kitti dataset
malaga_path = 'data/malaga';
addpath('providedFunctions'); % add provided functions from exercise sessions
use_init = false;

%% Nico-only section
% use_init = true;
% addpath('nicosFunctions');


if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
if ds == 0
    bootstrap_frames = [0,2];
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    bootstrap_frames = [1;3]; % for now, just use first and third frame
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    bootstrap_frames = [1;3]; % for now, just use first and third frame
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

%% initialize with bootstrap
if(use_init)
    [state] = initializePose(img0, img1, K);
    prev_img = img1;
else
    % hard-coded initialization with ground truth
    bootstrap_frames = [0, 0]; % so we start VO with frame 1
    prev_img = img0;
     state = struct;
     state.landmarks = load('p_W_landmarks.txt')';
     state.keypoints = load('keypoints.txt')';
     temp = load([kitti_path '/poses/00.txt']);
     temp = poseVectorToTransformationMatrix(temp(2,:));
     state.pose = reshape(temp(1:3,1:4),3,4);
     state.candidate_keypoints = [];
     state.candidate_keypoints_1=[];
     state.candidate_pose_1=[];
     state.discard =zeros(1,length(state.keypoints));
     state.candidate_discard = [];
end



%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    figure(1), imshow(prev_img)
    title(['Frame ' num2str(i-1)])
    set(gcf,'Color','w'); axis on,
    hold on
    
    for i=1:length(state.keypoints)
       
        plot(state.keypoints(2,i),state.keypoints(1,i),'r*')
    end
       
    
    % do all the fancy stuff
    [current_state] = processFrame_Marc(state, prev_img, image);
  
   
    
    figure(2)
    image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',1)]);
    imshow(image)
    title(['Frame ' num2str(1)])
    set(gcf,'Color','w'); axis on,
    hold on
    for i=1:length(state.keypoints)
        plot(current_state.keypoints(2,i),current_state.keypoints(1,i),'r*')
    end
    
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5 
        
    % Makes sure that plots refresh.    
    pause(0.01);
    
    prev_img = image;
end