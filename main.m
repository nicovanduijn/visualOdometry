%% Setup
clear all
close all
clc
rng(1);

%--------------------------------------------------------------------------
% Choose here which dataset to use
ds = 0; % 0: KITTI, 1: Malaga, 2: parking, 3: Own(ETH)

% No need to change anything below here (except if a different path should be set)
%--------------------------------------------------------------------------

parking_path = 'data/parking'; % path for parking dataset
kitti_path = 'data/kitti'; % path for kitti dataset
malaga_path = 'data/malaga'; % path for malaga
eth_path = 'data/eth'; %path for our own data set
addpath('approvedFunctions'); % functions we need
global params; % make parameters globally available

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    params = kittiParams();
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    params = malagaParams();
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    params =parkingParams();
elseif ds == 3
    % Path containing images
    assert(exist('eth_path', 'var') ~= 0);
    K = (csvread([eth_path '/K.txt']))';
    params =ethParams();
    last_frame = 433; % Actually 866, but we only use every other image
else
    assert(false);
end

%% Bootstrap
if ds == 0
    bootstrap_frames = [1,3];
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    bootstrap_frames = [1;3]; %use first and third frame
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    bootstrap_frames = [1;3]; %use first and third frame
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
elseif ds == 3
    bootstrap_frames = [1;5]; % for now, just use first and fifth frame
    img0 = imread([eth_path ...
        sprintf('/Path/Path_ETH_%04d.png',bootstrap_frames(1))]);
    img1 = imread([eth_path ...
        sprintf('/Path/Path_ETH_%04d.png',bootstrap_frames(2))]);
else
    assert(false);
end

%% initialize with bootstrap
[state] = initializePose(img0, img1, K);
prev_img = img1;
state.landmarkBundleAdjustment_struct = struct();

%% Continuous operation
init_counter = 0;
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
    elseif ds == 3
        image = imread([eth_path ...
            sprintf('/Path/Path_ETH_%04d.png',2*i)]);
    else
        assert(false);
    end
    
    % do all the fancy stuff
    [state] = processFrame(state, prev_img, image);
    
    % plot everything
    subplot(2,2,1:2);
    imshow(image);
    hold on;
    plot(state.keypoints(1,:), state.keypoints(2,:), 'gx', 'DisplayName', 'keypoints');
    hold on;
    num_old_keypoints = size(state.previous_keypoints,2);
    plot(state.keypoints(1,num_old_keypoints+1:end), state.keypoints(2,num_old_keypoints+1:end), 'bx', 'DisplayName', 'new keypoints');
    hold on;
    plot(state.new_candidate_keypoints(1,:), state.new_candidate_keypoints(2,:), 'rx', 'DisplayName', 'new candidate keypoints');
    hold on;
    num_old_candidate_keypoints = size(state.previous_candidate_keypoints,2);
    plot(state.candidate_keypoints(1,1:num_old_candidate_keypoints), state.candidate_keypoints(2,1:num_old_candidate_keypoints), 'yx', 'DisplayName', 'candidate keypoints');
    hold on;
    x_from = state.previous_keypoints(1,:);
    x_to = state.keypoints(1,1:num_old_keypoints);
    y_from = state.previous_keypoints(2,:);
    y_to = state.keypoints(2,1:num_old_keypoints);
    plot([x_from; x_to], [y_from; y_to], 'g-', 'Linewidth', 3, 'HandleVisibility', 'off');
    hold on;
    x_from = state.previous_candidate_keypoints(1,:);
    x_to = state.candidate_keypoints(1,1:num_old_candidate_keypoints);
    y_from = state.previous_candidate_keypoints(2,:);
    y_to = state.candidate_keypoints(2,1:num_old_candidate_keypoints);
    plot([x_from; x_to], [y_from; y_to], 'y-', 'Linewidth', 3, 'HandleVisibility', 'off');
    hold off;
    legend('show')
    subplot(2,2,3); % birds-eye view of our path
    if state.init_counter <= init_counter
        plot(state.pose(1,4),state.pose(3,4),'mo', 'MarkerSize', 10, 'DisplayName', 're-initialization');
    else
        plot(state.pose(1,4),state.pose(3,4),'rx', 'DisplayName', 'estimated path');
    end
    hold on
    if(ds ~= 1 && ds ~= 3) %no ground truth for malaga and ETH dataset
        plot(ground_truth(i,1),ground_truth(i,2),'bx', 'DisplayName', 'ground truth');
    end
    legend('show');
    axis equal
    
    pause(0.01); % makes sure that plots refresh.
    
    prev_img = image;
    init_counter = state.init_counter;
end