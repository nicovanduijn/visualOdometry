%% Setup
clear all % clean up
clc
close all
rng(1);

% set the dataset to use
ds = 2; % 0: KITTI, 1: Malaga, 2: parking, 3: ETH
parking_path = 'data/parking'; % path for parking dataset
kitti_path = 'data/kitti'; % path for kitti dataset
malaga_path = 'data/malaga';
eth_path = 'data/eth';
addpath('approvedFunctions');
use_init = true;
global params;

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
    % Path containing images, depths and all...
    assert(exist('eth_path', 'var') ~= 0);
    K = (csvread([eth_path '/K.txt']))';
    params =ethParams();
    last_frame = 433; % 866

else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
if ds == 0
    bootstrap_frames = [1,3];
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
elseif ds == 3
    bootstrap_frames = [1;5]; % for now, just use first and third frame
    img0 = imread([eth_path ...
        sprintf('/Path/Path_ETH_%04d.png',bootstrap_frames(1))]);
    img1 = imread([eth_path ...
        sprintf('/Path/Path_ETH_%04d.png',bootstrap_frames(2))]);

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
    state.landmarks = load('/data/kitti/p_W_landmarks.txt');
    state.landmarks = homogenize(state.landmarks');
    state.keypoints = load('/data/kitti/keypoints.txt');
    state.keypoints = [state.keypoints(:,2)';state.keypoints(:,1)'];
    temp = load([kitti_path '/poses/00.txt']);
    temp = poseVectorToTransformationMatrix(temp(2,:));
    state.pose = reshape(temp(1:3,1:4),3,4);
    state.candidate_keypoints = zeros(2,0)';
    state.candidate_keypoints_1= zeros(2,0)';
    state.candidate_pose_1 = zeros(12,0)';
    state.K = K;
    state.discard = zeros(1,size(state.landmarks,2));
    state.candidate_discard = zeros(1,size(state.candidate_keypoints,2));
end

state.landmarkBundleAdjustment_struct = struct();


%% Continuous operation

number_of_frames = last_frame; % last_frame

range = (bootstrap_frames(2)+1):number_of_frames;
init_counter = 0;
path = zeros(3,range(end)-range(1));
reinit_mask = false(1,range(end)-range(1));
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
    
    path(:,i-range(1)+1) = state.pose(:,4);
    reinit_mask(:,i-range(1)+1) = state.init_counter <= init_counter;
    
    prev_img = image;
    
    init_counter = state.init_counter;
end

%% Plots
figure(1)
plot(path(1,~reinit_mask),path(3,~reinit_mask),'r','DisplayName','estimated path')
hold on
% plot(path(1,reinit_mask),path(3,reinit_mask),'mo','MarkerSize',5,'DisplayName','reinitialization')
% hold on
axis equal
if(ds ~= 1 && ds ~=3) %no ground truth for malaga and ETH dataset
    plot(ground_truth(range,1),ground_truth(range,2),'b','DisplayName','ground truth')
end
legend('show')
hold off