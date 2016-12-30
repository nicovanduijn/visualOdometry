%% this script shows how to do image retieval in Matlab
clear; clc; close all;

%% parsing the dataset folder
bookcover_folder = fullfile('dataset', 'bookCovers');
image_set = imageSet(bookcover_folder);

%% generate visual vocabulary and index images
image_index = indexImages(image_set);

%% retrieval images
query_folder = fullfile(bookcover_folder, 'queries');
query_imgs = cell(3,1);
result_imgs = cell(3,1);

for i = 1:3
    query_img = imread(fullfile(query_folder, ['query', int2str(i), '.jpg']));
    disp(['Query image: ', ['query', int2str(i), '.jpg']]);
    img_ids = retrieveImages(query_img, image_index);
    result_img = imread(cell2mat(image_set.ImageLocation(img_ids(1))));
    disp(['Result image:', cell2mat(image_set.ImageLocation(img_ids(1)))]);
    query_imgs{i} = query_img;
    result_imgs{i} = result_img;
end

fig = figure('Name', 'Retrieval Result', 'NumberTitle', 'off');
subplot(321); imshow(query_imgs{1});  title('Query Images');
subplot(322); imshow(result_imgs{1}); title('Query Results');
subplot(323); imshow(query_imgs{2}); subplot(324); imshow(result_imgs{2});
subplot(325); imshow(query_imgs{3}); subplot(326); imshow(result_imgs{3});

