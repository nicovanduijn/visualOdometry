%% this script shows how to do image classification in Matlab
clear; clc; close all;

%% parsing the dataset folder
caltech_folder = fullfile('dataset', '101_ObjectCategories');
image_sets = [imageSet(fullfile(caltech_folder, 'airplanes')),...
    imageSet(fullfile(caltech_folder, 'ferry')),...
    imageSet(fullfile(caltech_folder, 'laptop')),];
min_num = min([image_sets.Count]);
image_sets = partition(image_sets, min_num, 'randomize');

[training_sets, validation_sets] = partition(image_sets, 0.3, 'randomize');

%% generate visual vocabulary
bag = bagOfFeatures(training_sets);

%% visualize a sample image
img = read(image_sets(randi(3)), randi(min_num));
words_vector = encode(bag, img);
fig1 = figure('Name', 'sample visual words distribution',...
    'NumberTitle', 'off');
subplot(2,1,1);
bar(words_vector);
title('Visual word occurrences')
xlabel('Visual word index')
ylabel('Frequency of occurrence')
subplot(2,1,2);
imshow(img);

%% train a classifier
classifier = trainImageCategoryClassifier(training_sets, bag);

%% evaluate classifier 
disp('evaluate on the training dataset...');
train_conf_matrix = evaluate(classifier, training_sets);  
disp('evaluate on the training dataset...');
validation_conf_matrix = evaluate(classifier, validation_sets);


%% predict the image classification
img = read(image_sets(randi(3)), randi(min_num));
fig2 = figure('Name', 'a random image from the validation sets', ...
    'NumberTitle', 'off');
imshow(img);
[label_idx, score] = predict(classifier, img);
disp(['the image category is ', cell2mat(classifier.Labels(label_idx))]);
disp(['the classification score is ', num2str(score)]);
