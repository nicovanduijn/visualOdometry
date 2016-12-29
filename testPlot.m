   

kitti_path = 'data/kitti';
ground_truth = load([kitti_path '/poses/00.txt']);
figure
hold on
for i=1:150
pose=poseVectorToTransformationMatrix(ground_truth(i,:));
plot(pose(1,4),pose(2,4),'rx');
end
axis equal