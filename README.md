# visualOdometry
Visual Odometry Pipeline implemented in MATLAB by Yvain de Viragh, Marc Ochsner and Nico van Duijn

An overview of the performance of this pipeline can be found at
https://www.youtube.com/watch?v=7DpAWUQQgVU

To run, make sure all files are in the same folder as a "data" folder which contains the datasets to be run.
Subfolders in '/data' are as follows:
-kitti (containing '/00', '/poses', 'keypoints.txt')
-malaga (containing '/Images', '/malaga-urban-...')
-parking (containing '/images', 'K.txt'...)
-eth (containing '/Path', 'cameraParams.mat', 'K.txt')

Pipeline parameters are defined in 
- ethParams.m 
- kittiParams.m 
- malagaParams.m
- parkingParams.m. 

in main, select the dataset to be run by setting ds=..
0 for kitti
1 for malaga
2 for parking
3 for ETH

ETH Dataset
This dataset can be found here: https://www.dropbox.com/sh/mhrpmllsdomer5p/AABb7Gz0JGaK-FXOYEMNzeGfa?dl=0
Just copy the folder eth in your data folder (data\eth) and run the main file with ds = 3.

state is defined as struct containing:
%       * landmarks: 4xN (homogeneous coordinates)
%       * keypoints: 2xN
%       * candidate_keypoints: 2xM
%       * candidate_keypoints_1: 2xM
%       * candidate_pose_1: 12xM
%       * pose: 3x4
%       * K: 3x3
%       * discard 1xN
%       * candidate_discard 1xM
%       * init_counter 1x1
%       * landmarkBundleAdjustment_struct: struct
%       * previous_keypoints (for plotting only)
%       * previous_candidate_keypoints (for plotting only)
%       * new_candidate_keypoints (for plotting only)




