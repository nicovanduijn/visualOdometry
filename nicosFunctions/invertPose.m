function [ inverted_pose ] = invertPose( pose )
%INVERTPOSE inverts a pose
%   Detailed explanation goes here

inverted_pose =pose;
inverted_pose(1:3, 1:4) = [pose(1:3,1:3)' -pose(1:3,1:3)'*pose(1:3,4)];

end
