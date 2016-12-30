function [idx, C] = kmeans_cluster(X, k)
%KMEANS_CLUSTER kmeans clustering algorithm, using euclidean distance
%   For N p-dimensional data
%   Input:
%       X: Nxp matrix.
%       k: how many clusters is desired.
%   Output:
%       idx: Nx1, each value stands for the cluster label.
%       C: kxp, the ith row of the matrix is the center of the kth cluster.

[idx, C] = kmeans(X, k);

end

