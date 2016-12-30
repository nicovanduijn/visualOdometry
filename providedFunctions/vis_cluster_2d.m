function fig = vis_cluster_2d(X, idx, C)
%VIS_CLUSTER_2D draws the cluster result, only work for 2d data
%   Input:
%       X: Nx2, original data
%       idx: Nx1, label for each data
%       C: px2, centroids for p clusters
%   Output:
%       fig: the figure handle

N = size(X, 1);
p = size(C, 1);

% check
assert(size(X,2) == 2);
assert(size(idx,1) == N);
assert(size(C,2) == 2);

% colors
rdn_colors = rand(p, 3);

fig = figure('Name', 'cluster visualization', 'NumberTitle', 'off');

% for each cluster, plot the points
for i = 1:p
    label = strcat('Cluster ', int2str(i));
    plot(X(idx==i,1),X(idx==i,2),...
        '.','MarkerSize',12, 'Color', rdn_colors(i, :),...
        'DisplayName', label);
    hold on
end

% for centers of the clusters
plot(C(:,1),C(:,2),...
        'kx', 'MarkerSize', 15, 'LineWidth', 3, ...
        'DisplayName', 'Centroids');
legend('Location','NW')
hold off


end

