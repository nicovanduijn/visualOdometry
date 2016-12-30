function p_handle = plot_quadrilateral(vertices, color)

if (nargin<2)
    color = 'm';
end

assert(size(vertices,2)==4);  % Check that there are four columns = points
assert(size(vertices,1)==2);  % Check that the points have two coordinates

% warning: Assume hold on is active
p_handle = plot(vertices(1,[1:4,1]), vertices(2,[1:4,1]), 'Color',color, 'LineWidth',2);
