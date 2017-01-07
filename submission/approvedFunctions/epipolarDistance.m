function [ cost ] = epipolarDistance( F, p1, p2 )
%EPIPOLARDISTANCE my version of epipolar line distances
% adapted form reference solutions to work with multiple points

NumPoints = size(p1,2);
homog_points = [p1, p2];
epi_lines = [F.'*p2, F*p1];
denom = epi_lines(1,:).^2 + epi_lines(2,:).^2;
costs =  ((sum(epi_lines.*homog_points,1).^2)./denom );
cost = sqrt(costs(1:NumPoints)+costs(NumPoints+1:end));
end

