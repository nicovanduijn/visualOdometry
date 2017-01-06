function [error] = reprojectionError(p,P,M)
% Inputs:
%   - p: 2D points 2xn
%   - P: homogeneous 3D points 4xn
%   - M: K*[R_CW, T_CW] 3x4

p_homo = M*P;
p_reprojected = p_homo(1:2,:) ./ p_homo(3,:);

error = norm(p - p_reprojected);

end

