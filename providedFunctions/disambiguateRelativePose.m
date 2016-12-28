% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C2_W = T_C2_C1 is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 1)
%   to camera 2.
%

function [R,T] = disambiguateRelativePose(Rots,u3,p1,p2,K1,K2)

M1 = K1 * eye(3,4); % Projection matrix of camera 1

num_positive_depths_best = 0;
for iRot = 1:2
    Rot_test = Rots(:,:,iRot);
    
    for iSignT = 1:2
        T_test = u3 * (-1)^iSignT;
        
        M2 = K2 * [Rot_test, T_test];
        P_test = linearTriangulation(p1,p2,M1,M2);
        
        num_positive_depths = sum(P_test(3,:) > 0);
              
        if (num_positive_depths > num_positive_depths_best)
            % Keep the rotation that gives the highest number of points
            % in front of both cameras
            R = Rot_test;
            T = T_test;
            num_positive_depths_best = num_positive_depths;
        end
    end
end

end

