function p_W_estimate_aligned = alignEstimateToGroundTruth(...
    p_W_GT, p_W_estimate)

twist_guess = HomogMatrix2twist(eye(4));
scale_guess = 1;

x = [twist_guess; scale_guess];
error_terms = @(x) alignError(x, p_W_GT, p_W_estimate);
options = optimoptions(@lsqnonlin, 'Display', 'iter');
x_optim = lsqnonlin(error_terms, x, [], [], options);

T_old_new = twist2HomogMatrix(x_optim(1:6));
scale_old_new = x_optim(7);

num_frames = size(p_W_estimate, 2);
p_W_estimate_aligned = scale_old_new * T_old_new(1:3, 1:3) * p_W_estimate ...
    + repmat(T_old_new(1:3, 4), [1 num_frames]);

end

