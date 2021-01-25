function w = Skew(x)
% S Generate the skew-matrix
%   This function will generate the 3x3 skew-matrix from a 3x1 input vector

w = [0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end

