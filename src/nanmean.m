function y = nanmean(varargin)
%NANMEAN   Average or mean value.
%   S = NANMEAN(X) is the mean value of the elements in X if X is a vector. 
%   For matrices, S is a row vector containing the mean value of each 
%   column. 
%   For N-D arrays, S is the mean value of the elements along the first 
%   array dimension whose size does not equal 1.
%
%   NANMEAN(X,DIM) takes the mean along the dimension DIM of X.
%
%   S = NANMEAN(...,TYPE) specifies the type in which the mean is performed, 
%   and the type of S. Available options are:
%
%   'double'    -  S has class double for any input X
%   'native'    -  S has the same class as X
%   'default'   -  If X is floating point, that is double or single,
%                  S has the same class as X. If X is not floating point, 
%                  S has class double.
%   Example:
%       X = [1 2 3; 3 3 6; 4 6 8; 4 7 7]
%       nanmean(X,1)
%       nanmean(X,2)
%
%   Class support for input X:
%      float: double, single
%      integer: uint8, int8, uint16, int16, uint32,
%               int32, uint64, int64
%
%   Copyright 1984-2017 The MathWorks, Inc. 
%   EDITED HP FOM MEAN TO NANMEAN 2019


y = mean(varargin{:}, 'omitnan');