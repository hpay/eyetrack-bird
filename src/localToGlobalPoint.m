function p_out = localToGlobalPoint(Rglobal, pglobal, plocal)
% Convert from local coordinates (e.g. calibrated angle of eye relative to 
% rigid body) to global coordinates, using rigidbody position measured over
% time
% 
% Rglobal is 3x3xnt
% vlocal is 3x1 or 1x3
    
nt = size(Rglobal,3);
p_out = zeros(3,nt);
for ii = 1:3
    Rcurr = squeeze(Rglobal(:,ii,:));
    p_out = p_out + Rcurr*plocal(ii);
end
p_out(:, isnan(Rcurr(1,:))) = NaN;

p_out = p_out + pglobal;

