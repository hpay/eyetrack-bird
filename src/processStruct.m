function E = processStruct(E, fcn)
% assums all fields have the same number of rows
% Do the same thing (fcn) to each column + 3rd dimension

fields = fieldnames(E);

for ii = 1:length(fields)
    currdata = E.(fields{ii});
    
    d = size(currdata);
    if length(d) < 3; d = [d 1]; end
    for jj = 1:d(2)
        for kk = 1:d(3)
            if any(~isnan(currdata(:,jj,kk)))
            currdata(:, jj,kk) = fcn(currdata(:,jj,kk));
            end
        end
    end
    E.(fields{ii}) = currdata;
    
end

