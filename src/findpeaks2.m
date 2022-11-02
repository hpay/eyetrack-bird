function [pks, row, col, widths, proms] = findpeaks2(data)

data2 = data';
[pks1, locs1,w1,p1] = findpeaks(data(:)); % peaks along y
[pks2, locs2,w2,p2] = findpeaks(data2(:)); % peaks along x

% A couple of steps are required to make the indices compatible, since the second was calculated on the transposed matrix:
data_size = size(data); % Gets matrix dimensions
[col2, row2] = ind2sub(flip(data_size), locs2); % Converts back to 2D indices
locs2 = sub2ind((data_size), row2, col2); % Swaps rows and columns and translates back to 1D indices

% At this point we only have to check which peak is present in both x and y and convert everything to 2D indices:
[ind, i1, i2] = intersect(locs1, locs2); % Finds common peak position
[row, col] = ind2sub(data_size, ind); % to 2D indices
pks = data(ind);
widths = mean([w1(i1) w2(i2)],2);
proms = mean([p1(i1) p2(i2)],2);

% Sort by prominence
[proms, order] = sort(proms,'descend');
widths = widths(order);
pks = pks(order);
row  = row(order);
col = col(order);

% figure; imagesc(data); hold on
% plot(col, row, '+r')