function shrink(varargin)
% Shrink window by fraction from  default size
% shrink % defualt 50%
% shrink(f) % shink by f
% shrink([h v])

if nargin>0 && all(ishghandle(varargin{1})) && ~(isnumeric(varargin{1})&&numel(varargin{1})<=2)
    ha = varargin{1};
    varargin(1)=[];
else 
    ha = gcf;
end

if isempty(varargin)
    f = [.5 .5];
else 
    f = varargin{1};
end

if length(f)==1
    f = [f f];
end
tempUnits = get(ha, 'Units');
set(ha, 'Units', 'pixels');

a = get(ha,'Position');
set(ha,'Position',[a(1) a(2) a(3)*f(1) a(4)*f(2)])
drawnow;
set(ha, 'Units', tempUnits);
