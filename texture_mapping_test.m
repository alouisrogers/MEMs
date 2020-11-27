imgRGB = imread('peppers.png');
[imgInd,map] = rgb2ind(imgRGB,256);
[imgIndRows,imgIndCols] = size(imgInd);
[X,Y,Z] = cylinder(imgIndRows,imgIndCols);
surface(X,Y,Z,flipud(imgInd),...
    'FaceColor','texturemap',...
    'EdgeColor','none',...
    'CDataMapping','direct')
colormap(map)
view(-35,45)