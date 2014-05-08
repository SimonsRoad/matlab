function [ pyr ] = get_pyramid(img, method, n)
% compute n levels of pyramid 

% transform to b&w if color
if size(img,3) > 1
    img = rgb2gray(img);
end

pyr = cell(n,1);
pyr{1} = img;

for i = 2:n
    if strcmp(method, 'mean')
        pyr{i} = pyrDownMean(pyr{i-1});
    end
    
    if strcmp(method, 'reduce')
        pyr{i} = impyramid(pyr{i-1}, 'reduce');
    end
    
    if strcmp(method, 'median')
        filtered = medfilt2(pyr{i-1});
        pyr{i} = impyramid(filtered, 'reduce');
    end
    
    if strcmp(method, 'median2') 
        pyr{i} = pyrDownMedian(pyr{i-1});
    end
end

end

function [img_out] = pyrDownMean(img_in) 
    height = floor(size(img_in,1)/2);
    width = floor(size(img_in, 2)/2);
    img_out = NaN(height, width);
    for i = 1:height
        for j = 1:width
            u1 = 2 * i;
            u0 = u1 - 1;
            v1 = 2 * j;
            v0 = v1 - 1;
            img_out(i, j) = double(img_in(u0,v0) + img_in(u0,v1) + img_in(u1, v0) + img_in(u1, v1)) / 4;
        end
    end
end

function [img_out] = pyrDownMedian(img_in) 
    height = floor(size(img_in,1)/2);
    width = floor(size(img_in, 2)/2);
    img_out = NaN(height, width);
    for i = 1:height
        for j = 1:width
            u1 = 2 * i;
            u0 = u1 - 1;
            v1 = 2 * j;
            v0 = v1 - 1;
            v = [img_in(u0,v0), img_in(u0,v1), img_in(u1, v0), img_in(u1, v1)];            
            img_out(i, j) = median(v);
        end
    end
end
