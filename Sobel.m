% Documentation from https://www.mathworks.com/help/images/detecting-a-cell-using-image-segmentation.html

clc;    
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
clear; 

%% Read Image
I = imread('gum.jpg');
figure(1);
imshow(I)
title('Original Image');
% text(size(I,2),size(I,1)+15, ...
%     'Image courtesy of Alan Partin', ...
%     'FontSize',7,'HorizontalAlignment','right');
% text(size(I,2),size(I,1)+25, ....
%     'Johns Hopkins University', ...
%     'FontSize',7,'HorizontalAlignment','right');

%% Detect entire picture outline
I=rgb2gray(I); %Convert to gray scale 
[~,threshold] = edge(I,'sobel');
fudgeFactor = 1;
BWs = edge(I,'sobel',threshold * fudgeFactor);
figure(2)
imshow(BWs)
title('Binary Gradient Mask')

%% Dilate the image 
se90 = strel('line',3,90);
se0 = strel('line',3,0);
BWsdil = imdilate(BWs,[se90 se0]);
figure(3)
imshow(BWsdil)
title('Dilated Gradient Mask')

%% Fill in interior gaps
BWdfill = imfill(BWsdil,'holes');
figure(4)
imshow(BWdfill)
title('Binary Image with Filled Holes')

%% Smooth the object
seD = strel('line',2,30);
BWfinal = imerode(BWdfill,seD);
BWfinal = imerode(BWfinal,seD);
figure(5)
imshow(BWfinal)
title('Segmented Image');

%% Visualize the segmentation
figure(6)
imshow(labeloverlay(I,BWfinal))
title('Mask Over Original Image')

% Alternative 
BWoutline = bwperim(BWfinal);
SegoutR = I;
SegoutG = I;
SegoutB = I;
%now set yellow, [255 255 0]
SegoutR(BWoutline) = 255;
SegoutG(BWoutline) = 255;
SegoutB(BWoutline) = 0;
SegoutRGB = cat(3, SegoutR, SegoutG, SegoutB);
figure(7)
imshow(SegoutRGB)
title('Outlined Original Image')

