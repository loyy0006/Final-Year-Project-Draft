% Documentation from https://www.mathworks.com/help/images/detecting-a-cell-using-image-segmentation.html
clc;    
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
clear; 
%% Read Image
I = imread('gum.jpg');
subplot(2,2,1);
imshow(I)
title('Original Image');

%% Binarize
I=rgb2gray(I); %Convert to gray scale 
BW = imbinarize(I);
subplot(2,2,2);
imshow(BW)
title('Original Image Converted to Binary Image')


%% Visualize the segmentation
%Mask
subplot(2,2,3);
maskImage = labeloverlay(I,BW);
imshow(maskImage)
title('Mask Over Original Image')

% Outline the perimeter 
BWoutline = bwperim(BW);
BWoutline = imdilate(BWoutline, strel('disk',1)); %change the thickness of the border

SegoutR = I;
SegoutG = I;
SegoutB = I;
%now set yellow, [255 255 0]
SegoutR(BWoutline) = 255;
SegoutG(BWoutline) = 255;
SegoutB(BWoutline) = 0;
SegoutRGB = cat(3, SegoutR, SegoutG, SegoutB);

subplot(2,2,4);
imshow(SegoutRGB)
title('Outlined Original Image')

% Area calculation in pixels
props = regionprops(BW, 'Area');
allAreas = [props.Area]
