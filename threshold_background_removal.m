% https://www.mathworks.com/matlabcentral/answers/386250-how-can-i-remove-the-background-this-image-i-do-thresholding-but-i-can-t-remove-the-background-i

clc;
clear all;
close all;
fontSize = 10;

%Original Image
I = imread('incisors.JPG');

subplot(3, 2, 1);
imshow(I, []);
title('Original Image', 'FontSize', fontSize, 'Interpreter', 'None');
axis on;
hp = impixelinfo();

% Grayscale Image
grayImage = rgb2gray(I); 
% ALTERNATE METHOD: Convert it to gray scale by taking only the green channel,
% which in a typical snapshot will be the least noisy channel.
%grayImage = I(:, :, 3); % Take blue channel.
subplot(3, 2, 2);
imshow(grayImage, []);
title('Gray Scale Image', 'FontSize', fontSize, 'Interpreter', 'None');
axis on;

% Display the histogram to gauge threshold
subplot(3, 2, [3 4]);
imhist(grayImage);
grid on;
title('Histogram of Grayscale Image', 'FontSize', fontSize, 'Interpreter', 'None');
hold on;

% Define a threshold (capture everything)
upperlimit = 95; 
lowerlimit = 15; 

% Put up line over histogram at that level
line([lowerlimit, lowerlimit], ylim, 'Color', 'r', 'LineWidth', 2);
line([upperlimit, upperlimit], ylim, 'Color', 'r', 'LineWidth', 2);
hold on;

% Define a threshold (background removal)
upperlimit1 = 105; 
lowerlimit1 = 15;

% Put up line over histogram at that level
line([lowerlimit1, lowerlimit1], ylim, 'Color', 'b', 'Linestyle', '--', 'LineWidth', 2);
line([upperlimit1, upperlimit1], ylim, 'Color', 'b', 'Linestyle', '--', 'LineWidth', 2);
caption = sprintf("" + lowerlimit + " < t < " + upperlimit);
caption2 = sprintf("" + lowerlimit1 + " < t < " + upperlimit1);
L(1) = plot(nan, nan, 'r-'); hold on;   
L(2) = plot(nan, nan, 'b--');
legend(L, {caption, caption2},'Location','northeast')
hold off;

% Determining upper and lower limit
% Replace threshold value to 0 (black) 
Ia = I;
for i = 1:size(I,1)
    for j = 1:size(I,2)
        if I(i,j,1) >= lowerlimit && I(i,j,1) <= upperlimit   
            Ia(i,j,1) = 0;  
            Ia(i,j,2) = 0;
            Ia(i,j,3) = 0;
        end
    end 
end 
subplot(3, 2, 5);
imshow(Ia, []);
caption = sprintf("Background Removal (" + lowerlimit + " < t < "+ upperlimit + ")");
title(caption, 'FontSize', fontSize, 'Interpreter', 'None');
axis on;

% Remove background
% Replace threshold value to 0 (black) 
Ib = I;

for i = 1:size(I,1)
    for j = 1:size(I,2)
        if I(i,j,1) >= lowerlimit1 && I(i,j,1) <= upperlimit1   
            Ib(i,j,1) = 0;  
            Ib(i,j,2) = 0;
            Ib(i,j,3) = 0;
        end
    end 
end 

subplot(3, 2, 6);
imshow(Ib, []);
caption = sprintf("Background Removal (" + lowerlimit1 + " < t < "+ upperlimit1 + ")");
title(caption, 'FontSize', fontSize, 'Interpreter', 'None');
axis on;

% % Get binary image
% binaryImage = (lowerlimit <= grayImage) & (grayImage <= upperlimit);

% % Fill holes.
%binaryImage = imfill(binaryImage, 'holes');

% % Get rid of blobs less than 100 pixels in area
% binaryImage = bwareaopen(binaryImage, 200);
% subplot(3, 2, 5);
% imshow(binaryImage, []);
% caption = sprintf('Binary Image Using Threshold');
% title(caption, 'FontSize', fontSize, 'Interpreter', 'None');
% axis on;

% % Mask over original image 
% maskImage = labeloverlay(I,binaryImage);
% subplot(3, 2, 6);
% imshow(maskImage)
% caption = sprintf('Masked Image After Threshold');
% title(caption, 'FontSize', fontSize, 'Interpreter', 'None');
% axis on;
