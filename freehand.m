%https://www.mathworks.com/matlabcentral/answers/68289-show-a-roi-over-the-original-image


clc;  % Clear command window.
workspace;  % Make sure the workspace panel is showing.
fontSize = 12;

I = imread('incisors.JPG');
imshow(I, []);
axis on;
message = sprintf('Left click and hold to begin drawing.\nSimply lift the mouse button to finish'); %pop up dialogue
uiwait(msgbox(message));
hFH = imfreehand();

% Create a binary image ("mask") from the ROI object.
maskImage = hFH.createMask();
xy = hFH.getPosition;
subplot(1, 3, 1);
imshow(I, []);
axis on;
drawnow;
title('Original Image', 'FontSize', fontSize);

% Display the freehand mask.
subplot(1, 3, 2);
imshow(maskImage);
axis on;
title('Binary mask of the region', 'FontSize', fontSize);

%Convert original to grayscale
grayImage = rgb2gray(I);

% Extract the red, green, and blue channels from the color image.
redChannel = I(:, :, 1);
greenChannel = I(:, :, 2);
blueChannel = I(:, :, 3);
% Create a new color channel images for the output.
outputImageR = grayImage;
outputImageG = grayImage;
outputImageB = grayImage;
% Transfer the colored parts.
outputImageR(maskImage) = redChannel(maskImage);
outputImageG(maskImage) = greenChannel(maskImage);
outputImageB(maskImage) = blueChannel(maskImage);
% Convert into an RGB image
outputRGBImage = cat(3, outputImageR, outputImageG, outputImageB);
% Display the output RGB image.
subplot(1, 3, 3);
imshow(outputRGBImage);
axis on;
title('Output RGB Image', 'FontSize', fontSize);