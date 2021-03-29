%https://www.mathworks.com/help/images/color-based-segmentation-using-k-means-clustering.html%

image = imread('incisors_removed.jpg');
figure(1);
imshow(image), title('Original Image');

lab_image = rgb2lab(image);

ab = lab_image(:,:,2:3);
ab = im2single(ab);
nColors = 3;
% repeat the clustering 3 times to avoid local minima
pixel_labels = imsegkmeans(ab,nColors,'NumAttempts',3);


figure(2);
imshow(pixel_labels,[])
title('Image Labeled by Cluster Index');

figure(3);
mask1 = pixel_labels==1;
cluster1 = image .* uint8(mask1);
imshow(cluster1)
title('Objects in Cluster 1');

figure(4);
mask2 = pixel_labels==2;
cluster2 = image .* uint8(mask2);
imshow(cluster2)
title('Objects in Cluster 2');

figure(5);
mask3 = pixel_labels==3;
cluster3 = image .* uint8(mask3);
imshow(cluster3)
title('Objects in Cluster 3');

% figure(6);
% mask4 = pixel_labels==4;
% cluster4 = image .* uint8(mask4);
% imshow(cluster4)
% title('Objects in Cluster 4');




