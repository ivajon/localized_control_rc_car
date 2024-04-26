%Simple comparison of sample images in different color spaces.
%The enviroment is basically gray. Use grayscale images?

imds = imageDatastore("capture2\"); %Get images

for i = 1:19
    Irgb = readimage(imds,i);  %Read images and convert from rgb to other color spaces
    Irgb = imresize(Irgb,0.2);
    Iy = rgb2ycbcr(Irgb);
    Ihsv = rgb2hsv(Irgb);

    %Display all channels side-by-side
    figure(1)
    imshow(separateColorImage(Ihsv));
    title("HSV")

    figure(2)
    imshow(separateColorImage(Iy));
    title("YCbCr")

    figure(3)
    imshow(separateColorImage(Irgb));
    title("RGB")
end



function separatedImage = separateColorImage(I)
        separatedImage = [I(:,:,1) I(:,:,2) I(:,:,3)];
end

