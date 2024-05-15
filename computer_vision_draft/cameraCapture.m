cam = webcam(1);
cam.Resolution = "640x480";

ImageFolder = pwd+"\Circles";
for k=1:100 % this loop will take 5 pictures and save them in the Matlab folder 
    img = snapshot(cam);
    file_name = sprintf('Image%d.png',k)% name Image with a sequence of number, ex Image1.png , Image2.png....
    %save the image as a Portable Graphics Format file(png)into the MatLab
    imgName = fullfile(ImageFolder,file_name) ;
    imwrite(img,imgName);
    imshow(img) % display the image for every second
    pause(0.5); % pause for one second
end
