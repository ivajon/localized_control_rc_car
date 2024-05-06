cam = webcam(1);
cam.Resolution = "640x480";
sampleRate = 10;
captureTime = 10;
numImages = sampleRate*captureTime;
ImageFolder = pwd+"\capture3";

numberSize = strlength(sprintf("%d",numImages));

for k=1:numImages % this loop will take 5 pictures and save them in the Matlab folder 
    img = snapshot(cam);
    number = sprintf("%d",k);
    numLen = numberSize-strlength(number);
    pad = "";
    for i = 1:numLen
        pad = append(pad,"0");
    end
    file_name = "Image"+pad+number+".png";% name Image with a sequence of number, ex Image1.png , Image2.png....
    %save the image as a Portable Graphics Format file(png)into the MatLab
    imgName = fullfile(ImageFolder,file_name) ;
    tic
    imwrite(img,imgName);
    toc
    % imshow(img) % display the image for every second
    %pause(1/sampleRate); % pause for one second
end