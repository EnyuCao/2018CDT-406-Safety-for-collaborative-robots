%This script is used to test the trained CNN with a live camera feed, the
%scipt uses the camera on a laptop and shows the different outputs from the
%net. If the scipt is stopped for some reason, you might have to manually
%stop the videoinput by typing "stop(x)"

load FaceNoFaceNet.mat

x = videoinput('winvideo', 1);
triggerconfig(x, 'manual');
start(x);

tic;
for i = 1:1000
    
    %Gets a snapshot from camera stream and resizes it to fit CNN 
    img = getsnapshot(x);
    img = imcrop(img, [280 1 719 719]);
    img = imresize(img, [227 227]);
    img = flipdim(img, 2);
    
    %Stop the program by blocking the camera
    if mean(mean(img)) < 50
        Etime = toc/i;
        disp('Stopped');
        break;
    end
    
    [output, scores] = classify(myNet, img);
    
    output = char(output);
    imshow(img);
    text(90, 10, output);
    drawnow;
end
Etime = toc/i;
disp('Stopped');
stop(x);
delete(x);
close all;