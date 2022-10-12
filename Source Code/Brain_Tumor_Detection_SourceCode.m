clc;

%INPUT
ext='*.jpg';
folder='C:\Users\Lenovo\Desktop\MATLAB Project\Source Code\Test Images';
image_File=uigetfile([folder '\' ext]);
image_data=imread(image_File);
image=image_data;
image_original=imresize(image_data,[200,200]);
figure(1);
imshow(image_data);
title('Original Image');


%Testing
%image_data=imnoise(image_data,'gaussian',0,0.025);
%image_data = imgaussfilt(image_data,2);


%Reduction of Noise
num_iter = 10;
 delta_t = 1/7;
 kappa = 15;
 option = 2;
 image_data = anisodiff(image_data,num_iter,delta_t,kappa,option);
 image_data= uint8(image_data);
figure(2);
imshow(image_data);
title('Reduce Noise');


%Binary Convertion
BW=im2bw(image_data,0.7);
label=bwlabel(BW);
figure(7);
imshow(BW);
title('Binary Image');



%Finding Tumor
stats=regionprops(BW,'Solidity','Area');
density=[stats.Solidity];
area=[stats.Area]

high_dens=density>0.6;
max_area=max(area(high_dens));
tumor_label=find(area==max_area);
tumor=ismember(label,tumor_label);

figure(3);
imshow(tumor);
title('Tumor Alone');

se=strel('square',5);
tumor=imdilate(tumor,se);

%Marking Tumor
B=bwboundaries(tumor,'noholes');
figure(4);
imshow(image);
hold on;
for i=1:length(B)
    plot(B{i}(:,2),B{i}(:,1),'red','linewidth',2);
   
end;
title('Tumor Region');
hold off;

%Findimg Brain
BW1=im2bw(image_data,0.2);
label1=bwlabel(BW1);
stats1=regionprops(BW1,'Solidity','Area');
density1=[stats1.Solidity];
area1=[stats1.Area];

high_dens1=density1>0.6;
max_area1=max(area1(high_dens1));
brain_label=find(area1==max_area1);
brain=ismember(label1,brain_label);

se1=strel('square',5);
brain=imdilate(brain,se1);
%figure(5);
%imshow(brain);
%title('Brain Alone');

%Classification
Img=image_original;
Img = im2bw(Img,0.5);
img_matrix = Img(:,:);
[cA1,cH1,cV1,cD1] = dwt2(img_matrix,'db4');
[cA2,cH2,cV2,cD2] = dwt2(cA1,'db4');
[cA3,cH3,cV3,cD3] = dwt2(cA2,'db4');

DWT_feat=[cA3,cH3,cV3,cD3];
G=pca(DWT_feat);
g=graycomatrix(G);
stats=graycoprops(g,'Contrast Correlation Energy Homogeneity');
Contrast = stats.Contrast;
Correlation = stats.Correlation;
Energy = stats.Energy;
Homogeneity = stats.Homogeneity;
Mean = mean2(G);
Standard_Deviation = std2(G);
Entropy = entropy(G);
RMS = mean2(rms(G));

Variance = mean2(var(double(G)));
a = sum(double(G(:)));
Smoothness = 1-(1/(1+a));
Kurtosis = kurtosis(double(G(:)));
Skewness = skewness(double(G(:)));
% Inverse Difference Movement
m = size(G,1);
n = size(G,2);
in_diff = 0;
for i = 1:m
    for j = 1:n
        temp = G(i,j)./(1+(i-j).^2);
        in_diff = in_diff+temp;
    end
end
IDM = double(in_diff);
    
feat = [Contrast,Correlation,Energy,Homogeneity, Mean, Standard_Deviation, Entropy, RMS, Variance, Smoothness, Kurtosis, Skewness, IDM];

load Trainset.mat
 xdata=meas;
 group=label;
 svmStruct1=svmtrain(xdata,group,'kernel_function', 'linear');
 species = svmclassify(svmStruct1,feat,'showplot',false)
