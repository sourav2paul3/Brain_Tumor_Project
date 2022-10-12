function varargout = Brain_Tumor_Detection_GUI(varargin)
% BRAIN_TUMOR_DETECTION_GUI MATLAB code for Brain_Tumor_Detection_GUI.fig
%      BRAIN_TUMOR_DETECTION_GUI, by itself, creates a new BRAIN_TUMOR_DETECTION_GUI or raises the existing
%      singleton*.
%
%      H = BRAIN_TUMOR_DETECTION_GUI returns the handle to a new BRAIN_TUMOR_DETECTION_GUI or the handle to
%      the existing singleton*.
%
%      BRAIN_TUMOR_DETECTION_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BRAIN_TUMOR_DETECTION_GUI.M with the given input arguments.
%
%      BRAIN_TUMOR_DETECTION_GUI('Property','Value',...) creates a new BRAIN_TUMOR_DETECTION_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Brain_Tumor_Detection_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Brain_Tumor_Detection_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Brain_Tumor_Detection_GUI

% Last Modified by GUIDE v2.5 06-Jan-2021 12:53:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Brain_Tumor_Detection_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Brain_Tumor_Detection_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Brain_Tumor_Detection_GUI is made visible.
function Brain_Tumor_Detection_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Brain_Tumor_Detection_GUI (see VARARGIN)

% Choose default command line output for Brain_Tumor_Detection_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

global image1 image2 image3
image1=[];
image2=[];
image3=[];
% UIWAIT makes Brain_Tumor_Detection_GUI wait for user response (see UIRESUME)
% uiwait(handles.dfdf);


% --- Outputs from this function are returned to the command line.
function varargout = Brain_Tumor_Detection_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image1 image2 image3
[path,nofile]=imgetfile();
if nofile
    warndlg(sprintf('Image not found'),'Error');
    return;
end;
image1=imread(path);

image2 =imresize(image1,[200,200]);
guidata(hObject,handles);
axes(handles.axes1);
imshow(image1);
title('Brain MRI');
image3=image1;


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image1 image2 image3

if(isempty(image1))
    warndlg(sprintf('Image not found'),'Error');
    return;

else
    %Noise Reduction
    num_iter = 10;
     delta_t = 1/7;
     kappa = 15;
     option = 2;
     image1 = anisodiff(image1,num_iter,delta_t,kappa,option);
     image1= uint8(image1);


    axes(handles.axes2);

    %Binary Convertion
    BW=im2bw(image1,0.7);
    label=bwlabel(BW);

    %Finding Tumor & Size
    stats=regionprops(BW,'Solidity','Area');
    density=[stats.Solidity];
    area=[stats.Area];

    high_dens=density>0.6;
    max_area = max(area(high_dens));
    tumor_label=find(area == max_area);
    tumor=ismember(label,tumor_label);

    se=strel('square',5);
    tumor=imdilate(tumor,se);

    axes(handles.axes3);
    imshow(tumor);
    title('Tumor alone');

    axes(handles.axes2);

    %Marking Tumor
    [B,L]=bwboundaries(tumor,'noholes');
    imshow(image3);
    hold on;

    for i=1:length(B)
        plot(B{i}(:,2),B{i}(:,1),'red','linewidth',2);
    end;
    title('Tumor Region');
    hold off;

    %Classification
    Img=image2;
    Img = im2bw(Img);
    guidata(hObject,handles);
    signal1 = Img(:,:);

    [cA1,cH1,cV1,cD1] = dwt2(signal1,'db4');
    [cA2,cH2,cV2,cD2] = dwt2(cA1,'db4');
    [cA3,cH3,cV3,cD3] = dwt2(cA2,'db4');

    DWT_feat = [cA3,cH3,cV3,cD3];
    G = pca(DWT_feat);
    g = graycomatrix(G);
    stats = graycoprops(g,'Contrast Correlation Energy Homogeneity');
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
     xdata = meas;
     group = label;
     svmStruct1 = svmtrain(xdata,group,'kernel_function', 'linear');
     species = svmclassify(svmStruct1,feat,'showplot',false);
     set(handles.edit1,'string',species);
end;
 


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global image1 image2 image3

%Removing Image Matrices 
image1=[];
image2=[];
image3=[];

%Reseting the Axis
axes(handles.axes1);
cla(handles.axes1,'reset');
set(gca,'Color',[0.83,0.82,0.78]);
set(gca,'XColor','none','YColor','none','TickDir','out');

axes(handles.axes2);
cla(handles.axes2,'reset');
set(gca,'Color',[0.83,0.82,0.78]);
set(gca,'XColor','none','YColor','none','TickDir','out');

axes(handles.axes3);
cla(handles.axes3,'reset');
set(gca,'Color',[0.83,0.82,0.78]);
set(gca,'XColor','none','YColor','none','TickDir','out');

set(handles.edit1,'String','')
