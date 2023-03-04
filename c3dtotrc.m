clear all
txtdir=['C:\MyCloud\OneDriveUcf\Real\Simulation\Passive_Parameter_prediction\Robotic_Leg\Data\Second'];
listing = dir(txtdir);
names=[];
Title='\ninDegrees=no\nnum_controls=1\nnum_derivatives=0\nDataType=double\nversion=3\nnRows=%d\nnColumns=%d\nendheader\n';
Torqueflage=0;
for i=1:length(listing)
    curname=string(listing(i,1).name);
%     if contains(curname,"Trial")&~contains(curname,"Knee")&~contains(curname,"Marker")&~contains(curname,"Torque")
    if contains(curname,".c3d")
        names=[names; curname];
    end
end

% Knee=filedata.Knee;
% Ankle=filedata.Ankle;
% Trial=filedata.Trial;
% Subject=filedata.Subject;
filedata.trial=[];
newFPS=100;
RMatrix=[0 0 1; ...
    1 0 0; ...
    0 1 0];
% angle=[]
for S=1:length(names)
    %             trial_name=char(filedata.trialas(S));
    %             indxuderline=strfind(trial_name,'_');
    %             Subject=trial_name(1:indxuderline(1)-1);
    %             fname=erase(trial_name,append(Subject,"_"));
    %             Trc_path=append(filedata.Basepath,'\Moca\',Subject,'\');
    %             fname=append(Knee(K),"_",Ankle(A),"_L_",Trial(T));
    %             fullname=filedata.trialas(S);
    filedir=char(fullfile(txtdir,names(S)));
    markdatastruct = c3d_getdata(filedir, 0);
   
    oldFPS=markdatastruct.marker_data.Info.frequency;
    Markerset=fieldnames(markdatastruct.marker_data.Markers);
    Markerset=Markerset(~contains(Markerset,'C_'));
    % checking if US5 and US 4 is swapped
    MarkerData=[];
    MarkerDatare=[];
    rm=length(Markerset);
    Markersetnew=Markerset;
    [endtimeindx,cc]=size(markdatastruct.marker_data.Markers.(Markerset{1}));
    for i = 1:rm
        RawMarker=markdatastruct.marker_data.Markers.(Markerset{i})*RMatrix;
        zeronumbers=find(RawMarker==0);
        if zeronumbers>1
            endtimeindx=min(zeronumbers(1)-1,endtimeindx);
        end
        if isempty(zeronumbers) zeronumbers=0; end
        if zeronumbers(1)~=1
            [bb,aa] = butter(2, 0.05,'low');
            MarerDatafilt=filtfilt(bb,aa,RawMarker);
            MarkerData =[MarkerData MarerDatafilt];
        else
            MarkerDatare=[MarkerDatare string(Markerset{i})];
            fprintf('Warning: %s has removed from %s \n',string(Markerset{i}),fullname);
            Markersetnew=Markerset(~contains(Markerset,MarkerDatare));
        end
    end
    markerinfo=markdatastruct.marker_data.Info;
    frames_oldFPS=markerinfo.First_Frame:markerinfo.Last_Frame;
    time_oldFPS=frames_oldFPS/oldFPS;
    endframe_oldFPS=frames_oldFPS(endtimeindx);
    Etime=time_oldFPS(endtimeindx);
    Stime=time_oldFPS(1);
    time_newFPS=Stime:1/newFPS:Etime;
    MarkerData_newFPS=interp1(time_oldFPS,MarkerData,time_newFPS,'linear');
    [r,c]=size(MarkerData_newFPS);

    markdatastruct.marker_data.Info.frequency=newFPS;
    if markdatastruct.marker_data.Info.First_Frame>1
        fprintf('Warning: First frame of %s changed to 1 \n',names(S));
    end

    markdatastruct.marker_data.Time= time_newFPS;
    MarkerData_newFPS=[time_newFPS' MarkerData_newFPS];
    markerinfo.Last_Frame=r;
    markerinfo.First_Frame=1;
    markerinfo.NumFrames=markerinfo.Last_Frame-markerinfo.First_Frame+1;
    generate_Marker_Trc(Markersetnew,MarkerData_newFPS,markerinfo);
    %save force
    delimiterIn='\t';
    Dataheaderforce=['time' delimiterIn '/forceset/knee_act'];
%     calibration equation for voltage to torque
if Torqueflage
    Torque_volt=[markdatastruct.analog_data.Time  markdatastruct.analog_data.Channels.Sensor_16_EMG16];
    Torqueinterp=interp1(Torque_volt(:,1),Torque_volt(:,2),time_newFPS','linear','extrap'); %Interpolates data to match sampling time to desierd sampling time
%     Torque=(Torqueinterp-0.168)*2/0.168*67;
    TorqueData=[time_newFPS' Torqueinterp];
    [TFr,TFc]=size(TorqueData);
    Titledata=[TFr,TFc];
    F_fnames= strrep(names(S),'.c3d','_Torque.mot');
     makefile(txtdir,F_fnames,Title,Titledata,Dataheaderforce,TorqueData,7,delimiterIn);
end
end
