clear all;
close all;
clc;
% Some times there is no need to import raw data because all data will
% save in FinalDatafor first time. readflage=1 means import files again.
readflage= 0;
% folder=uigetdir(); % get Data directory
SubjectNumber='T002';
Project='P006';
folder=append('C:\MyCloud\OneDriveUcf\Real\Simulation\Source\',Project,'\',SubjectNumber);
Datafolder=append(folder,'\Data');
results_folder = append(folder,'\Result');
Pardata=importdata(append(Datafolder,"\","Parameters.csv"));
ResultData.info.ForceRatio=Pardata.data(1);
ResultData.info.M_ThresholdMin=Pardata.data(2);
ResultData.info.M_ThresholdMax=Pardata.data(3);
psname=append(Project,'_',SubjectNumber);
Jointname='RKnee';
Subjectname =append(psname,"_",Jointname);
Terials1=["Fl"];
Terials2=["H90","H70","H55","H40","H25","H10"];
Terials3=["iter1","iter2","iter3"];
% Terials1=["Fl"];
% Terials2=["IsoK60"];
ArmWeight=2.72;
ArmCOM=0.218;
Fdata=[];
Gdata=[0];
k=0;
% DStime=0.0005192; % desired sampling time
DStime=0.01;
%
ExpMuscle=["RBICF","RSEMT","RMGAS","RRECF","RVASL","RVASM"];

if readflage
    for T1=1:length(Terials1)
        for T2=1:length(Terials2)
            ww=[];
            ts=[];
            te=[];
            filename=append(Subjectname,'_',Terials1(T1),'_',Terials2(T2));
            Datadr=append(Datafolder,"\",Subjectname,'_',Terials1(T1),'_',Terials2(T2),'_Q.csv');
            data=importdata(Datadr);
            for ii=2:2:length(data.textdata)
                kk=1;
                tflage=0;
                ww=0;
                for jj=1:length(data.data)%# of data points in a given trial
                    if ~isnan(data.data(jj,ii))&&data.data(jj,ii)~=0
                        kk=jj;  %finding zero data at the end of each chanel
                        tflage=1;
                    elseif tflage==0
                        ww=jj;  %finding zero data at the begining
                    end
                end
                ts(ii/2)=data.data(ww+1,ii-1); % first time of first chanel to set as final time for every other channel.
                te(ii/2)=data.data(kk,ii-1); % final time of first chanel to set as final time for every other channel.
            end
%%             finding the highest starting 
            tsfinal=max(ts);
            tefinal=min(te);
            interpolatetime=tsfinal:DStime:tefinal;
            for ii=2:2:length(data.textdata)    
               
                starttimeinx=find(data.data(:,ii-1)>= tsfinal & data.data(:,ii-1)<=tefinal);    
                y=interp1(data.data(starttimeinx,ii-1),data.data(starttimeinx,ii),interpolatetime,'linear','extrap'); %Interpolates data to match sampling time to desierd sampling time
                
                b=y';
                %                 ends(ii)=length(b);
                if (size(Gdata(:,1)) == 1) %recombines data into a matrix padded with NaN
                    Gdata = [interpolatetime' b];
                else
                    Gdata = [Gdata b];
                end
                
            end
            FinalData.(filename).data=Gdata;
            FinalData.(filename).colheaders=["time" data.textdata(2:2:end)];
            clear Gdata
            Gdata=[0];
            
        end
    end
    
    save ([Datafolder '\RawData.mat'],'FinalData');
    
end
%%
load ([Datafolder '\RawData.mat']);
Dataheadermotion=["time","pelvis_tilt","pelvis_tx","pelvis_ty","hip_flexion_r","hip_adduction_r","hip_rotation_r","knee_angle_r","subtalar_angle_r","ankle_angle_r"];
Dataheaderforce=["time","reaction_force_vx","reaction_force_vy","reaction_force_vz","reaction_force_px","reaction_force_py","reaction_force_pz","reaction_torque_x","reaction_torque_y","reaction_torque_z"];

%getting goniometer calibration coefficient
[Ph,Pk,Pa]= GnCalib(Datafolder,psname,1);
for T1=1:length(Terials1)
    for T2=1:length(Terials2)
        EMGHDdata=[""];
        filename=append(Subjectname,'_',Terials1(T1),'_',Terials2(T2));
        Data=FinalData.(filename).data;
        HData=FinalData.(filename).colheaders;
        [rg,ca]=find(strncmp(HData,'Gn A',4));
        %find Knee Goniometer
        [rk,ck]=find(strncmp(HData,'Gn K',4));
        %find Hip goniometer
        [rh,ch]=find(strncmp(HData,'Gn H',4));
        %find Biodex
        [rb,cb]=find(strncmp(HData,'Biodex',6));
        %find EMG
        [re,ce]=find(contains(HData,'EMG')&~contains(HData,'RMS'));
        [r,c]=size(Data);
        %% Process on Motion Data
        
        %knee calibration
        GonK=Data(:,ck(2));
        GonCalibratedK = polyval(Pk,GonK);
        GonCalibratedK(GonCalibratedK<0)=0;
        %Hip calibration
        GonH=Data(:,ch(2));
        GonCalibratedH = polyval(Ph,GonH);
        %Ankle calibration
        GonA=Data(:,ca(2));
        GonCalibratedA = polyval(Pa,GonA);

        %% Save Motion
        delimiterIn='\t';
        F_fnames=append(Subjectname,char(filename),'_Motion.mot');
%         MDatadata=[1,0,0.055,1.059,1,0,0,1,0,0].*ones(r,10);
        MTable=[Data(:,1),GonCalibratedH,GonCalibratedK,GonCalibratedA,Data(:,cb(2))];
        Titledata=[r,length(MTable(1,:))];
%         makefile(Datafolder,F_fnames,Dataheadermotion,MDatadata,5,delimiterIn);    
        %% Process Force
        %%% Caculating Torque from Arm
        BiodexAngle=-35.55*Data(:,cb(2))+105;
        ArmTorque=cos(BiodexAngle*pi()/180)*ArmWeight*9.8*ArmCOM;
        %%% Calcuating Torque from biodex
        x=1*Data(:,cb(1)); %data of a trial
        TotalTorque=-1.*(141.81.*x-25.047);
        %%% Arm weight compensation 
        Mb=TotalTorque-ArmTorque;
        %% Save Force
        F_fnames=append(Subjectname,char(filename),'_Torque.mot');
        FDatadata=[Data(:,1),zeros(r,8),Mb];
        Titledata=[r,length(FDatadata(1,:))];
%         makefile(Datafolder,F_fnames,Dataheaderforce,FDatadata,5,delimiterIn);
        
        
        %% Finding events
        Event=EventDetection(filename,DStime,FDatadata(:,10),ResultData.info.ForceRatio,MTable,[ResultData.info.M_ThresholdMin ResultData.info.M_ThresholdMax]);
        Stime=Event(:,1);
        Etime=Event(:,2);
        %% Trail check
        if length(Stime)~=3||length(Etime)~=3
            fprintf('\nERROR: %s Wrong trail ...\n\n', filename);
        end
        %% Strat reading Simulation files
        ResultData.(filename).('ExpForce').('full')=[Data(:,1),Mb];
        ResultData.(filename).('Motion').('full')=[Data(:,1),GonCalibratedK];

        for itr=1:length(Stime)
            Expindx=find(Data(:,1)>=Stime(itr)&Data(:,1)<=Etime(itr));
            ResultData.(filename).('time').Exp.(Terials3(itr))=Data(Expindx,1);
            ResultData.(filename).('Motion').(Terials3(itr))=ResultData.(filename).Motion.full(Expindx,2);
            ResultData.(filename).('ExpForce').(Terials3(itr))=ResultData.(filename).ExpForce.full(Expindx,2);
        end
        
    end
end
save (append(results_folder,"\",psname,"_ResultData.mat"),'ResultData');

