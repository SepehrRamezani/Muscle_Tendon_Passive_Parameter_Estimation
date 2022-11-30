clear all;
close all;
clc;
% Some times there is no need to import raw data because all data will
% save in FinalDatafor first time. readflage=1 means import files again.
readflage= 1;
% folder=uigetdir(); % get Data directory
SubjectNumber='T003';
Project='P006';

folder=append('C:\MyCloud\OneDriveUcf\Real\Simulation\Source\',Project,'\',SubjectNumber);
Datafolder=append(folder,'\Data');
results_folder = append(folder,'\Result');
Pardata=importdata(append(Datafolder,"\","Parameters.csv"));
ResultData.info.ForceRatio=Pardata.data(1);
ResultData.info.M_ThresholdMin=Pardata.data(2);
ResultData.info.M_ThresholdMax=Pardata.data(3);
optForce=Pardata.data(6);
psname=append(Project,'_',SubjectNumber);
Jointname='LKnee';
Subjectname =append(psname,"_",Jointname);


Terials1=["Fl"];
Terials2=["H90","H70","H55","H40","H25","H10"];
Terials3=["iter1","iter2","iter3"];
% Terials1=["Fl"];
% Terials2=["IsoK60"];
ArmWeight=2.72;
ArmCOM=0.27;
Fdata=[];
k=0;
% DStime=0.0005192; % desired sampling time
DStime=0.5;
%

if readflage
    for T1=1:length(Terials1)
        for T2=1:length(Terials2)
            filename=append(Subjectname,'_',Terials1(T1),'_',Terials2(T2));
            Datadr=append(Datafolder,"\",Subjectname,'_',Terials1(T1),'_',Terials2(T2),'_Q.csv');
            data=importdata(Datadr);
            [Gdata,Gheader]= ReshapingData(data,DStime);
            FinalData.(filename).data=Gdata;
            FinalData.(filename).colheaders=Gheader;
        end
    end
    
    save ([Datafolder '\RawData.mat'],'FinalData');
    
end
%%
load ([Datafolder '\RawData.mat']);
delimiterIn='\t';
% joints=["hip","walker_knee","patellofemoral","ankle","mtp","subtalar"];
% Data.joints(2:end)=makingstr(Data.joints(2:end),Data.whichleg);
Dataheadermotion=['time' delimiterIn '/jointset/hip_r/hip_flexion_r/value' ...
    delimiterIn '/jointset/walker_knee_r/knee_angle_r/value' ...
    delimiterIn '/jointset/ankle_r/ankle_angle_r/value' ...
    delimiterIn 'BiodexAngle'];
Dataheaderforce=['time' delimiterIn '/forceset/knee_angle_r_act'];
if contains(Jointname,'LKnee')
    Dataheadermotion=strrep(Dataheadermotion,'_r','_l');
    Dataheaderforce=strrep(Dataheaderforce,'_r','_l');
end

TitleM='\nversion=1\nnRows=%d\nnColumns=%d\ninDegrees=no\nendheader\n';
Title='\ninDegrees=no\nnum_controls=1\nnum_derivatives=0\nDataType=double\nversion=3\nnRows=%d\nnColumns=%d\nendheader\n';

%getting goniometer calibration coefficient

[Ph,Pk,Pa,P_Bidoex_Calibration,Torqueref]= GnCalib(Datafolder,psname,DStime,0);

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
        %Biodex angle 
        BiodexAngle=(-35.5*Data(:,cb(2))+103)*pi()/180;
        MTable=[Data(:,1),GonCalibratedH,GonCalibratedK,GonCalibratedA,BiodexAngle];
        %% Finding events
        Event=EventDetection(filename,DStime,MTable,[ResultData.info.M_ThresholdMin ResultData.info.M_ThresholdMax]);
        Sindx=Event(:,1);
        Eindx=Event(:,2);
        %% Save Motion        
        F_fnames=append(char(filename),'_Motion.mot');
        TrimMTable=MTable(Sindx:Eindx,:);
        %%% removing offset 
        TrimMTable(:,1)=TrimMTable(:,1)-TrimMTable(1,1);
        [TMr,TMc]=size(TrimMTable);
        Titledata=[TMr TMc];
        makefile(Datafolder,F_fnames,TitleM,Titledata,Dataheadermotion,TrimMTable,5,delimiterIn);    
        %% Process Force
        %%% Caculating Torque from Arm
        % ArmTorque=cos(BiodexAngle*pi()/180)*ArmWeight*9.8*ArmCOM;
        ArmTorque=cos(BiodexAngle)*Torqueref;
        %%% Calcuating Torque from biodex
        x=1*Data(:,cb(1)); %data of a trial
        TotalTroque=polyval(P_Bidoex_Calibration,x);
        KneeTorque=TotalTroque-ArmTorque;
        
        KneeControl=KneeTorque/optForce;
        %% Save Force
        F_fnames=append(char(filename),'_Torque.mot');
        FData=[Data(Sindx:Eindx,1),KneeControl(Sindx:Eindx,1)];
        [TFr,TFc]=size(FData);
        Titledata=[TFr TFc];
        makefile(Datafolder,F_fnames,Title,Titledata,Dataheaderforce,FData,7,delimiterIn);      
        

        %% Trail check
%         if length(Stime)~=3||length(Etime)~=3
%             fprintf('\nERROR: %s Wrong trail ...\n\n', filename);
%         end
        %% Strat reading Simulation files
        ResultData.(char(filename)).('ExpTorque').('full')=FData;
        ResultData.(char(filename)).('ExpMotion').('full')=TrimMTable;
        

%         for itr=1:length(Stime)
%             Expindx=find(Data(:,1)>=Stime(itr)&Data(:,1)<=Etime(itr));
%             ResultData.(filename).('time').Exp.(Terials3(itr))=Data(Expindx,1);
%             ResultData.(filename).('Motion').(Terials3(itr))=ResultData.(filename).Motion.full(Expindx,2);
%             ResultData.(filename).('ExpForce').(Terials3(itr))=ResultData.(filename).ExpForce.full(Expindx,2);
%         end
        
    end
end
save (append(results_folder,"\",psname,"_ResultData.mat"),'ResultData');

